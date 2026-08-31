#include "map_update_executor.hpp"

#include <algorithm>
#include <limits>
#include <optional>

#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/execution/algorithm_context.hpp>
#include <open_lmm/server/nodes/map_update_node.hpp>
#include <open_lmm/server/stage_runner.hpp>

namespace open_lmm {
namespace {

uint64_t EstimateAgentMemory(const std::filesystem::path& directory) {
  uint64_t bytes = 1;
  std::error_code error;
  std::filesystem::recursive_directory_iterator iterator(
      directory, std::filesystem::directory_options::skip_permission_denied,
      error);
  const std::filesystem::recursive_directory_iterator end;
  while (!error && iterator != end) {
    if (iterator->is_regular_file(error)) {
      const uint64_t size = iterator->file_size(error);
      if (!error) {
        if (size > (std::numeric_limits<uint64_t>::max() - bytes) / 2) {
          return std::numeric_limits<uint64_t>::max();
        }
        bytes += size * 2;
      }
    }
    iterator.increment(error);
  }
  return bytes;
}

Result<std::shared_ptr<void>> AcquireHeavyPhase(
    const std::shared_ptr<ResourceGovernor>& governor,
    const std::shared_ptr<CancellationToken>& cancellation) {
  auto acquired = governor->AcquireHeavyMemoryPhase(cancellation);
  if (!acquired) {
    return Result<std::shared_ptr<void>>::Failure(acquired.GetError());
  }
  return Result<std::shared_ptr<void>>::Ok(
      std::shared_ptr<void>(new int(0), [governor](void* value) {
        delete static_cast<int*>(value);
        governor->ReleaseHeavyMemoryPhase();
      }));
}

}  // namespace

Result<ExecutionCandidate> MapUpdateExecutor::Execute(
    MapUpdateExecutionContext input) const {
  return execute(std::move(input), std::nullopt);
}

Result<ExecutionCandidate> MapUpdateExecutor::ExecuteAgent(
    MapUpdateExecutionContext input, const AgentId& agent) const {
  return execute(std::move(input), agent);
}

Result<ExecutionCandidate> MapUpdateExecutor::execute(
    MapUpdateExecutionContext input, std::optional<AgentId> target_agent) const {
  if (!input.committed || !input.committed->config ||
      !input.committed->config->data_loader ||
      !input.committed->config->dynamic_remover || !input.governor ||
      !input.algorithms || !input.pending_files ||
      input.output_directory.empty()) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "MapUpdate execution requires committed config, governor, output, and pending file set"));
  }
  if (StageNodes(StageId::kMapUpdate) !=
      std::vector<NodeId>{NodeId::kMapUpdate}) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("unsupported MapUpdate execution spec"));
  }
  const auto& base_payload = input.committed->payload;
  if (!base_payload || !base_payload->database ||
      !base_payload->optimizer || base_payload->contexts.empty() ||
      base_payload->database->optimized_data.empty()) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before MapUpdate"));
  }
  auto contexts = base_payload->contexts;
  if (target_agent) {
    const auto selected = std::find_if(
        contexts.begin(), contexts.end(), [&](const auto& context) {
          return context.agent.id == *target_agent;
        });
    if (selected == contexts.end()) {
      return Result<ExecutionCandidate>::Failure(
          Error::InvalidArgument("unknown MapUpdate agent"));
    }
    AgentPipelineCtx selected_context = *selected;
    contexts.clear();
    contexts.push_back(std::move(selected_context));
  }
  for (auto& context : contexts) {
    context.flow = ControlFlow::kContinue;
    context.cancellation = input.cancellation;
  }
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base_payload->database->raw_data;
  database->optimized_data = base_payload->database->optimized_data;
  if (!input.committed->config->documents) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("MapUpdate config document is unavailable"));
  }
  ExecutionContext command{input.cancellation, {}, input.committed->revision,
                           input.progress};
  auto data_loader_context = MakeAlgorithmExecutionContext(
      *input.committed, command, {},
      input.committed->config->documents->data_loader,
      "open_lmm.data_loader", "raw_scan_stream",
      input.committed->config->data_loader->type);
  auto remover_context = MakeAlgorithmExecutionContext(
      *input.committed, command, {},
      input.committed->config->documents->dynamic_remover,
      "open_lmm.dynamic_remover", "map_remove",
      input.committed->config->dynamic_remover->model);
  for (const auto& context : contexts) {
    auto destination = input.output_directory /
        ("global_map_" + context.agent.id.Value() + ".pcd");
    auto temporary = destination;
    temporary += ".tmp";
    input.pending_files->Add(std::move(temporary), std::move(destination));
  }

  if (input.parallel) {
    const auto& remover = *input.committed->config->dynamic_remover;
    if (remover.thread_safety !=
        PluginThreadSafety::kInstanceIsolatedParallel) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "parallel MapUpdate requires an instance-isolated remover"));
    }
    const std::size_t internal_threads = remover.internal_cpu_threads;
    if (internal_threads == 0 ||
        internal_threads > input.governor->Budget().max_cpu_threads) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "MapUpdate internal thread count exceeds the CPU budget"));
    }
    const std::size_t concurrency = std::min(
        {std::max<std::size_t>(input.max_parallel_agents, 1), contexts.size(),
         input.governor->Budget().max_agent_tasks,
         input.governor->Budget().max_cpu_threads / internal_threads});
    if (concurrency == 0) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "MapUpdate parallel execution has zero admitted concurrency"));
    }
    for (std::size_t begin = 0; begin < contexts.size();
         begin += concurrency) {
      const std::size_t end = std::min(contexts.size(), begin + concurrency);
      std::vector<BoundedTaskHandle> tasks;
      for (std::size_t index = begin; index < end; ++index) {
        auto submitted = input.governor->AgentExecutor().Submit(
            [&, index]() -> Result<void> {
              const AgentId agent = contexts[index].agent.id;
              auto memory = input.governor->ReserveMemory(
                  EstimateAgentMemory(contexts[index].data_dir),
                  MemoryClass::kHeavyMap, input.cancellation);
              if (!memory) return Result<void>::Failure(memory.GetError());
              DataLoaderConfig loader_config =
                  *input.committed->config->data_loader;
              loader_config.show_progress = false;
              auto loader = input.algorithms->CreateDataLoader(loader_config);
              if (!loader) {
                auto context = data_loader_context;
                context.agent = contexts[index].agent;
                return Result<void>::Failure(
                    WithAlgorithmContext(loader.GetError(), context));
              }
              SharedDatabase isolated;
              const auto optimized = database->optimized_data.find(agent);
              if (optimized == database->optimized_data.end()) {
                return Result<void>::Failure(Error::InvalidArgument(
                    "MapUpdate optimized payload is unavailable"));
              }
              isolated.optimized_data.emplace(agent, optimized->second);
              AgentPipelineCtx isolated_context = contexts[index];
              MapUpdateNode node(
                  std::move(loader).Value(),
                  [algorithms = input.algorithms,
                   config = input.committed->config->dynamic_remover] {
                    return algorithms->CreateDynamicRemover(*config);
                  },
                  input.output_directory.string(), input.save_voxel_size, true,
                  [governor = input.governor,
                   cancellation = input.cancellation] {
                    return AcquireHeavyPhase(governor, cancellation);
                  },
                  data_loader_context, remover_context);
              auto updated = node.Process(isolated_context, isolated);
              if (!updated) return Result<void>::Failure(updated.GetError());
              return Result<void>::Ok();
            },
            input.cancellation);
        if (!submitted) {
          for (const auto& task : tasks) (void)task.Wait();
          return Result<ExecutionCandidate>::Failure(submitted.GetError());
        }
        tasks.push_back(std::move(submitted).Value());
      }
      for (const auto& task : tasks) {
        auto completed = task.Wait();
        if (!completed) {
          return Result<ExecutionCandidate>::Failure(completed.GetError());
        }
      }
    }
  } else {
    auto loader = input.algorithms->CreateDataLoader(
        *input.committed->config->data_loader);
    if (!loader) {
      auto context = data_loader_context;
      if (!contexts.empty()) context.agent = contexts.front().agent;
      return Result<ExecutionCandidate>::Failure(
          WithAlgorithmContext(loader.GetError(), context));
    }
    MapUpdateNode node(
        std::move(loader).Value(),
        [algorithms = input.algorithms,
         config = input.committed->config->dynamic_remover] {
          return algorithms->CreateDynamicRemover(*config);
        },
        input.output_directory.string(), input.save_voxel_size, true, {},
        data_loader_context, remover_context);
    for (auto& context : contexts) {
      auto updated = node.Process(context, *database);
      if (!updated) {
        return Result<ExecutionCandidate>::Failure(updated.GetError());
      }
      context.flow = updated.Value();
      if (updated.Value() == ControlFlow::kKill) {
        return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
            "MapUpdate node requested pipeline termination"));
      }
    }
  }
  if (input.cancellation && input.cancellation->IsCancellationRequested()) {
    return Result<ExecutionCandidate>::Failure(
        Error::Cancelled("before MapUpdate candidate publication"));
  }
  std::vector<AgentId> execution_agents;
  execution_agents.reserve(contexts.size());
  for (const auto& context : contexts) {
    execution_agents.push_back(context.agent.id);
  }
  // MapUpdate only prepares external files. Its local contexts/database are
  // transient algorithm workspaces; returning the committed payload verbatim
  // preserves descriptor/alignment stores and the exact resident owners.
  return Result<ExecutionCandidate>::Ok(
      {input.committed->revision, base_payload,
       std::move(execution_agents),
       target_agent ? ArtifactCompletionKind::kMapUpdateAgent
                    : ArtifactCompletionKind::kMapUpdateStage,
       target_agent});
}

}  // namespace open_lmm
