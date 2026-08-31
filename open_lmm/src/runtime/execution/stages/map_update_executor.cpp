#include "map_update_executor.hpp"

#include <algorithm>
#include <cstdint>
#include <exception>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <domain/data_loader/data_loader_base.hpp>
#include <domain/data_loader/resident_memory.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <foundation/diagnostics/profiling.hpp>
#include <foundation/logging/logging.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/model/execution_spec.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>
#include <runtime/execution/nodes/map_update_node.hpp>

namespace open_lmm {
namespace {

constexpr uint64_t kMapUpdateWorkingSetMultiplier = 3;

struct MapUpdateMemoryEstimate {
  uint64_t filtered_point_bytes = 0;
  uint64_t known_host_bytes = 0;
  uint64_t admitted_bytes = 1;
};

MapUpdateMemoryEstimate EstimateMapUpdateMemory(
    const AgentRawData& raw, const AgentOptimizedData& optimized) noexcept {
  MapUpdateMemoryEstimate estimate;
  for (const auto& scan : raw.filtered_scans) {
    if (!scan) continue;
    estimate.filtered_point_bytes = data_loader_memory::SaturatingAdd(
        estimate.filtered_point_bytes,
        data_loader_memory::SaturatingMultiply(
            scan->points.capacity(), sizeof(pcl::PointXYZI)));
  }
  const uint64_t scan_workspace_bytes =
      data_loader_memory::SaturatingMultiply(
          raw.filtered_scans.size(),
          sizeof(std::pair<std::size_t, ScanVec::value_type>) +
              sizeof(ScanVec::value_type));
  const uint64_t pose_workspace_bytes =
      data_loader_memory::SaturatingMultiply(
          optimized.optimized_poses.size(),
          sizeof(PoseVec::value_type) + sizeof(bool));
  estimate.known_host_bytes = data_loader_memory::SaturatingAdd(
      scan_workspace_bytes, pose_workspace_bytes);
  // Proxy the simultaneous buffered input, remover raw/static result, and
  // final downsample output. This is deliberately a soft admission estimate,
  // not a hard bound on plugin allocations or process RSS.
  estimate.admitted_bytes = std::max<uint64_t>(
      1, data_loader_memory::SaturatingAdd(
             data_loader_memory::SaturatingMultiply(
                 estimate.filtered_point_bytes,
                 kMapUpdateWorkingSetMultiplier),
             estimate.known_host_bytes));
  return estimate;
}

class HeavyPhaseLease {
 public:
  explicit HeavyPhaseLease(std::shared_ptr<ResourceGovernor> governor)
      : governor_(std::move(governor)) {}
  ~HeavyPhaseLease() {
    if (armed_) governor_->ReleaseHeavyMemoryPhase();
  }
  void Arm() noexcept { armed_ = true; }

 private:
  std::shared_ptr<ResourceGovernor> governor_;
  bool armed_ = false;
};

void LogMapUpdateAdmission(const AgentId& agent, bool parallel,
                           const MapUpdateMemoryEstimate& estimate,
                           const ResourceGovernor& governor) {
  // This is a soft allocation-shaped proxy, not a hard RSS bound. Raw decoded
  // scans, allocator/hash overhead, and plugin-internal state remain opaque.
  LogInfo("[resource] MapUpdate admission agent=" + agent.Value() +
          " mode=" + (parallel ? "parallel" : "sequential") +
          " filtered_point_bytes=" +
          std::to_string(estimate.filtered_point_bytes) +
          " known_host_bytes=" +
          std::to_string(estimate.known_host_bytes) +
          " estimated_heavy_bytes=" +
          std::to_string(estimate.admitted_bytes) +
          " current_bytes=" +
          std::to_string(governor.ReservedMemoryBytes()) +
          " limit_bytes=" +
          std::to_string(governor.Budget().soft_memory_bytes));
  OPEN_LMM_PLOT("map_update.estimated_heavy_bytes",
                estimate.admitted_bytes);
}

Result<std::shared_ptr<void>> AcquireHeavyPhase(
    const std::shared_ptr<ResourceGovernor>& governor,
    const std::shared_ptr<CancellationToken>& cancellation) {
  // Allocate the lifetime token before acquiring the one-slot gate so an
  // allocation failure can never leave the phase permanently armed.
  auto lease = std::make_shared<HeavyPhaseLease>(governor);
  auto acquired = governor->AcquireHeavyMemoryPhase(cancellation);
  if (!acquired) {
    return Result<std::shared_ptr<void>>::Failure(acquired.GetError());
  }
  lease->Arm();
  return Result<std::shared_ptr<void>>::Ok(std::move(lease));
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
    if (!base_payload->database->optimized_data.contains(*target_agent)) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "MapUpdate target has no ready optimized result"));
    }
  } else {
    std::erase_if(contexts, [&](const AgentPipelineCtx& context) {
      return !base_payload->database->optimized_data.contains(
          context.agent.id);
    });
    if (contexts.empty()) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "MapUpdate has no aligned agents to process"));
    }
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

  const auto run_agent = [&](std::size_t index,
                             bool hide_progress) -> Result<void> {
    const AgentId agent = contexts[index].agent.id;
    const auto raw = database->raw_data.find(agent);
    const auto optimized = database->optimized_data.find(agent);
    if (raw == database->raw_data.end() || !raw->second ||
        optimized == database->optimized_data.end() || !optimized->second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "MapUpdate agent has no committed raw/optimized payload"));
    }
    const auto estimate = EstimateMapUpdateMemory(*raw->second,
                                                   *optimized->second);
    LogMapUpdateAdmission(agent, input.parallel, estimate, *input.governor);
    auto admitted = input.governor->ReserveMemory(
        estimate.admitted_bytes, MemoryClass::kHeavyMap,
        input.cancellation);
    if (!admitted) return Result<void>::Failure(admitted.GetError());
    auto reservation = std::move(admitted).Value();

    DataLoaderConfig loader_config = *input.committed->config->data_loader;
    if (hide_progress) loader_config.show_progress = false;
    auto loader = input.algorithms->CreateDataLoader(loader_config);
    if (!loader) {
      auto context = data_loader_context;
      context.agent = contexts[index].agent;
      return Result<void>::Failure(
          WithAlgorithmContext(loader.GetError(), context));
    }
    try {
      SharedDatabase isolated;
      isolated.optimized_data.emplace(agent, optimized->second);
      AgentPipelineCtx isolated_context = contexts[index];
      MapUpdateNode node(
          std::move(loader).Value(),
          [algorithms = input.algorithms,
           config = input.committed->config->dynamic_remover] {
            return algorithms->CreateDynamicRemover(*config);
          },
          input.output_directory.string(), input.save_voxel_size, true,
          [governor = input.governor, cancellation = input.cancellation] {
            return AcquireHeavyPhase(governor, cancellation);
          },
          data_loader_context, remover_context);
      auto updated = node.Process(isolated_context, isolated);
      if (!updated) return Result<void>::Failure(updated.GetError());
      if (updated.Value() == ControlFlow::kKill) {
        return Result<void>::Failure(Error::InvalidArgument(
            "MapUpdate node requested pipeline termination"));
      }
    } catch (const std::exception& error) {
      auto context = remover_context;
      context.agent = contexts[index].agent;
      return Result<void>::Failure(WithAlgorithmContext(
          Error::InvalidArgument(std::string("MapUpdate operation exception: ") +
                                 error.what()),
          context));
    } catch (...) {
      auto context = remover_context;
      context.agent = contexts[index].agent;
      return Result<void>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("unknown MapUpdate operation exception"),
          context));
    }
    (void)reservation;
    return Result<void>::Ok();
  };

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
      tasks.reserve(end - begin);
      for (std::size_t index = begin; index < end; ++index) {
        auto submitted = input.governor->AgentExecutor().Submit(
            [&, index]() -> Result<void> {
              return run_agent(index, true);
            },
            input.cancellation);
        if (!submitted) {
          for (const auto& task : tasks) (void)task.Wait();
          return Result<ExecutionCandidate>::Failure(submitted.GetError());
        }
        tasks.push_back(std::move(submitted).Value());
      }
      std::optional<Error> first_error;
      for (const auto& task : tasks) {
        auto completed = task.Wait();
        if (!completed && !first_error) first_error = completed.GetError();
      }
      if (first_error) {
        return Result<ExecutionCandidate>::Failure(std::move(*first_error));
      }
    }
  } else {
    for (std::size_t index = 0; index < contexts.size(); ++index) {
      auto updated = run_agent(index, false);
      if (!updated) return Result<ExecutionCandidate>::Failure(updated.GetError());
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
