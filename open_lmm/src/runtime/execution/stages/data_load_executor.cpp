#include "data_load_executor.hpp"

#include <algorithm>
#include <limits>

#include <domain/data_loader/data_loader_base.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>
#include <runtime/execution/stage_runner.hpp>
#include <runtime/state/runtime_payload_builder.hpp>

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

uint64_t ResidentRawDataBytes(const AgentRawData& raw) {
  uint64_t bytes = sizeof(AgentRawData) + raw.agent_id.Value().capacity();
  const auto add = [&bytes](uint64_t value) {
    bytes = value > std::numeric_limits<uint64_t>::max() - bytes
                ? std::numeric_limits<uint64_t>::max()
                : bytes + value;
  };
  add(raw.odom_poses.capacity() * sizeof(Eigen::Isometry3d));
  add(raw.filtered_scans.capacity() * sizeof(ScanVec::value_type));
  for (const auto& scan : raw.filtered_scans) {
    if (scan) add(scan->points.capacity() * sizeof(pcl::PointXYZI));
  }
  return std::max<uint64_t>(bytes, 1);
}

}  // namespace

Result<ExecutionCandidate> DataLoadExecutor::Execute(
    DataLoadExecutionContext input) const {
  if (!input.committed || !input.committed->config ||
      !input.committed->config->data_loader || !input.committed->payload ||
      !input.governor || !input.algorithms) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "DataLoad execution requires committed config and a resource governor"));
  }
  if (input.contexts.empty() || !input.optimizer) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "DataLoad execution requires agent contexts and an optimizer owner"));
  }
  if (StageNodes(StageId::kDataLoad) !=
      std::vector<NodeId>{NodeId::kDataLoad}) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("unsupported DataLoad execution spec"));
  }
  if (!input.database) input.database = std::make_shared<SharedDatabase>();
  for (auto& context : input.contexts) {
    context.cancellation = input.cancellation;
  }

  std::vector<AgentRawDataHandle> loaded(input.contexts.size());
  std::vector<std::shared_ptr<MemoryReservation>> reserved(
      input.contexts.size());
  std::vector<uint64_t> estimates;
  estimates.reserve(input.contexts.size());
  uint64_t total_estimate = 0;
  for (const auto& context : input.contexts) {
    const uint64_t estimate = EstimateAgentMemory(context.data_dir);
    estimates.push_back(estimate);
    total_estimate =
        estimate > std::numeric_limits<uint64_t>::max() - total_estimate
            ? std::numeric_limits<uint64_t>::max()
            : total_estimate + estimate;
  }
  uint64_t replacement_credit = 0;
  for (const auto& [agent, reservation] :
       input.committed->payload->resident_memory_reservations) {
    (void)agent;
    if (!reservation) continue;
    replacement_credit =
        reservation->Bytes() >
                std::numeric_limits<uint64_t>::max() - replacement_credit
            ? std::numeric_limits<uint64_t>::max()
            : replacement_credit + reservation->Bytes();
  }
  const auto load_one = [&](std::size_t index,
                            bool hide_progress) -> Result<void> {
    const auto& item = input.contexts[index];
    auto admitted = replacement_credit == 0
                        ? input.governor->ReserveMemory(
                              estimates[index],
                              MemoryClass::kResidentPayload,
                              input.cancellation)
                        : input.governor->ReserveReplacementMemory(
                              estimates[index], replacement_credit,
                              total_estimate, input.cancellation);
    if (!admitted) return Result<void>::Failure(admitted.GetError());
    DataLoaderConfig config = *input.committed->config->data_loader;
    if (hide_progress) config.show_progress = false;
    if (!input.committed->config->documents) {
      return Result<void>::Failure(
          Error::InvalidArgument("DataLoad config document is unavailable"));
    }
    ExecutionContext command{input.cancellation, {},
                             input.committed->revision, input.progress};
    auto algorithm_context = MakeAlgorithmExecutionContext(
        *input.committed, command, item.agent,
        input.committed->config->documents->data_loader,
        "open_lmm.data_loader", "data_load", config.type);
    auto loader = input.algorithms->CreateDataLoader(config);
    if (!loader) {
      return Result<void>::Failure(
          WithAlgorithmContext(loader.GetError(), algorithm_context));
    }
    auto raw = loader.Value()->Process(
        algorithm_context, DataLoaderInput{item.data_dir});
    if (!raw) return Result<void>::Failure(raw.GetError());
    if (input.cancellation &&
        input.cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(
          Error::Cancelled("before DataLoad result publication"));
    }
    auto owned = std::make_shared<const AgentRawData>(std::move(raw).Value());
    auto memory = std::make_shared<MemoryReservation>(
        std::move(admitted).Value());
    auto resized = memory->Resize(ResidentRawDataBytes(*owned));
    if (!resized) return resized;
    loaded[index] = std::move(owned);
    reserved[index] = std::move(memory);
    if (input.on_agent_loaded) {
      try {
        input.on_agent_loaded(item.agent.id, loaded[index]);
      } catch (...) {
        // A best-effort read model must not affect candidate correctness.
      }
    }
    return Result<void>::Ok();
  };

  if (input.parallel) {
    const std::size_t concurrency = std::min(
        {std::max<std::size_t>(input.max_parallel_agents, 1),
         input.contexts.size(), input.governor->Budget().max_agent_tasks,
         input.governor->Budget().max_cpu_threads});
    if (concurrency == 0) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "DataLoad parallel execution has zero admitted concurrency"));
    }
    for (std::size_t begin = 0; begin < input.contexts.size();
         begin += concurrency) {
      const std::size_t end = std::min(input.contexts.size(),
                                       begin + concurrency);
      std::vector<BoundedTaskHandle> tasks;
      for (std::size_t index = begin; index < end; ++index) {
        auto submitted = input.governor->AgentExecutor().Submit(
            [&, index] { return load_one(index, true); }, input.cancellation);
        if (!submitted) {
          for (const auto& task : tasks) (void)task.Wait();
          return Result<ExecutionCandidate>::Failure(
              submitted.GetError());
        }
        tasks.push_back(std::move(submitted).Value());
      }
      for (const auto& task : tasks) {
        auto completed = task.Wait();
        if (!completed) {
          return Result<ExecutionCandidate>::Failure(
              completed.GetError());
        }
      }
    }
  } else {
    for (std::size_t index = 0; index < input.contexts.size(); ++index) {
      auto loaded_one = load_one(index, false);
      if (!loaded_one) {
        return Result<ExecutionCandidate>::Failure(
            loaded_one.GetError());
      }
    }
  }

  if (replacement_credit != 0) {
    auto replacement_valid =
        input.governor->ValidateReplacementMemory(replacement_credit);
    if (!replacement_valid) {
      return Result<ExecutionCandidate>::Failure(
          replacement_valid.GetError());
    }
  }

  std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations;
  for (std::size_t index = 0; index < input.contexts.size(); ++index) {
    if (!loaded[index] || !reserved[index]) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "DataLoad did not produce an owned result for every agent"));
    }
    auto& context = input.contexts[index];
    context.raw_data = loaded[index];
    if (!input.database->raw_data.emplace(context.agent.id,
                                          loaded[index]).second) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "DataLoad attempted to publish duplicate agent raw data"));
    }
    reservations.emplace(context.agent.id, reserved[index]);
  }
  RuntimePayloadBuilder builder(input.committed->payload);
  auto built = builder.SetContexts(std::move(input.contexts))
                   .SetDatabase(std::move(input.database))
                   .SetOptimizer(std::move(input.optimizer))
                   .ReplaceResidentReservations(std::move(reservations))
                   .Build();
  if (!built) {
    return Result<ExecutionCandidate>::Failure(built.GetError());
  }
  auto payload = std::move(built).Value();
  std::vector<AgentId> execution_agents;
  execution_agents.reserve(payload->contexts.size());
  for (const auto& context : payload->contexts) {
    execution_agents.push_back(context.agent.id);
  }
  return Result<ExecutionCandidate>::Ok(
      {input.committed->revision, std::move(payload),
       std::move(execution_agents), ArtifactCompletionKind::kDataLoadStage,
       std::nullopt});
}

Result<ExecutionCandidate> DataLoadExecutor::ExecuteAgent(
    DataLoadExecutionContext input, const AgentId& agent) const {
  if (!input.committed || !input.committed->config ||
      !input.committed->config->data_loader || !input.committed->payload ||
      !input.committed->payload->database || !input.governor ||
      !input.optimizer || !input.algorithms) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "agent DataLoad requires committed payload, config, optimizer, and governor"));
  }
  auto contexts = input.committed->payload->contexts;
  const auto selected = std::find_if(
      contexts.begin(), contexts.end(), [&agent](const auto& context) {
        return context.agent.id == agent;
      });
  if (selected == contexts.end()) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("unknown DataLoad agent"));
  }
  const std::size_t changed = static_cast<std::size_t>(
      std::distance(contexts.begin(), selected));
  const auto old_reservation =
      input.committed->payload->resident_memory_reservations.find(agent);
  const uint64_t replacement_credit =
      old_reservation ==
                  input.committed->payload->resident_memory_reservations.end() ||
              !old_reservation->second
          ? 0
          : old_reservation->second->Bytes();
  const uint64_t estimate = EstimateAgentMemory(selected->data_dir);
  auto admitted = replacement_credit == 0
                      ? input.governor->ReserveMemory(
                            estimate,
                            MemoryClass::kResidentPayload,
                            input.cancellation)
                      : input.governor->ReserveReplacementMemory(
                            estimate, replacement_credit, estimate,
                            input.cancellation);
  if (!admitted) {
    return Result<ExecutionCandidate>::Failure(admitted.GetError());
  }
  if (!input.committed->config->documents) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("DataLoad config document is unavailable"));
  }
  ExecutionContext command{input.cancellation, {}, input.committed->revision,
                           input.progress};
  auto algorithm_context = MakeAlgorithmExecutionContext(
      *input.committed, command, selected->agent,
      input.committed->config->documents->data_loader,
      "open_lmm.data_loader", "data_load",
      input.committed->config->data_loader->type);
  auto loader = input.algorithms->CreateDataLoader(
      *input.committed->config->data_loader);
  if (!loader) {
    return Result<ExecutionCandidate>::Failure(
        WithAlgorithmContext(loader.GetError(), algorithm_context));
  }
  auto raw = loader.Value()->Process(
      algorithm_context, DataLoaderInput{selected->data_dir});
  if (!raw) return Result<ExecutionCandidate>::Failure(raw.GetError());
  if (input.cancellation && input.cancellation->IsCancellationRequested()) {
    return Result<ExecutionCandidate>::Failure(
        Error::Cancelled("before agent DataLoad candidate publication"));
  }
  auto owned = std::make_shared<const AgentRawData>(std::move(raw).Value());
  auto reservation = std::make_shared<MemoryReservation>(
      std::move(admitted).Value());
  auto resized = reservation->Resize(ResidentRawDataBytes(*owned));
  if (!resized) {
    return Result<ExecutionCandidate>::Failure(resized.GetError());
  }
  if (replacement_credit != 0) {
    auto replacement_valid =
        input.governor->ValidateReplacementMemory(replacement_credit);
    if (!replacement_valid) {
      return Result<ExecutionCandidate>::Failure(
          replacement_valid.GetError());
    }
  }
  if (input.on_agent_loaded) {
    try {
      input.on_agent_loaded(agent, owned);
    } catch (...) {
      // A best-effort read model must not affect candidate correctness.
    }
  }
  selected->raw_data = owned;
  selected->cancellation = input.cancellation;
  selected->flow = ControlFlow::kContinue;
  selected->loop_output.reset();
  auto database = std::make_shared<SharedDatabase>(
      *input.committed->payload->database);
  database->raw_data[agent] = std::move(owned);
  for (std::size_t index = changed; index < contexts.size(); ++index) {
    contexts[index].loop_output.reset();
    database->optimized_data.erase(contexts[index].agent.id);
  }
  auto reservations =
      input.committed->payload->resident_memory_reservations;
  reservations[agent] = std::move(reservation);
  RuntimePayloadBuilder builder(input.committed->payload);
  auto built = builder.SetContexts(std::move(contexts))
                   .SetDatabase(std::move(database))
                   .SetOptimizer(std::move(input.optimizer))
                   .ReplaceResidentReservations(std::move(reservations))
                   .Build();
  if (!built) {
    return Result<ExecutionCandidate>::Failure(built.GetError());
  }
  return Result<ExecutionCandidate>::Ok(
      {input.committed->revision, std::move(built).Value(), {agent},
       ArtifactCompletionKind::kDataLoadAgent, agent});
}

}  // namespace open_lmm
