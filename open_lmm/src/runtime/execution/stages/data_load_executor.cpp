#include "data_load_executor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

#include <domain/data_loader/data_loader_base.hpp>
#include <domain/support/pointcloud_utils.hpp>
#include <domain/data_loader/resident_memory.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/model/execution_spec.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>
#include <runtime/state/runtime_payload_builder.hpp>

namespace open_lmm {
namespace {

class DataLoadPreviewBuilder {
 public:
  explicit DataLoadPreviewBuilder(float voxel_size_m)
      : voxel_millimeters_(static_cast<uint32_t>(std::clamp(
            std::llround(static_cast<double>(voxel_size_m) * 1000.0), 1LL,
            static_cast<long long>(std::numeric_limits<uint32_t>::max())))),
        voxels_(voxel_size_m, 0.0F, 0.0F, false) {}

  void Add(const Eigen::Isometry3d& world_T_scan,
           const pcl::PointCloud<pcl::PointXYZI>& scan) noexcept {
    if (failed_) return;
    try {
      for (const auto& point : scan) {
        const Eigen::Vector3d transformed =
            world_T_scan * Eigen::Vector3d(point.x, point.y, point.z);
        if (!transformed.allFinite()) continue;
        const Eigen::Vector3f transformed_float = transformed.cast<float>();
        if (!transformed_float.allFinite()) continue;
        pcl::PointXYZI output;
        output.x = transformed_float.x();
        output.y = transformed_float.y();
        output.z = transformed_float.z();
        output.intensity = point.intensity;
        voxels_.Add(output);
      }
    } catch (...) {
      failed_ = true;
    }
  }

  VisualizationPointPreviewHandle Finish() noexcept {
    if (failed_) return {};
    try {
      auto preview = std::make_shared<VisualizationPointPreview>();
      preview->voxel_millimeters = voxel_millimeters_;
      preview->source_point_count = voxels_.SourcePointCount();
      preview->points.reserve(voxels_.Size());
      std::move(voxels_).ConsumeAverages(
          [&preview](float x, float y, float z, float intensity) {
            preview->points.push_back({x, y, z, intensity});
            const Eigen::Vector3f position(x, y, z);
            if (!preview->has_bounds) {
              preview->min_bound = preview->max_bound = position;
              preview->has_bounds = true;
            } else {
              preview->min_bound = preview->min_bound.cwiseMin(position);
              preview->max_bound = preview->max_bound.cwiseMax(position);
            }
          });
      return preview;
    } catch (...) {
      failed_ = true;
      return {};
    }
  }

 private:
  uint32_t voxel_millimeters_ = 0;
  IncrementalVoxelAccumulator voxels_;
  bool failed_ = false;
};

std::unique_ptr<DataLoadPreviewBuilder> MakeDataLoadPreviewBuilder(
    bool enabled, float voxel_size_m) noexcept {
  if (!enabled) return {};
  try {
    return std::make_unique<DataLoadPreviewBuilder>(voxel_size_m);
  } catch (...) {
    return {};
  }
}

Error DataLoadMemoryError(Error error,
                          const AlgorithmExecutionContext& context) {
  error.context.stage = "data_load";
  return WithAlgorithmContext(std::move(error), context);
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
    auto admitted = replacement_credit == 0
                        ? input.governor->ReserveMemory(
                              1, MemoryClass::kResidentPayload,
                              input.cancellation)
                        : input.governor->ReserveReplacementMemory(
                              1, replacement_credit, 0,
                              input.cancellation);
    if (!admitted) {
      return Result<void>::Failure(
          DataLoadMemoryError(admitted.GetError(), algorithm_context));
    }
    auto memory = std::make_shared<MemoryReservation>(
        std::move(admitted).Value());
    auto loader = input.algorithms->CreateDataLoader(config);
    if (!loader) {
      return Result<void>::Failure(
          WithAlgorithmContext(loader.GetError(), algorithm_context));
    }
    auto preview_builder = MakeDataLoadPreviewBuilder(
        static_cast<bool>(input.on_agent_loaded), input.preview_voxel_size_m);
    DataLoaderInput loader_input{
        .data_directory = item.data_dir,
        .admit_resident_bytes = [memory, algorithm_context](uint64_t bytes) {
          auto resized = memory->Resize(std::max<uint64_t>(bytes, 1));
          if (!resized) {
            return Result<void>::Failure(DataLoadMemoryError(
                resized.GetError(), algorithm_context));
          }
          return Result<void>::Ok();
        },
        .observe_filtered_scan =
            preview_builder
                ? DataLoaderInput::FilteredScanObserver(
                      [builder = preview_builder.get()](
                          std::size_t, const Eigen::Isometry3d& pose,
                          const pcl::PointCloud<pcl::PointXYZI>& scan) {
                        builder->Add(pose, scan);
                      })
                : DataLoaderInput::FilteredScanObserver{}};
    auto raw = loader.Value()->Process(algorithm_context, loader_input);
    if (!raw) return Result<void>::Failure(raw.GetError());
    if (input.cancellation &&
        input.cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(
          Error::Cancelled("before DataLoad result publication"));
    }
    ReportAlgorithmProgress(algorithm_context,
                            AlgorithmProgressPhase::kBuildPreview, 0, 1);
    auto owned = std::make_shared<const AgentRawData>(std::move(raw).Value());
    auto resized = memory->Resize(
        data_loader_memory::ResidentRawDataBytes(*owned));
    if (!resized) {
      return Result<void>::Failure(
          DataLoadMemoryError(resized.GetError(), algorithm_context));
    }
    loaded[index] = std::move(owned);
    reserved[index] = std::move(memory);
    auto preview = preview_builder ? preview_builder->Finish()
                                   : VisualizationPointPreviewHandle{};
    if (input.on_agent_loaded) {
      try {
        input.on_agent_loaded(item.agent.id, loaded[index], preview);
      } catch (...) {
        // A best-effort read model must not affect candidate correctness.
      }
    }
    ReportAlgorithmProgress(algorithm_context,
                            AlgorithmProgressPhase::kBuildPreview, 1, 1);
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
      std::optional<Error> batch_error;
      for (const auto& task : tasks) {
        auto completed = task.Wait();
        if (!completed && !batch_error) batch_error = completed.GetError();
      }
      if (batch_error) {
        return Result<ExecutionCandidate>::Failure(*batch_error);
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
  auto admitted = replacement_credit == 0
                      ? input.governor->ReserveMemory(
                            1, MemoryClass::kResidentPayload,
                            input.cancellation)
                      : input.governor->ReserveReplacementMemory(
                            1, replacement_credit, 0,
                            input.cancellation);
  if (!admitted) {
    return Result<ExecutionCandidate>::Failure(
        DataLoadMemoryError(admitted.GetError(), algorithm_context));
  }
  auto reservation = std::make_shared<MemoryReservation>(
      std::move(admitted).Value());
  auto loader = input.algorithms->CreateDataLoader(
      *input.committed->config->data_loader);
  if (!loader) {
    return Result<ExecutionCandidate>::Failure(
        WithAlgorithmContext(loader.GetError(), algorithm_context));
  }
  auto preview_builder = MakeDataLoadPreviewBuilder(
      static_cast<bool>(input.on_agent_loaded), input.preview_voxel_size_m);
  DataLoaderInput loader_input{
      .data_directory = selected->data_dir,
      .admit_resident_bytes = [reservation, algorithm_context](uint64_t bytes) {
        auto resized = reservation->Resize(std::max<uint64_t>(bytes, 1));
        if (!resized) {
          return Result<void>::Failure(DataLoadMemoryError(
              resized.GetError(), algorithm_context));
        }
        return Result<void>::Ok();
      },
      .observe_filtered_scan =
          preview_builder
              ? DataLoaderInput::FilteredScanObserver(
                    [builder = preview_builder.get()](
                        std::size_t, const Eigen::Isometry3d& pose,
                        const pcl::PointCloud<pcl::PointXYZI>& scan) {
                      builder->Add(pose, scan);
                    })
              : DataLoaderInput::FilteredScanObserver{}};
  auto raw = loader.Value()->Process(algorithm_context, loader_input);
  if (!raw) return Result<ExecutionCandidate>::Failure(raw.GetError());
  if (input.cancellation && input.cancellation->IsCancellationRequested()) {
    return Result<ExecutionCandidate>::Failure(
        Error::Cancelled("before agent DataLoad candidate publication"));
  }
  ReportAlgorithmProgress(algorithm_context,
                          AlgorithmProgressPhase::kBuildPreview, 0, 1);
  auto owned = std::make_shared<const AgentRawData>(std::move(raw).Value());
  auto resized = reservation->Resize(
      data_loader_memory::ResidentRawDataBytes(*owned));
  if (!resized) {
    return Result<ExecutionCandidate>::Failure(
        DataLoadMemoryError(resized.GetError(), algorithm_context));
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
      input.on_agent_loaded(
          agent, owned,
          preview_builder ? preview_builder->Finish()
                          : VisualizationPointPreviewHandle{});
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
  ReportAlgorithmProgress(algorithm_context,
                          AlgorithmProgressPhase::kBuildPreview, 1, 1);
  return Result<ExecutionCandidate>::Ok(
      {input.committed->revision, std::move(built).Value(), {agent},
       ArtifactCompletionKind::kDataLoadAgent, agent});
}

}  // namespace open_lmm
