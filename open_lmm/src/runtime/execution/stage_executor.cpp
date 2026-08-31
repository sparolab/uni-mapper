#include "stage_executor.hpp"

#include <algorithm>
#include <utility>

#include <runtime/state/artifact_repository.hpp>
#include <runtime/composition/runtime_bootstrapper.hpp>
#include <foundation/logging/logging.hpp>

namespace open_lmm {
namespace {

class ExecutionLease {
 public:
  explicit ExecutionLease(std::atomic_flag& active)
      : active_(active),
        acquired_(!active_.test_and_set(std::memory_order_acquire)) {}
  ~ExecutionLease() {
    if (acquired_) active_.clear(std::memory_order_release);
  }
  [[nodiscard]] explicit operator bool() const { return acquired_; }

 private:
  std::atomic_flag& active_;
  bool acquired_;
};

Result<void> ValidateNodeTarget(const RuntimeState& state, NodeId node,
                                const std::optional<AgentId>& agent) {
  const auto& spec = ExecutionSpecFor(node);
  if (spec.scope == ExecutionScope::kPerAgent && !agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("per-agent node requires an agent target"));
  }
  if (spec.scope == ExecutionScope::kRuntime && agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime node must not have an agent target"));
  }
  if (agent &&
      std::find(state.ordered_agents.begin(), state.ordered_agents.end(),
                *agent) == state.ordered_agents.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }
  return Result<void>::Ok();
}

VisualizationPhase InferVisualizationPhase(const RuntimeState& state) {
  if (!state.payload || !state.payload->database) {
    return VisualizationPhase::kDataLoad;
  }
  if (!state.payload->database->optimized_data.empty()) {
    return VisualizationPhase::kOptimization;
  }
  if (std::any_of(state.payload->contexts.begin(), state.payload->contexts.end(),
                  [](const AgentPipelineCtx& context) {
                    return static_cast<bool>(context.loop_output);
                  })) {
    return VisualizationPhase::kLoopDetection;
  }
  return VisualizationPhase::kDataLoad;
}

std::optional<VisualizationSource> BuildVisualizationSource(
    const std::shared_ptr<const RuntimeState>& state) {
  if (!state || !state->config || !state->payload ||
      !state->payload->database) {
    return std::nullopt;
  }
  VisualizationSource source;
  source.revision = state->revision;
  source.output_directory = state->config->root.output_directory;
  const auto& database = *state->payload->database;
  source.agents.reserve(database.raw_data.size());
  for (const auto& [agent, raw] : database.raw_data) {
    const auto optimized = database.optimized_data.find(agent);
    const auto context = std::find_if(
        state->payload->contexts.begin(), state->payload->contexts.end(),
        [&agent](const AgentPipelineCtx& item) {
          return item.agent.id == agent;
        });
    source.agents.push_back(
        {agent, raw,
         optimized == database.optimized_data.end() ? nullptr
                                                    : optimized->second,
         context == state->payload->contexts.end() ? nullptr
                                                   : context->loop_output});
  }
  return source;
}

}  // namespace

StageExecutor::StageExecutor(
    BootstrapConfigSnapshot bootstrap_config,
    std::optional<std::filesystem::path> output_directory,
    std::shared_ptr<ResourceGovernor> resource_governor)
    : resource_governor_(std::move(resource_governor)) {
  InitializeLogging();
  RuntimeBootstrapper bootstrap;
  auto initialized = bootstrap.Bootstrap(
      {std::move(bootstrap_config), std::move(output_directory), {}});
  if (!initialized) {
    initialization_error_ = initialized.GetError();
    return;
  }
  auto result = std::move(initialized).Value();
  if (!resource_governor_) {
    resource_governor_ =
        std::make_shared<ResourceGovernor>(result.suggested_resource_budget);
  }
  runtime_state_store_.Initialize(std::move(result.initial_state));
  coordinator_ = std::make_unique<StageCoordinator>(
      runtime_state_store_, output_repository_, resource_governor_,
      std::move(result.algorithms),
      [this](uint64_t base_revision, const AgentId& agent,
             const AgentRawDataHandle& raw) {
        visualization_projector_.PublishDataLoadCandidate(base_revision,
                                                          agent, raw);
      });
}

StageExecutor::~StageExecutor() = default;

std::shared_ptr<const RuntimeState> StageExecutor::CommittedState() const {
  return runtime_state_store_.Snapshot();
}

Result<void> StageExecutor::EnsureReady() {
  if (initialization_error_) {
    Error error = *initialization_error_;
    error.MarkFatalRuntime();
    return Result<void>::Failure(std::move(error));
  }
  const auto state = CommittedState();
  if (!state || !state->config || !state->payload ||
      !state->payload->database || !state->payload->optimizer ||
      state->ordered_agents.empty()) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime state is unavailable"));
  }
  if (state->ordered_agents.size() > AgentSymbolCatalog::kMaximumAgents) {
    return Result<void>::Failure(Error::InvalidArgument(
        "At most 255 agents are supported by the GTSAM symbol catalog"));
  }
  if (!coordinator_) {
    return Result<void>::Failure(
        Error::InvalidArgument("stage coordinator is unavailable"));
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::ValidateReady() {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  return EnsureReady();
}

CancellationCapability StageExecutor::CancellationMetadata() const {
  return {
      .cooperative = false,
      .mode = CancellationMode::kHostSafePoints,
      .non_interruptible_operations =
          {"descriptor plugin call", "map alignment registration",
           "dynamic remover plugin call", "GTSAM optimize"},
      .requires_process_isolation = true,
  };
}

CommittedRuntimeSnapshot StageExecutor::Snapshot() const {
  const auto state = CommittedState();
  if (!state) return {};
  CommittedRuntimeSnapshot snapshot{state->revision,
                                    state->config ? state->config->revision : 0,
                                    state->ordered_agents, state->artifacts};
  if (state->payload && state->payload->database) {
    const auto& descriptors = state->payload->database->descriptor_store;
    snapshot.descriptor_count =
        descriptors.total_db ? descriptors.total_db->getSize() : 0;
    for (const auto& [agent, database] : descriptors.per_agent_db) {
      snapshot.per_agent_descriptor_count[agent] =
          database ? database->getSize() : 0;
    }
  }
  return snapshot;
}

void StageExecutor::PublishEmptyVisualization() {
  const auto state = CommittedState();
  visualization_projector_.Clear(state ? state->revision : 0);
}

void StageExecutor::PublishVisualization(VisualizationPhase phase,
                                         bool include_maps) {
  const auto state = CommittedState();
  auto source = BuildVisualizationSource(state);
  if (!source) {
    visualization_projector_.Clear(state ? state->revision : 0);
    return;
  }
  visualization_projector_.Publish(std::move(*source), phase, include_maps);
}

Result<VisualizationSnapshot> StageExecutor::Visualization(
    const VisualizationQuery& query) const {
  return visualization_projector_.Project(query);
}

Result<ExecutionReceipt> StageExecutor::Execute(
    const ExecutionCommand& command, const ExecutionContext& context) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = EnsureReady();
  if (!ready) return Result<ExecutionReceipt>::Failure(ready.GetError());
  if (!context.cancellation) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("execution cancellation token is required"));
  }
  const auto base = CommittedState();
  if (context.base_revision != base->revision) {
    return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
        "execution base revision does not match committed runtime"));
  }

  Result<void> result = Result<void>::Failure(
      Error::InvalidArgument("unknown execution command"));
  switch (command.kind) {
    case ExecutionCommandKind::kStage:
      if (!command.stage) {
        return Result<ExecutionReceipt>::Failure(
            Error::InvalidArgument("stage command requires a stage"));
      }
      result = coordinator_->ExecuteStage(base, *command.stage, context);
      if (!result && *command.stage == StageId::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base->revision);
      }
      if (result) {
        const VisualizationPhase phase =
            *command.stage == StageId::kDataLoad
                ? VisualizationPhase::kDataLoad
            : *command.stage == StageId::kAlignment
                ? VisualizationPhase::kOptimization
            : *command.stage == StageId::kMapUpdate
                ? VisualizationPhase::kMapUpdate
                : VisualizationPhase::kSave;
        PublishVisualization(phase, *command.stage == StageId::kMapUpdate ||
                                        *command.stage == StageId::kSave);
      }
      break;
    case ExecutionCommandKind::kNode: {
      if (!command.node) {
        return Result<ExecutionReceipt>::Failure(
            Error::InvalidArgument("node command requires a node"));
      }
      auto target = ValidateNodeTarget(*base, *command.node, command.agent);
      if (!target) {
        return Result<ExecutionReceipt>::Failure(target.GetError());
      }
      result = coordinator_->ExecuteNode(base, *command.node, command.agent,
                                         context);
      if (!result && *command.node == NodeId::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base->revision);
      }
      if (result) {
        const VisualizationPhase phase =
            *command.node == NodeId::kDataLoad
                ? VisualizationPhase::kDataLoad
            : *command.node == NodeId::kLoopDetect
                ? VisualizationPhase::kLoopDetection
            : *command.node == NodeId::kOptimize
                ? VisualizationPhase::kOptimization
            : *command.node == NodeId::kMapUpdate
                ? VisualizationPhase::kMapUpdate
                : VisualizationPhase::kSave;
        PublishVisualization(phase, *command.node == NodeId::kMapUpdate ||
                                        *command.node == NodeId::kPoseSave ||
                                        *command.node == NodeId::kFallbackMapSave);
      }
      break;
    }
    case ExecutionCommandKind::kOptimizeThrough:
      if (!command.agent) {
        return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
            "optimizer replay command requires a target agent"));
      }
      result = coordinator_->ExecuteOptimizeThrough(base, *command.agent,
                                                    context);
      if (result) {
        PublishVisualization(VisualizationPhase::kOptimization, false);
      }
      break;
    case ExecutionCommandKind::kReconfigure:
      if (!command.config_domain || command.config_revision == 0) {
        return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
            "reconfigure command requires a domain and revision"));
      }
      result = coordinator_->ExecuteReconfigure(
          base, *command.config_domain, command.config_revision, context);
      if (result) {
        PublishVisualization(InferVisualizationPhase(*CommittedState()), false);
      }
      break;
  }
  if (!result) return Result<ExecutionReceipt>::Failure(result.GetError());

  const auto after = Snapshot();
  if (after.revision <= base->revision) {
    return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
        "successful command did not advance the committed revision"));
  }
  const CommittedRuntimeSnapshot before{
      base->revision, base->config->revision, base->ordered_agents,
      base->artifacts};
  return Result<ExecutionReceipt>::Ok(
      {base->revision, after.revision,
       ArtifactRevisionAffectedAgents(before, after)});
}

Result<ConfigApplyReceipt> StageExecutor::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected,
    const ExecutionContext& context) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<ConfigApplyReceipt>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = EnsureReady();
  if (!ready) {
    return Result<ConfigApplyReceipt>::Failure(ready.GetError());
  }
  if (!context.cancellation) {
    return Result<ConfigApplyReceipt>::Failure(
        Error::InvalidArgument("execution cancellation token is required"));
  }
  const auto base = CommittedState();
  if (context.base_revision != base->revision ||
      expected.runtime_revision != base->revision ||
      !base->config || expected.config_revision != base->config->revision) {
    return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
        "config transaction revision does not match committed runtime"));
  }
  auto applied = coordinator_->ApplyConfig(base, candidate, expected, context);
  if (!applied) return applied;
  PublishVisualization(InferVisualizationPhase(*CommittedState()), false);
  return applied;
}

Result<void> StageExecutor::InitializeRuntimeRevisions(
    uint64_t runtime_revision, uint64_t config_revision) {
  const auto committed = CommittedState();
  if (!committed || !committed->config) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime state is unavailable for rebase"));
  }
  if (runtime_revision == 0 || config_revision == 0) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime revisions must be non-zero"));
  }
  auto rebased = std::make_shared<RuntimeState>(*committed);
  auto config = std::make_shared<RuntimeConfig>(*committed->config);
  rebased->revision = runtime_revision;
  config->revision = config_revision;
  rebased->config = std::move(config);
  runtime_state_store_.Initialize(std::move(rebased));
  PublishEmptyVisualization();
  return Result<void>::Ok();
}

}  // namespace open_lmm
