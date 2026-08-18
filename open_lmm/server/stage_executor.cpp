#include "stage_executor.hpp"

#include <algorithm>
#include <utility>

#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/bootstrap/session_bootstrapper.hpp>
#include <open_lmm/utils/logging.hpp>

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

Result<void> ValidateNodeTarget(const SessionState& state, NodeId node,
                                const std::optional<AgentId>& agent) {
  const auto& spec = ExecutionSpecFor(node);
  if (spec.scope == ExecutionScope::kPerAgent && !agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("per-agent node requires an agent target"));
  }
  if (spec.scope == ExecutionScope::kSession && agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("session node must not have an agent target"));
  }
  if (agent &&
      std::find(state.ordered_agents.begin(), state.ordered_agents.end(),
                *agent) == state.ordered_agents.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }
  return Result<void>::Ok();
}

}  // namespace

StageExecutor::StageExecutor(
    BootstrapConfigSnapshot bootstrap_config,
    std::optional<std::filesystem::path> output_directory,
    std::shared_ptr<ResourceGovernor> resource_governor)
    : resource_governor_(std::move(resource_governor)) {
  InitializeLogging();
  SessionBootstrapper bootstrap;
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
  session_manager_.Initialize(std::move(result.initial_state));
  coordinator_ = std::make_unique<StageCoordinator>(
      session_manager_, output_repository_, resource_governor_);
}

StageExecutor::~StageExecutor() = default;

std::shared_ptr<const SessionState> StageExecutor::CommittedState() const {
  return session_manager_.Snapshot();
}

Result<void> StageExecutor::EnsureReady() {
  if (initialization_error_) {
    Error error = *initialization_error_;
    error.MarkFatalSession();
    return Result<void>::Failure(std::move(error));
  }
  const auto state = CommittedState();
  if (!state || !state->config || !state->payload ||
      !state->payload->database || !state->payload->optimizer ||
      state->ordered_agents.empty()) {
    return Result<void>::Failure(
        Error::InvalidArgument("session state is unavailable"));
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

CommittedSessionSnapshot StageExecutor::Snapshot() const {
  const auto state = CommittedState();
  if (!state) return {};
  CommittedSessionSnapshot snapshot{state->revision,
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

void StageExecutor::PublishVisualization(bool include_maps) {
  visualization_projector_.Publish(CommittedState(), include_maps);
}

Result<VisualizationSnapshot> StageExecutor::Visualization(
    const AgentId& agent) const {
  return visualization_projector_.Project(agent);
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
        "execution base revision does not match committed session"));
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
      if (result && *command.stage == StageId::kDataLoad) {
        PublishEmptyVisualization();
      } else if (result) {
        PublishVisualization(*command.stage == StageId::kMapUpdate ||
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
      if (result && (*command.node == NodeId::kDataLoad ||
                     *command.node == NodeId::kLoopDetect)) {
        PublishEmptyVisualization();
      } else if (result) {
        PublishVisualization(*command.node == NodeId::kMapUpdate ||
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
      if (result) PublishVisualization(false);
      break;
    case ExecutionCommandKind::kReconfigure:
      if (!command.config_domain || command.config_revision == 0) {
        return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
            "reconfigure command requires a domain and revision"));
      }
      result = coordinator_->ExecuteReconfigure(
          base, *command.config_domain, command.config_revision, context);
      if (result && (*command.config_domain == ConfigDomain::kLoopDetector ||
                     *command.config_domain == ConfigDomain::kOptimizer)) {
        PublishEmptyVisualization();
      } else if (result) {
        PublishVisualization(false);
      }
      break;
  }
  if (!result) return Result<ExecutionReceipt>::Failure(result.GetError());

  const auto after = Snapshot();
  if (after.revision <= base->revision) {
    return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
        "successful command did not advance the committed revision"));
  }
  const CommittedSessionSnapshot before{
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
      expected.session_revision != base->revision ||
      !base->config || expected.config_revision != base->config->revision) {
    return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
        "config transaction revision does not match committed session"));
  }
  auto applied = coordinator_->ApplyConfig(base, candidate, expected, context);
  if (!applied) return applied;
  if (candidate.domain == ConfigDomain::kLoopDetector ||
      candidate.domain == ConfigDomain::kOptimizer) {
    PublishEmptyVisualization();
  } else {
    PublishVisualization(false);
  }
  return applied;
}

}  // namespace open_lmm
