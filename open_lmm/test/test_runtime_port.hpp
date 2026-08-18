#pragma once

#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/stage_ports.hpp>

#include <functional>
#include <mutex>
#include <utility>

namespace open_lmm::test {

class RuntimePortFixture : public StageRuntimePort {
 public:
  explicit RuntimePortFixture(std::vector<AgentId> agents,
                              uint64_t config_revision = 1)
      : agents_(std::move(agents)) {
    artifacts_.Reset(agents_);
    snapshot_.revision = 1;
    snapshot_.config_revision = config_revision;
    snapshot_.ordered_agents = agents_;
    snapshot_.artifacts = artifacts_.Snapshot();
  }

  Result<ExecutionReceipt> Execute(
      const ExecutionCommand& command,
      const ExecutionContext& context) final {
    CommittedRuntimeSnapshot before;
    {
      std::lock_guard lock(mutex_);
      before = snapshot_;
    }
    if (context.base_revision != before.revision) {
      return Result<ExecutionReceipt>::Failure(
          Error::InvalidArgument("fixture base revision mismatch: context=" +
                                 std::to_string(context.base_revision) +
                                 " committed=" +
                                 std::to_string(before.revision)));
    }
    auto affected = AffectedAgents(command, before);
    if (!affected) {
      return Result<ExecutionReceipt>::Failure(affected.GetError());
    }
    auto result = ExecuteFixture(command, context);
    if (!result) {
      return Result<ExecutionReceipt>::Failure(result.GetError());
    }

    std::lock_guard lock(mutex_);
    artifacts_.Restore(snapshot_.artifacts);
    switch (command.kind) {
      case ExecutionCommandKind::kStage:
        artifacts_.BeginStage(*command.stage);
        artifacts_.CompleteStage(*command.stage);
        break;
      case ExecutionCommandKind::kNode:
        artifacts_.BeginNode(*command.node, affected.Value());
        artifacts_.CompleteNode(*command.node, affected.Value());
        break;
      case ExecutionCommandKind::kOptimizeThrough:
        artifacts_.CompleteOptimizeThrough(*command.agent, agents_);
        break;
      case ExecutionCommandKind::kReconfigure:
        artifacts_.ApplyConfig(*command.config_domain,
                               command.config_revision);
        snapshot_.config_revision = command.config_revision;
        break;
    }
    ++snapshot_.revision;
    snapshot_.artifacts = artifacts_.Snapshot();
    return Result<ExecutionReceipt>::Ok(AdjustReceipt(
        {before.revision, snapshot_.revision,
         std::move(affected).Value()}));
  }

  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context) override {
    std::lock_guard lock(mutex_);
    if (context.base_revision != snapshot_.revision ||
        expected.runtime_revision != snapshot_.revision ||
        expected.config_revision != snapshot_.config_revision) {
      return Result<ConfigApplyReceipt>::Failure(
          Error::InvalidArgument("fixture config revision mismatch"));
    }
    const uint64_t previous_config = snapshot_.config_revision;
    const uint64_t base_session = snapshot_.revision;
    artifacts_.Restore(snapshot_.artifacts);
    artifacts_.ApplyConfig(candidate.domain, previous_config + 1);
    ++snapshot_.config_revision;
    ++snapshot_.revision;
    snapshot_.artifacts = artifacts_.Snapshot();
    return Result<ConfigApplyReceipt>::Ok(
        {previous_config, snapshot_.config_revision, base_session,
         snapshot_.revision, agents_});
  }

  [[nodiscard]] CommittedRuntimeSnapshot Snapshot() const override {
    std::lock_guard lock(mutex_);
    return snapshot_;
  }

  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const AgentId& agent) const override {
    return CreateVisualization(agent);
  }

  Result<void> InitializeRuntimeRevisions(uint64_t runtime_revision,
                                          uint64_t config_revision) override {
    if (runtime_revision == 0 || config_revision == 0) {
      return Result<void>::Failure(
          Error::InvalidArgument("fixture runtime revisions must be non-zero"));
    }
    std::lock_guard lock(mutex_);
    snapshot_.revision = runtime_revision;
    snapshot_.config_revision = config_revision;
    return Result<void>::Ok();
  }

 protected:
  virtual Result<void> ExecuteFixture(const ExecutionCommand& command,
                                      const ExecutionContext& context) = 0;
  virtual Result<VisualizationSnapshot> CreateVisualization(
      const AgentId&) const {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("fixture visualization is unavailable"));
  }
  virtual ExecutionReceipt AdjustReceipt(ExecutionReceipt receipt) const {
    return receipt;
  }

 private:
  Result<std::vector<AgentId>> AffectedAgents(
      const ExecutionCommand& command,
      const CommittedRuntimeSnapshot& before) {
    artifacts_.Restore(before.artifacts);
    switch (command.kind) {
      case ExecutionCommandKind::kStage:
      case ExecutionCommandKind::kReconfigure:
        return Result<std::vector<AgentId>>::Ok(agents_);
      case ExecutionCommandKind::kNode:
        if (!command.node) {
          return Result<std::vector<AgentId>>::Failure(
              Error::InvalidArgument("fixture node is missing"));
        }
        return artifacts_.ExecutionAgents(*command.node, command.agent);
      case ExecutionCommandKind::kOptimizeThrough:
        if (!command.agent) {
          return Result<std::vector<AgentId>>::Failure(
              Error::InvalidArgument("fixture replay agent is missing"));
        }
        return OrderedAgentPrefix(agents_, *command.agent);
    }
    return Result<std::vector<AgentId>>::Failure(
        Error::InvalidArgument("unknown fixture command"));
  }

  std::vector<AgentId> agents_;
  mutable std::mutex mutex_;
  ArtifactRepository artifacts_;
  CommittedRuntimeSnapshot snapshot_;
};

}  // namespace open_lmm::test
