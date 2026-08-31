#pragma once

#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <runtime/execution/stage_runner.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace open_lmm {

enum class ExecutionCommandKind : uint8_t {
  kStage,
  kNode,
  kOptimizeThrough,
  kReconfigure,
};

struct ExecutionCommand {
  ExecutionCommandKind kind = ExecutionCommandKind::kStage;
  std::optional<StageId> stage;
  std::optional<NodeId> node;
  std::optional<AgentId> agent;
  std::optional<ConfigDomain> config_domain;
  uint64_t config_revision = 0;

  [[nodiscard]] static ExecutionCommand Stage(StageId value) {
    ExecutionCommand command;
    command.kind = ExecutionCommandKind::kStage;
    command.stage = value;
    return command;
  }

  [[nodiscard]] static ExecutionCommand Node(
      NodeId value, std::optional<AgentId> target = std::nullopt) {
    ExecutionCommand command;
    command.kind = ExecutionCommandKind::kNode;
    command.node = value;
    command.agent = std::move(target);
    return command;
  }

  [[nodiscard]] static ExecutionCommand OptimizeThrough(AgentId target) {
    ExecutionCommand command;
    command.kind = ExecutionCommandKind::kOptimizeThrough;
    command.agent = std::move(target);
    return command;
  }

  [[nodiscard]] static ExecutionCommand Reconfigure(
      ConfigDomain domain, uint64_t revision) {
    ExecutionCommand command;
    command.kind = ExecutionCommandKind::kReconfigure;
    command.config_domain = domain;
    command.config_revision = revision;
    return command;
  }
};

struct ExecutionContext {
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  uint64_t base_revision = 0;
  AlgorithmProgressCallback progress;
};

struct ExecutionReceipt {
  uint64_t base_revision = 0;
  uint64_t committed_revision = 0;
  std::vector<AgentId> affected_agents;
};

class StageCommandPort {
 public:
  virtual ~StageCommandPort() = default;
  [[nodiscard]] virtual CancellationCapability CancellationMetadata() const {
    return {};
  }
  virtual Result<ExecutionReceipt> Execute(
      const ExecutionCommand& command,
      const ExecutionContext& context) = 0;
  virtual Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate&, const ExpectedRevision&,
      const ExecutionContext&) {
    return Result<ConfigApplyReceipt>::Failure(
        Error::InvalidArgument("config transactions are not supported"));
  }
};

class RuntimeQueryPort {
 public:
  virtual ~RuntimeQueryPort() = default;
  [[nodiscard]] virtual CommittedRuntimeSnapshot Snapshot() const = 0;
  [[nodiscard]] virtual Result<VisualizationSnapshot> Visualization(
      const VisualizationQuery& query) const = 0;
};

class StageRuntimePort : public StageCommandPort, public RuntimeQueryPort {
 public:
  ~StageRuntimePort() override = default;
  // Bootstrap creates revision 1 by default. A private root replacement may
  // rebase its candidate before publication so the public runtime revision is
  // monotonic across epochs. Test ports that own no mutable state can retain
  // this no-op implementation.
  virtual Result<void> InitializeRuntimeRevisions(uint64_t, uint64_t) {
    return Result<void>::Ok();
  }
};

}  // namespace open_lmm
