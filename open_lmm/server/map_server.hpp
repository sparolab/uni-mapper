#pragma once

#include <memory>

#include <open_lmm/server/stage_runner.hpp>

namespace open_lmm {

class StageExecutor;

// Public runtime façade. Session ownership, transaction commits, algorithm
// assembly, output persistence and visualization snapshots live in the
// internal StageExecutor component.
class MapServer final : public StageRunner {
 public:
  MapServer();
  ~MapServer() override;

  MapServer(const MapServer&) = delete;
  MapServer& operator=(const MapServer&) = delete;

  Result<void> process();
  void SetCancellationToken(std::shared_ptr<CancellationToken> token) override;
  [[nodiscard]] CancellationCapability CancellationMetadata() const override;
  void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker> broker) override;
  Result<void> RunStage(StageId stage) override;
  Result<void> RunNode(NodeId node, std::optional<AgentId> agent) override;
  Result<void> RunOptimizeThrough(const AgentId& target_agent) override;
  Result<void> Reconfigure(ConfigDomain domain, uint64_t revision) override;
  [[nodiscard]] std::vector<AgentId> AgentIds() const override;
  [[nodiscard]] std::optional<CommittedSessionSnapshot>
  SessionSnapshot() const override;
  [[nodiscard]] Result<VisualizationSnapshot> CreateVisualizationSnapshot(
      const AgentId& agent) const override;
  Result<void> ValidateReady();

 private:
  std::unique_ptr<StageExecutor> executor_;
};

}  // namespace open_lmm
