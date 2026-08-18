#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/server/execution/stage_coordinator.hpp>
#include <open_lmm/server/query/visualization_projector.hpp>
#include <open_lmm/server/resource_governor.hpp>
#include <open_lmm/server/session_manager.hpp>
#include <open_lmm/server/stage_ports.hpp>

namespace open_lmm {

// Thin internal façade: bootstrap once, serialize commands, delegate stage
// candidates/transactions, and publish committed query projections.
class StageExecutor {
 public:
  StageExecutor(
      std::filesystem::path config_directory,
      std::optional<std::filesystem::path> output_directory = std::nullopt,
      std::shared_ptr<ResourceGovernor> resource_governor = {});
  ~StageExecutor();

  [[nodiscard]] CancellationCapability CancellationMetadata() const;
  Result<ExecutionReceipt> Execute(const ExecutionCommand& command,
                                   const ExecutionContext& context);
  [[nodiscard]] CommittedSessionSnapshot Snapshot() const;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const AgentId& agent) const;
  Result<void> ValidateReady();

 private:
  Result<void> EnsureReady();
  void PublishVisualization(bool include_maps);
  void PublishEmptyVisualization();
  [[nodiscard]] std::shared_ptr<const SessionState> CommittedState() const;

  std::shared_ptr<ResourceGovernor> resource_governor_;
  std::optional<Error> initialization_error_;
  SessionManager session_manager_;
  OutputRepository output_repository_;
  std::unique_ptr<StageCoordinator> coordinator_;
  VisualizationProjector visualization_projector_;
  std::atomic_flag execution_active_ = ATOMIC_FLAG_INIT;
};

}  // namespace open_lmm
