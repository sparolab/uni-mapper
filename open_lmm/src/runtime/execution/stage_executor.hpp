#pragma once

#include <atomic>
#include <filesystem>
#include <memory>
#include <optional>

#include <runtime/execution/stages/stage_coordinator.hpp>
#include <config/bootstrap/bootstrap_config.hpp>
#include <visualization/projection/visualization_projector.hpp>
#include <runtime/resources/resource_governor.hpp>
#include <runtime/state/runtime_state_store.hpp>
#include <runtime/execution/stage_ports.hpp>
#include <storage/transactions/output_repository.hpp>

namespace open_lmm {

// Thin internal façade: bootstrap once, serialize commands, delegate stage
// candidates/transactions, and publish committed query projections.
class StageExecutor {
 public:
  StageExecutor(
      BootstrapConfigSnapshot bootstrap_config,
      std::optional<std::filesystem::path> output_directory = std::nullopt,
      std::shared_ptr<ResourceGovernor> resource_governor = {});
  ~StageExecutor();

  [[nodiscard]] CancellationCapability CancellationMetadata() const;
  Result<ExecutionReceipt> Execute(const ExecutionCommand& command,
                                   const ExecutionContext& context);
  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context);
  Result<void> InitializeRuntimeRevisions(uint64_t runtime_revision,
                                          uint64_t config_revision);
  [[nodiscard]] CommittedRuntimeSnapshot Snapshot() const;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const VisualizationQuery& query) const;
  Result<void> ValidateReady();

 private:
  Result<void> EnsureReady();
  void PublishVisualization(VisualizationPhase phase, bool include_maps);
  void PublishEmptyVisualization();
  [[nodiscard]] std::shared_ptr<const RuntimeState> CommittedState() const;

  std::shared_ptr<ResourceGovernor> resource_governor_;
  std::optional<Error> initialization_error_;
  RuntimeStateStore runtime_state_store_;
  OutputRepository output_repository_;
  std::unique_ptr<StageCoordinator> coordinator_;
  VisualizationProjector visualization_projector_;
  std::atomic_flag execution_active_ = ATOMIC_FLAG_INIT;
};

}  // namespace open_lmm
