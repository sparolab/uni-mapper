#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/server/stage_ports.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>

namespace open_lmm {

class StageExecutor;
class ResourceGovernor;

// Public runtime façade. Runtime ownership, transaction commits, algorithm
// assembly, output persistence and visualization snapshots live in the
// internal StageExecutor component.
class MapServer final : public StageRuntimePort {
 public:
  explicit MapServer(
      BootstrapConfigSnapshot bootstrap_config,
      std::optional<std::filesystem::path> output_directory = std::nullopt,
      std::shared_ptr<ResourceGovernor> resource_governor = {});
  ~MapServer() override;

  MapServer(const MapServer&) = delete;
  MapServer& operator=(const MapServer&) = delete;

  Result<void> process();
  [[nodiscard]] CancellationCapability CancellationMetadata() const override;
  Result<ExecutionReceipt> Execute(
      const ExecutionCommand& command,
      const ExecutionContext& context) override;
  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context) override;
  [[nodiscard]] CommittedRuntimeSnapshot Snapshot() const override;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const AgentId& agent) const override;
  Result<void> InitializeRuntimeRevisions(uint64_t runtime_revision,
                                          uint64_t config_revision) override;
  Result<void> ValidateReady();

 private:
  std::unique_ptr<StageExecutor> executor_;
};

}  // namespace open_lmm
