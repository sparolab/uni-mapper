#pragma once

#include <filesystem>
#include <functional>
#include <memory>
#include <optional>

#include <runtime/execution/stage_ports.hpp>
#include <config/bootstrap/bootstrap_config.hpp>

namespace open_lmm {

class StageExecutor;
class ResourceGovernor;
struct StageExecutorDiagnostics;

// Concrete runtime port. Runtime ownership, transaction commits, algorithm
// assembly, output persistence and visualization snapshots live in the
// internal StageExecutor component.
class MapServer final : public StageRuntimePort {
 public:
  explicit MapServer(
      BootstrapConfigSnapshot bootstrap_config,
      std::optional<std::filesystem::path> output_directory = std::nullopt,
      std::shared_ptr<ResourceGovernor> resource_governor = {},
      std::function<void()> before_presentation_publish = {});
  ~MapServer() override;

  MapServer(const MapServer&) = delete;
  MapServer& operator=(const MapServer&) = delete;

  Result<void> process();
  [[nodiscard]] CancellationCapability CancellationMetadata() const override;
  Result<ExecutionReceipt> Execute(
      const ExecutionCommand& command,
      const ExecutionContext& context) override;
  Result<ConfigCommandReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context) override;
  [[nodiscard]] CommittedRuntimeSnapshot Snapshot() const override;
  [[nodiscard]] Result<CommittedConfigDocuments> ConfigDocuments()
      const override;
  [[nodiscard]] Result<ConfigCandidateCatalog> ConfigCandidates()
      const override;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const AgentId& agent) const;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const VisualizationQuery& query) const override;
  [[nodiscard]] StageExecutorDiagnostics Diagnostics() const;
  Result<void> InitializeRuntimeRevisions(uint64_t runtime_revision,
                                          uint64_t config_revision) override;
  void RecordRecoveryRequired(
      std::shared_ptr<const Error> recovery_required) noexcept override;
  Result<void> ValidateReady();

 private:
  std::unique_ptr<StageExecutor> executor_;
};

}  // namespace open_lmm
