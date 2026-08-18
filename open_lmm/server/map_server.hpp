#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/server/stage_ports.hpp>

namespace open_lmm {

class StageExecutor;
class ResourceGovernor;

// Public runtime façade. Session ownership, transaction commits, algorithm
// assembly, output persistence and visualization snapshots live in the
// internal StageExecutor component.
class MapServer final : public StageRuntimePort {
 public:
  MapServer();
  explicit MapServer(
      std::filesystem::path config_directory,
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
  [[nodiscard]] CommittedSessionSnapshot Snapshot() const override;
  [[nodiscard]] Result<VisualizationSnapshot> Visualization(
      const AgentId& agent) const override;
  Result<void> ValidateReady();

 private:
  std::unique_ptr<StageExecutor> executor_;
};

}  // namespace open_lmm
