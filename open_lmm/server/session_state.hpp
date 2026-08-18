#pragma once

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <open_lmm/server/stage_runner.hpp>

namespace open_lmm {

struct SessionRootConfig {
  std::vector<std::filesystem::path> data_directories;
  std::filesystem::path output_directory;
  int anchor_agent_index = 0;
  bool enable_map_updater = false;
  double save_voxel_size = 0.2;
};

// Immutable, validated configuration snapshot owned by one runtime session.
struct SessionConfig {
  uint64_t revision = 1;
  SessionRootConfig root;
  std::shared_ptr<const DataLoaderConfig> data_loader;
  std::shared_ptr<const LoopDetectorConfig> loop_detector;
  std::shared_ptr<const OptimizerConfig> optimizer;
  std::shared_ptr<const DynamicRemoverConfig> dynamic_remover;
  std::shared_ptr<const MapSaveConfig> map_save;
  std::string fingerprint;
};

struct OptimizerStateMetadata {
  std::vector<AgentId> processed_agents;
};

struct SessionPayload {
  std::vector<AgentPipelineCtx> contexts;
  std::shared_ptr<const SharedDatabase> database;
  std::shared_ptr<BackendOptimizerBase> optimizer;
};

struct SessionState {
  uint64_t revision = 0;
  std::shared_ptr<const SessionConfig> config;
  std::vector<AgentId> ordered_agents;
  AgentSymbolCatalogHandle agent_catalog;
  std::shared_ptr<const SessionPayload> payload;
  OptimizerStateMetadata optimizer;
  std::vector<ArtifactMetadata> artifacts;
};

class SessionTransaction {
 public:
  explicit SessionTransaction(std::shared_ptr<const SessionState> base);

  [[nodiscard]] uint64_t BaseRevision() const;
  [[nodiscard]] const std::shared_ptr<const SessionState>& Base() const;
  SessionState& Working();
  void SetPayload(std::shared_ptr<const SessionPayload> payload);

  Result<std::shared_ptr<const SessionState>> Finalize(
      const std::shared_ptr<CancellationToken>& cancellation) &&;

 private:
  Result<void> Validate() const;

  std::shared_ptr<const SessionState> base_;
  std::unique_ptr<SessionState> working_;
};

}  // namespace open_lmm
