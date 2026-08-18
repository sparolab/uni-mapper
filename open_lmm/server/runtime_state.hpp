#pragma once

#include <filesystem>
#include <memory>
#include <map>
#include <string>
#include <vector>

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <open_lmm/server/stage_runner.hpp>
#include <open_lmm/server/resource_governor.hpp>

namespace open_lmm {

class SchemaRegistry;

struct RuntimeRootConfig {
  std::vector<std::filesystem::path> data_directories;
  std::filesystem::path output_directory;
  int anchor_agent_index = 0;
  bool enable_map_updater = false;
  double save_voxel_size = 0.2;
  bool parallel_data_load = false;
  bool parallel_map_update = false;
  std::size_t max_parallel_agents = 1;
};

struct RuntimeConfigDocument {
  std::filesystem::path path;
  std::string canonical_json;
};

// Canonical source documents and paths are revisioned with the typed config.
// Reconfiguration must derive its candidate from this snapshot, never from a
// mutable coordinator-side mirror.
struct RuntimeConfigDocuments {
  RuntimeConfigDocument root;
  RuntimeConfigDocument map_server;
  RuntimeConfigDocument data_loader;
  RuntimeConfigDocument loop_detector;
  RuntimeConfigDocument optimizer;
  RuntimeConfigDocument dynamic_remover;
};

struct AlignmentArtifactMetadata {
  std::filesystem::path cache_path;
  std::map<AgentId, std::string> input_fingerprints;
  std::string runtime_fingerprint;
};

// Immutable, validated configuration snapshot owned by one active runtime.
struct RuntimeConfig {
  uint64_t revision = 1;
  RuntimeRootConfig root;
  std::shared_ptr<const DataLoaderConfig> data_loader;
  std::shared_ptr<const LoopDetectorConfig> loop_detector;
  std::shared_ptr<const OptimizerConfig> optimizer;
  std::shared_ptr<const DynamicRemoverConfig> dynamic_remover;
  std::shared_ptr<const MapSaveConfig> map_save;
  std::string fingerprint;
  std::shared_ptr<const RuntimeConfigDocuments> documents;
  std::shared_ptr<const AlignmentArtifactMetadata> alignment_artifacts;
  std::shared_ptr<const SchemaRegistry> schema_registry;
};

struct OptimizerStateMetadata {
  std::vector<AgentId> processed_agents;
};

struct RuntimePayload {
  std::vector<AgentPipelineCtx> contexts;
  std::shared_ptr<const SharedDatabase> database;
  std::shared_ptr<BackendOptimizerBase> optimizer;
  std::map<AgentId, std::shared_ptr<MemoryReservation>>
      resident_memory_reservations;
};

struct RuntimeState {
  uint64_t revision = 0;
  std::shared_ptr<const RuntimeConfig> config;
  std::vector<AgentId> ordered_agents;
  AgentSymbolCatalogHandle agent_catalog;
  std::shared_ptr<const RuntimePayload> payload;
  OptimizerStateMetadata optimizer;
  std::vector<ArtifactMetadata> artifacts;
};

class RuntimeTransaction {
 public:
  explicit RuntimeTransaction(std::shared_ptr<const RuntimeState> base);

  [[nodiscard]] uint64_t BaseRevision() const;
  [[nodiscard]] const std::shared_ptr<const RuntimeState>& Base() const;
  RuntimeState& Working();
  void SetPayload(std::shared_ptr<const RuntimePayload> payload);

  Result<std::shared_ptr<const RuntimeState>> Finalize(
      const std::shared_ptr<CancellationToken>& cancellation) &&;

 private:
  Result<void> Validate() const;

  std::shared_ptr<const RuntimeState> base_;
  std::unique_ptr<RuntimeState> working_;
};

}  // namespace open_lmm
