#pragma once

#include <algorithm>
#include <atomic>
#include <filesystem>
#include <memory>
#include <mutex>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/server/session_manager.hpp>
#include <open_lmm/server/session_state.hpp>
#include <open_lmm/server/stage_runner.hpp>
#include <open_lmm/utils/config.hpp>
#include <vector>

namespace fs = std::filesystem;
namespace open_lmm {

// Internal runtime/session owner. MapServer deliberately delegates to this
// component so its public StageRunner surface remains a thin façade.
class StageExecutor {
 public:
  StageExecutor();
  ~StageExecutor();

  Result<void> process();
  void SetCancellationToken(std::shared_ptr<CancellationToken> token);
  [[nodiscard]] CancellationCapability CancellationMetadata() const;
  void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker> broker);
  Result<void> RunStage(StageId stage);
  Result<void> RunNode(NodeId node, std::optional<AgentId> agent);
  Result<void> RunOptimizeThrough(const AgentId& target_agent);
  Result<void> Reconfigure(ConfigDomain domain, uint64_t revision);
  [[nodiscard]] std::vector<AgentId> AgentIds() const;
  [[nodiscard]] std::optional<CommittedSessionSnapshot>
  SessionSnapshot() const;
  [[nodiscard]] Result<VisualizationSnapshot> CreateVisualizationSnapshot(
      const AgentId& agent) const;
  Result<void> ValidateReady();

 private:
  void parseConfig();
  std::vector<AgentPipelineCtx> buildContexts() const;
  Result<void> ensureReady();
  Result<void> validateInputFiles() const;
  Result<void> runDataLoadStage();
  Result<void> runAlignmentStage();
  Result<void> runLoopDetectThrough(const AgentId& target_agent);
  Result<void> prepareAlignmentArtifacts(
      const SessionState& state, PendingOutputSet& pending,
      ArtifactRepository& artifacts) const;
  void computeAlignmentFingerprints();
  void loadAlignmentCache();
  void installStoredAlignments(SharedDatabase& database) const;
  Result<void> runMapUpdateStage();
  Result<void> runSaveStage();
  Result<void> runOptimizeThrough(const AgentId& target_agent);
  void publishVisualizationState(
      const std::shared_ptr<const SessionState>& state, bool include_maps);
  void publishEmptyVisualizationState(uint64_t session_revision);
  [[nodiscard]] std::shared_ptr<const SessionState> committedState() const;
  Result<std::shared_ptr<BackendOptimizerBase>> createOptimizer() const;
  Result<void> commitTransaction(SessionTransaction transaction,
                                 PendingOutputSet* pending = nullptr,
                                 bool check_cancellation = true);
  std::unique_ptr<ArtifactRepository> artifactEditor(
      const SessionState& state) const;
  Result<void> prepareOptimizedPoses(
      const SessionState& state, const std::string& save_dir,
      PendingOutputSet& pending, ArtifactRepository& artifacts);
  Result<void> prepareOptimizedMap(
      const SessionState& state, const std::string& save_dir,
      PendingOutputSet& pending, ArtifactRepository& artifacts);

  struct VisualizationState {
    uint64_t revision = 0;
    std::map<AgentId, VisualizationSnapshot> agents;
    std::map<AgentId, fs::path> map_paths;
  };

  int agent_num_ = 0;
  std::vector<AgentId> configured_agent_ids_;
  std::vector<AgentId> agent_ids_;
  AgentSymbolCatalogHandle agent_catalog_;
  std::vector<fs::path> data_dir_list_;
  std::string output_save_dir_;
  fs::path alignment_cache_path_;
  std::string alignment_config_fingerprint_;
  std::map<AgentId, std::string> alignment_input_fingerprints_;
  std::string alignment_session_fingerprint_;
  std::map<AgentId, StoredAlignment> cached_alignments_;
  std::optional<Config> config_map_server_;
  bool enable_map_updater_ = false;
  int anchor_agent_index_ = 0;
  double save_voxel_size_ = 0.2;
  std::shared_ptr<CancellationToken> cancellation_;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback_;

  std::optional<Error> initialization_error_;
  std::optional<Config> config_data_loader_;
  std::optional<Config> config_loop_detector_;
  std::optional<Config> config_backend_optimizer_;
  std::optional<Config> config_dynamic_remover_;
  mutable std::mutex state_mutex_;
  SessionManager session_manager_;
  OutputRepository output_repository_;
  std::shared_ptr<const VisualizationState> visualization_state_ =
      std::make_shared<VisualizationState>();
  std::atomic_flag execution_active_ = ATOMIC_FLAG_INIT;
  bool inputs_validated_ = false;
};

}  // namespace open_lmm
