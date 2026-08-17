#pragma once

#include <algorithm>
#include <filesystem>
#include <memory>
#include <mutex>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/server/stage_runner.hpp>
#include <open_lmm/utils/config.hpp>
#include <vector>

namespace fs = std::filesystem;
namespace open_lmm {

class MapServer : public StageRunner {
 public:
  MapServer();
  ~MapServer() override;
  void parseConfig();

  // Backward-compatible full pipeline entry point.
  Result<void> process();
  void SetCancellationToken(std::shared_ptr<CancellationToken> token) override;
  void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker> broker) override;
  Result<void> RunStage(StageId stage) override;
  Result<void> RunNode(NodeId node, std::optional<char> agent) override;
  Result<void> RunOptimizeThrough(char target_agent) override;
  Result<void> Reconfigure(ConfigDomain domain) override;
  [[nodiscard]] std::vector<char> AgentIds() const override;
  [[nodiscard]] Result<VisualizationSnapshot> CreateVisualizationSnapshot(
      char agent) const override;
  Result<void> ValidateReady();

  Result<void> saveOptimizedPoses(const std::string& save_dir);
  Result<void> saveOptimizedMap(const std::string& save_dir);

 private:
  std::vector<AgentPipelineCtx> buildContexts() const;
  Result<void> ensureReady();
  Result<void> validateInputFiles() const;
  Result<void> runDataLoadStage();
  Result<void> runAlignmentStage();
  Result<void> saveAlignmentArtifacts() const;
  void computeAlignmentFingerprints();
  void loadAlignmentCache();
  void installStoredAlignments();
  Result<void> runMapUpdateStage();
  Result<void> runSaveStage();

  std::shared_ptr<SharedDatabase> shared_data_ =
      std::make_shared<SharedDatabase>();
  int agent_num_ = 0;
  std::vector<fs::path> data_dir_list_;
  std::string output_save_dir_;
  fs::path alignment_cache_path_;
  std::string alignment_config_fingerprint_;
  std::map<char, std::string> alignment_input_fingerprints_;
  std::string alignment_session_fingerprint_;
  std::map<char, StoredAlignment> cached_alignments_;
  std::optional<Config> config_map_server_;
  bool enable_map_updater_ = false;
  int anchor_agent_index_ = 0;
  double save_voxel_size_ = 0.2;
  std::vector<AgentPipelineCtx> contexts_;
  std::shared_ptr<CancellationToken> cancellation_;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback_;

  std::shared_ptr<BackendOptimizerBase> backend_optimizer_;
  std::optional<Error> initialization_error_;
  std::optional<Config> config_data_loader_;
  std::optional<Config> config_loop_detector_;
  std::optional<Config> config_dynamic_remover_;
  mutable std::recursive_mutex state_mutex_;
  bool inputs_validated_ = false;
};

}  // namespace open_lmm
