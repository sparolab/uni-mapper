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
  Result<void> RunStage(StageId stage) override;
  Result<void> RunNode(NodeId node, std::optional<char> agent) override;
  Result<void> RunOptimizeThrough(char target_agent) override;
  [[nodiscard]] std::vector<char> AgentIds() const override;
  [[nodiscard]] Result<VisualizationSnapshot> CreateVisualizationSnapshot(
      char agent, std::size_t max_points) const override;
  Result<void> ResetSession();

  Result<void> saveOptimizedPoses(const std::string& save_dir);
  Result<void> saveOptimizedMap(const std::string& save_dir);

 private:
  std::vector<AgentPipelineCtx> buildContexts() const;
  Result<void> ensureReady();
  Result<void> runDataLoadStage();
  Result<void> runAlignmentStage();
  Result<void> runMapUpdateStage();
  Result<void> runSaveStage();

  std::shared_ptr<SharedDatabase> shared_data_ =
      std::make_shared<SharedDatabase>();
  int agent_num_ = 0;
  std::vector<fs::path> data_dir_list_;
  std::string output_save_dir_;
  std::optional<Config> config_map_server_;
  bool enable_map_updater_ = false;
  int anchor_agent_index_ = 0;
  double save_voxel_size_ = 0.2;
  std::vector<AgentPipelineCtx> contexts_;
  std::shared_ptr<CancellationToken> cancellation_;

  std::shared_ptr<BackendOptimizerBase> backend_optimizer_;
  std::optional<Error> initialization_error_;
  std::optional<Config> config_data_loader_;
  std::optional<Config> config_loop_detector_;
  std::optional<Config> config_dynamic_remover_;
  mutable std::recursive_mutex state_mutex_;
};

}  // namespace open_lmm
