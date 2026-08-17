#pragma once

#include <algorithm>
#include <filesystem>
#include <memory>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/utils/config.hpp>
#include <vector>

namespace fs = std::filesystem;
namespace open_lmm {

class MapServer {
 public:
  MapServer();
  ~MapServer();
  void parseConfig();
  Result<void> process();
  void saveOptimizedPoses(const std::string& save_dir);
  Result<void> saveOptimizedMap(const std::string& save_dir);

 private:
  std::vector<AgentPipelineCtx> buildContexts() const;

  std::shared_ptr<SharedDatabase> shared_data_ =
      std::make_shared<SharedDatabase>();
  int agent_num_;
  std::vector<fs::path> data_dir_list_;
  std::string output_save_dir_;
  std::optional<Config> config_map_server_;
  bool   enable_map_updater_;
  int    anchor_agent_index_;
  double save_voxel_size_;

  // 에이전트 간 누적 팩터 상태 공유
  std::shared_ptr<BackendOptimizerBase> backend_optimizer_;
  std::optional<Error> initialization_error_;

  // 노드 생성용 config
  std::optional<Config> config_data_loader_;
  std::optional<Config> config_loop_detector_;
  std::optional<Config> config_dynamic_remover_;
};

}  // namespace open_lmm
