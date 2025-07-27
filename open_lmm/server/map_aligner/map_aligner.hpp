#pragma once

#include <filesystem>
#include <memory>
#include <optional>

// #include <Eigen/Core>
#include <Eigen/Geometry>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/core/loop_detector/loop_detector_base.hpp>
#include <open_lmm/utils/config.hpp>

namespace fs = std::filesystem;
namespace open_lmm {

class MapAlginer {
 public:
  explicit MapAlginer(const char agent_id,
                      const std::shared_ptr<SharedDatabase>& shared_data);
  ~MapAlginer();
  void process(fs::path data_dir_path);
  void parseConfig();
  std::tuple<PoseVec, ScanVec, ScanVec> runDataLoader(fs::path data_dir_path);

 private:
  const char agent_id_;
  std::shared_ptr<SharedDatabase> server_db_;
  std::unique_ptr<DataLoaderBase> data_loader_;
  std::unique_ptr<LoopDetectorBase> loop_detector_;
  std::unique_ptr<BackendOptimizerBase> backend_optimizer_;
  std::shared_ptr<DynamicRemoverBase> dynamic_remover_;

  std::optional<Config> config_data_loader_;
  std::optional<Config> config_loop_detector_;
  std::optional<Config> config_backend_optimizer_;
  std::optional<Config> config_dynamic_remover_;
};

}  // namespace open_lmm