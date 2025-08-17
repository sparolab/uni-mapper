#pragma once

#include <filesystem>
#include <memory>
#include <optional>

// #include <Eigen/Core>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/utils/config.hpp>

#include <Eigen/Geometry>

namespace fs = std::filesystem;
namespace open_lmm {

class MapUpdater {
 public:
  explicit MapUpdater(const char agent_id,
                      const std::shared_ptr<SharedDatabase>& shared_data);
  ~MapUpdater();
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(fs::path data_dir_path);
  void parseConfig();

 private:
  const char agent_id_;
  std::shared_ptr<SharedDatabase> server_db_;
  std::unique_ptr<DataLoaderBase> data_loader_;
  std::shared_ptr<DynamicRemoverBase> dynamic_remover_;

  std::optional<Config> config_data_loader_;
  std::optional<Config> config_dynamic_remover_;
};

}  // namespace open_lmm