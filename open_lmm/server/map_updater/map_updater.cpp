#include "map_updater.hpp"

namespace open_lmm {
MapUpdater::MapUpdater(const char agent_id,
                       const std::shared_ptr<SharedDatabase>& shared_data)
    : agent_id_(agent_id), server_db_(shared_data) {
  parseConfig();
  data_loader_ = DataLoaderBase::createInstance(config_data_loader_.value());
  dynamic_remover_ =
      DynamicRemoverBase::createInstance(config_dynamic_remover_.value());
  // TODO(gil) : add other modules (e.g. change detector)
}

void MapUpdater::parseConfig() {
  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  config_dynamic_remover_ =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  // TODO(gil) : add other modules (e.g. change detector)
}

MapUpdater::~MapUpdater() {}

pcl::PointCloud<pcl::PointXYZI>::Ptr MapUpdater::process(
    fs::path data_dir_path) {
  std::cout << "\n"
            << "\033[1m"
            << "============================================================"
            << std::endl;
  std::cout << "\033[1m" << "Map Updater : " << data_dir_path << std::endl;
  std::cout << "------------------------------------------------------------"
            << std::endl;

  auto raw_scans = data_loader_->loadRawScanData(data_dir_path);
  auto optimized_poses_pair = server_db_->db_optimized_poses[agent_id_];
  auto static_map = dynamic_remover_->process(raw_scans, optimized_poses_pair);

  // TODO(gil) : add other modules (e.g. change detector)

  std::cout << "\033[1m"
            << "============================================================"
            << std::endl;

  return static_map;
}

}  // namespace open_lmm