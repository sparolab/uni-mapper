#include "map_aligner.hpp"

namespace open_lmm {
MapAlginer::MapAlginer(const char agent_id,
                       const std::shared_ptr<SharedDatabase>& shared_data)
    : agent_id_(agent_id), server_db_(shared_data) {
  parseConfig();
  data_loader_ = DataLoaderBase::createInstance(config_data_loader_.value());
  loop_detector_ =
      LoopDetectorBase::createInstance(config_loop_detector_.value());
  backend_optimizer_ =
      BackendOptimizerBase::createInstance(config_backend_optimizer_.value());
}

void MapAlginer::parseConfig() {
  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  config_loop_detector_ =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  config_backend_optimizer_ =
      Config(GlobalConfig::get_global_config_path("config_backend_optimizer"));
}

MapAlginer::~MapAlginer() {}

// std::tuple<PoseVec, ScanVec, ScanVec> MapAlginer::runDataLoader(
//     fs::path data_dir_path) {
//   return data_loader_->process(server_db_, agent_id_, data_dir_path);
//   //   PoseVec raw_poses = data_loader_->loadPoseData(data_dir_path);
//   //   ScanVec raw_scans = data_loader_->loadRawScanData(data_dir_path);
//   //   ScanVec filtered_scans =
//   //   data_loader_->loadFilteredScanData(data_dir_path); return {raw_poses,
//   //   raw_scans, filtered_scans};
//   // server_db_->db_odom_poses[agent_id_] = poses;
//   // server_db_->db_scans[agent_id_] = filtered_scans;
// }

void MapAlginer::process(fs::path data_dir_path) {
  // auto [poses, raw_scans, filtered_scans] = runDataLoader(data_dir_path);
  //////////////////////////////////////////////////////////////////////

  // TODO(gil): Refactor needed — remove dependency on centralized `server_db_`
  //  aim for a more functional design
  std::cout << "\n"
            << "\033[1m"
            << "============================================================"
            << std::endl;
  std::cout << "\033[1m" << "Map Aligner : " << data_dir_path << std::endl;
  std::cout << "------------------------------------------------------------"
            << std::endl;

  auto [poses, raw_scans, filtered_scans] =
      data_loader_->process(server_db_, agent_id_, data_dir_path);

  auto [intra_loops, inter_loops] =
      loop_detector_->process(server_db_, agent_id_, filtered_scans);

  auto optimized_poses = backend_optimizer_->process(
      server_db_, agent_id_, poses, intra_loops, inter_loops);

  std::cout << "\033[1m"
            << "============================================================"
            << std::endl;
}

}  // namespace open_lmm