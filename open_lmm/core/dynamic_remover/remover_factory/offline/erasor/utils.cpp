
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */

#include "utils.hpp"



namespace common {

Config::Config() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_dynamic_remover"));
  max_range_ = config.param<double>("dynamic_remover", "max_range", 80.0);
  num_rings_ = config.param<int>("dynamic_remover", "num_rings", 20);
  num_sectors_ = config.param<int>("dynamic_remover", "num_sectors", 108);
  min_h_ = config.param<double>("dynamic_remover", "min_h", -1.7);
  max_h_ = config.param<double>("dynamic_remover", "max_h", 3.1);
  scan_ratio_threshold =
      config.param<double>("dynamic_remover", "scan_ratio_threshold", 0.20);
  minimum_num_pts = config.param<int>("dynamic_remover", "minimum_num_pts", 6);
  th_dist_ = config.param<double>("dynamic_remover", "gf_dist_thr", 0.125);
  iter_groundfilter_ = config.param<int>("dynamic_remover", "gf_iter", 3);
  num_lprs_ = config.param<int>("dynamic_remover", "gf_num_lpr", 10);
  th_seeds_heights_ = config.param<double>("dynamic_remover", "gf_th_seeds_height", 0.5);
  num_lowest_pts = config.param<int>("dynamic_remover", "num_lowest_pts", 1);
  query_voxel_size_ = config.param<double>("dynamic_remover", "query_voxel_size", 0.1);
  map_voxel_size_ = config.param<double>("dynamic_remover", "map_voxel_size", 0.2);
  global_voxelization_period_ = config.param<int>("dynamic_remover", "voxelization_interval", 10);
  removal_interval_ = config.param<int>("dynamic_remover", "removal_interval", 5);
  tf_z = config.param<double>("dynamic_remover", "tf_z", 0.7);
  replace_intensity = config.param<bool>("dynamic_remover", "replace_intensity", false);
}



}  // namespace common