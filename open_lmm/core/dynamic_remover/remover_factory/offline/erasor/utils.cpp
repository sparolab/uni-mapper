
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */

#include "utils.hpp"



namespace common {

Config::Config(const open_lmm::Config& config) {
  max_range_ = config.param_cast<double>("dynamic_remover", "max_range");
  num_rings_ = config.param_cast<int>("dynamic_remover", "num_rings");
  num_sectors_ = config.param_cast<int>("dynamic_remover", "num_sectors");
  min_h_ = config.param_cast<double>("dynamic_remover", "min_h");
  max_h_ = config.param_cast<double>("dynamic_remover", "max_h");
  scan_ratio_threshold =
      config.param_cast<double>("dynamic_remover", "scan_ratio_threshold");
  minimum_num_pts = config.param_cast<int>("dynamic_remover", "minimum_num_pts");
  th_dist_ = config.param_cast<double>("dynamic_remover", "gf_dist_thr");
  iter_groundfilter_ = config.param_cast<int>("dynamic_remover", "gf_iter");
  num_lprs_ = config.param_cast<int>("dynamic_remover", "gf_num_lpr");
  th_seeds_heights_ = config.param_cast<double>("dynamic_remover", "gf_th_seeds_height");
  num_lowest_pts = config.param_cast<int>("dynamic_remover", "num_lowest_pts");
  query_voxel_size_ = config.param_cast<double>("dynamic_remover", "query_voxel_size");
  map_voxel_size_ = config.param_cast<double>("dynamic_remover", "map_voxel_size");
  global_voxelization_period_ = config.param_cast<int>("dynamic_remover", "voxelization_interval");
  removal_interval_ = config.param_cast<int>("dynamic_remover", "removal_interval");
  tf_z = config.param_cast<double>("dynamic_remover", "tf_z");
  replace_intensity = config.param_cast<bool>("dynamic_remover", "replace_intensity");
  internal_cpu_threads =
      config.param_cast<int>("dynamic_remover", "internal_cpu_threads");
}



}  // namespace common
