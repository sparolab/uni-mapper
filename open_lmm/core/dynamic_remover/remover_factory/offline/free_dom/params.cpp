#include "params.hpp"

FreeDomParams::FreeDomParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_dynamic_remover"));

  replace_intensity = config.param<bool>("dynamic_remover", "replace_intensity", true);
  sensor_min_range = config.param<double>("dynamic_remover", "min_range", 2.7);
  sensor_max_range = config.param<double>("dynamic_remover", "max_range", 80.0);
  sensor_min_z = config.param<double>("dynamic_remover", "min_z", -20.0);
  sensor_max_z = config.param<double>("dynamic_remover", "max_z", 20.0);

  sub_voxel_size = config.param<double>("dynamic_remover", "sub_voxel_size", 0.1);
  voxel_depth = config.param<int>("dynamic_remover", "voxel_depth", 2);
  block_depth = config.param<int>("dynamic_remover", "block_depth", 5);

  enable_local_map = config.param<bool>("dynamic_remover", "enable_local_map", false);
  local_map_range = config.param<double>("dynamic_remover", "local_map_range", 50.0);
  local_map_min_z = config.param<double>("dynamic_remover", "local_map_min_z", -20.0);
  local_map_max_z = config.param<double>("dynamic_remover", "local_map_max_z", 20.0);

  raycast_max_range = config.param<double>("dynamic_remover", "raycast_max_range", 80.0);
  raycast_min_z = config.param<double>("dynamic_remover", "raycast_min_z", -10.0);
  raycast_max_z = config.param<double>("dynamic_remover", "raycast_max_z", 10.0);

  counts_to_free = config.param<int>("dynamic_remover", "counts_to_free", 6);
  counts_to_revert = config.param<int>("dynamic_remover", "counts_to_revert", 20);

  conservative_connectivity = config.param<int>("dynamic_remover", "conservative_connectivity", 26);
  aggressive_connectivity = config.param<int>("dynamic_remover", "aggressive_connectivity", 124);

  enable_raycast_enhancement = config.param<bool>("dynamic_remover", "enable_raycast_enhancement", true);

  lidar_horizon_fov = config.param<double>("dynamic_remover", "lidar_horizon_fov_degree", 360.0);
  lidar_vertical_fov_upper = config.param<double>("dynamic_remover", "lidar_vertical_fov_upper_degree", 2.0);
  lidar_vertical_fov_lower = config.param<double>("dynamic_remover", "lidar_vertical_fov_lower_degree", -24.8);
  depth_image_vertical_lines = config.param<int>("dynamic_remover", "depth_image_vertical_lines", 64);

  depth_image_min_range = config.param<double>("dynamic_remover", "depth_image_min_range", 0.2);
  max_raycast_enhancement_range = config.param<double>("dynamic_remover", "max_raycast_enhancement_range", 80.0);
  raycast_enhancement_depth_margin = config.param<double>("dynamic_remover", "raycast_enhancement_depth_margin", 0.2);

  inpaint_size = config.param<int>("dynamic_remover", "inpaint_size", 3);
  erosion_size = config.param<int>("dynamic_remover", "erosion_size", 0);
  min_raycast_enhancement_area = config.param<double>("dynamic_remover", "min_raycast_enhancement_area", 0.0);
  depth_image_top_margin = config.param<double>("dynamic_remover", "depth_image_top_margin", 0);

  learn_fov = config.param<bool>("dynamic_remover", "learn_fov", false);
  enable_fov_mask = config.param<bool>("dynamic_remover", "enable_fov_mask", false);
  fov_mask_path = config.param<std::string>("dynamic_remover", "fov_mask_path", "");

  num_threads = config.param<int>("dynamic_remover", "num_threads", 8);
}

