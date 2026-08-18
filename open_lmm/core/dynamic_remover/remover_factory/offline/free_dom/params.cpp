#include "params.hpp"

#include "common_types.h"

#include <numbers>

FreeDomParams::FreeDomParams(const open_lmm::Config& config) {

  const auto unsigned_value = [&](const char* key) {
    return static_cast<unsigned int>(
        config.param_cast<int>("dynamic_remover", key));
  };

  replace_intensity = config.param_cast<bool>("dynamic_remover", "replace_intensity");
  const auto removal_level = config.param_cast<std::string>("dynamic_remover", "dynamic_removal_level");
  if (removal_level == "aggressive") {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::AGGRESSIVE_DYNAMIC);
  } else if (removal_level == "moderate") {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::MODERATE_DYNAMIC);
  } else {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::CONSERVATIVE_DYNAMIC);
  }
  sensor_min_range = config.param_cast<double>("dynamic_remover", "min_range");
  sensor_max_range = config.param_cast<double>("dynamic_remover", "max_range");
  sensor_min_z = config.param_cast<double>("dynamic_remover", "min_z");
  sensor_max_z = config.param_cast<double>("dynamic_remover", "max_z");

  sub_voxel_size = config.param_cast<double>("dynamic_remover", "sub_voxel_size");
  voxel_depth = unsigned_value("voxel_depth");
  block_depth = unsigned_value("block_depth");

  enable_local_map = config.param_cast<bool>("dynamic_remover", "enable_local_map");
  local_map_range = config.param_cast<double>("dynamic_remover", "local_map_range");
  local_map_min_z = config.param_cast<double>("dynamic_remover", "local_map_min_z");
  local_map_max_z = config.param_cast<double>("dynamic_remover", "local_map_max_z");

  raycast_max_range = config.param_cast<double>("dynamic_remover", "raycast_max_range");
  raycast_min_z = config.param_cast<double>("dynamic_remover", "raycast_min_z");
  raycast_max_z = config.param_cast<double>("dynamic_remover", "raycast_max_z");

  counts_to_free = unsigned_value("counts_to_free");
  counts_to_revert = unsigned_value("counts_to_revert");

  conservative_connectivity = unsigned_value("conservative_connectivity");
  aggressive_connectivity = unsigned_value("aggressive_connectivity");

  enable_raycast_enhancement = config.param_cast<bool>("dynamic_remover", "enable_raycast_enhancement");

  constexpr double kDegreesToRadians = std::numbers::pi_v<double> / 180.0;
  lidar_horizon_fov_rad =
      config.param_cast<double>("dynamic_remover", "lidar_horizon_fov_degree") *
      kDegreesToRadians;
  lidar_vertical_fov_upper_rad =
      config.param_cast<double>("dynamic_remover", "lidar_vertical_fov_upper_degree") *
      kDegreesToRadians;
  lidar_vertical_fov_lower_rad =
      config.param_cast<double>("dynamic_remover", "lidar_vertical_fov_lower_degree") *
      kDegreesToRadians;
  depth_image_vertical_lines =
      unsigned_value("depth_image_vertical_lines");

  depth_image_min_range = config.param_cast<double>("dynamic_remover", "depth_image_min_range");
  max_raycast_enhancement_range = config.param_cast<double>("dynamic_remover", "max_raycast_enhancement_range");
  raycast_enhancement_depth_margin = config.param_cast<double>("dynamic_remover", "raycast_enhancement_depth_margin");

  inpaint_size = unsigned_value("inpaint_size");
  erosion_size = unsigned_value("erosion_size");
  min_raycast_enhancement_area = config.param_cast<double>("dynamic_remover", "min_raycast_enhancement_area");
  depth_image_top_margin = config.param_cast<double>("dynamic_remover", "depth_image_top_margin");

  learn_fov = config.param_cast<bool>("dynamic_remover", "learn_fov");
  enable_fov_mask = config.param_cast<bool>("dynamic_remover", "enable_fov_mask");
  fov_mask_path = config.param_cast<std::string>("dynamic_remover", "fov_mask_path");

  num_threads = unsigned_value("num_threads");
}
