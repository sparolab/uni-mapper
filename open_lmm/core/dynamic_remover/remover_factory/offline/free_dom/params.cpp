#include "params.hpp"

#include "common_types.h"

#include <stdexcept>
#include <numbers>
#include <cmath>

FreeDomParams::FreeDomParams(const open_lmm::Config& config) {

  const auto positive_unsigned = [&](const char* key, int fallback) {
    const int value = config.param<int>("dynamic_remover", key, fallback);
    if (value <= 0) {
      throw std::invalid_argument(std::string("dynamic_remover.") + key +
                                  " must be a positive integer");
    }
    return static_cast<unsigned int>(value);
  };
  const auto nonnegative_unsigned = [&](const char* key, int fallback) {
    const int value = config.param<int>("dynamic_remover", key, fallback);
    if (value < 0) {
      throw std::invalid_argument(std::string("dynamic_remover.") + key +
                                  " must be a non-negative integer");
    }
    return static_cast<unsigned int>(value);
  };

  replace_intensity = config.param<bool>("dynamic_remover", "replace_intensity", true);
  const auto removal_level = config.param<std::string>(
      "dynamic_remover", "dynamic_removal_level", "aggressive");
  if (removal_level == "aggressive") {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::AGGRESSIVE_DYNAMIC);
  } else if (removal_level == "moderate") {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::MODERATE_DYNAMIC);
  } else if (removal_level == "conservative") {
    dynamic_removal_threshold =
        static_cast<uint8_t>(DynamicLevel::CONSERVATIVE_DYNAMIC);
  } else {
    throw std::invalid_argument(
        "dynamic_remover.dynamic_removal_level must be aggressive, "
        "moderate, or conservative");
  }
  sensor_min_range = config.param<double>("dynamic_remover", "min_range", 2.7);
  sensor_max_range = config.param<double>("dynamic_remover", "max_range", 80.0);
  sensor_min_z = config.param<double>("dynamic_remover", "min_z", -20.0);
  sensor_max_z = config.param<double>("dynamic_remover", "max_z", 20.0);

  sub_voxel_size = config.param<double>("dynamic_remover", "sub_voxel_size", 0.1);
  voxel_depth = positive_unsigned("voxel_depth", 2);
  block_depth = positive_unsigned("block_depth", 5);

  enable_local_map = config.param<bool>("dynamic_remover", "enable_local_map", false);
  local_map_range = config.param<double>("dynamic_remover", "local_map_range", 50.0);
  local_map_min_z = config.param<double>("dynamic_remover", "local_map_min_z", -20.0);
  local_map_max_z = config.param<double>("dynamic_remover", "local_map_max_z", 20.0);

  raycast_max_range = config.param<double>("dynamic_remover", "raycast_max_range", 80.0);
  raycast_min_z = config.param<double>("dynamic_remover", "raycast_min_z", -10.0);
  raycast_max_z = config.param<double>("dynamic_remover", "raycast_max_z", 10.0);

  counts_to_free = positive_unsigned("counts_to_free", 6);
  counts_to_revert = positive_unsigned("counts_to_revert", 20);

  conservative_connectivity = positive_unsigned("conservative_connectivity", 26);
  aggressive_connectivity = positive_unsigned("aggressive_connectivity", 124);

  enable_raycast_enhancement = config.param<bool>("dynamic_remover", "enable_raycast_enhancement", true);

  constexpr double kDegreesToRadians = std::numbers::pi_v<double> / 180.0;
  lidar_horizon_fov_rad =
      config.param<double>("dynamic_remover", "lidar_horizon_fov_degree", 360.0) *
      kDegreesToRadians;
  lidar_vertical_fov_upper_rad =
      config.param<double>("dynamic_remover", "lidar_vertical_fov_upper_degree", 2.0) *
      kDegreesToRadians;
  lidar_vertical_fov_lower_rad =
      config.param<double>("dynamic_remover", "lidar_vertical_fov_lower_degree", -24.8) *
      kDegreesToRadians;
  depth_image_vertical_lines =
      positive_unsigned("depth_image_vertical_lines", 64);

  depth_image_min_range = config.param<double>("dynamic_remover", "depth_image_min_range", 0.2);
  max_raycast_enhancement_range = config.param<double>("dynamic_remover", "max_raycast_enhancement_range", 80.0);
  raycast_enhancement_depth_margin = config.param<double>("dynamic_remover", "raycast_enhancement_depth_margin", 0.2);

  inpaint_size = nonnegative_unsigned("inpaint_size", 3);
  erosion_size = nonnegative_unsigned("erosion_size", 0);
  min_raycast_enhancement_area = config.param<double>("dynamic_remover", "min_raycast_enhancement_area", 0.0);
  depth_image_top_margin = config.param<double>("dynamic_remover", "depth_image_top_margin", 0);

  learn_fov = config.param<bool>("dynamic_remover", "learn_fov", false);
  enable_fov_mask = config.param<bool>("dynamic_remover", "enable_fov_mask", false);
  fov_mask_path = config.param<std::string>("dynamic_remover", "fov_mask_path", "");

  num_threads = positive_unsigned("num_threads", 8);
  if (!std::isfinite(sensor_min_range) || !std::isfinite(sensor_max_range) ||
      !std::isfinite(sensor_min_z) || !std::isfinite(sensor_max_z) ||
      !std::isfinite(sub_voxel_size) || !std::isfinite(local_map_range) ||
      !std::isfinite(local_map_min_z) || !std::isfinite(local_map_max_z) ||
      !std::isfinite(raycast_max_range) || !std::isfinite(raycast_min_z) ||
      !std::isfinite(raycast_max_z) ||
      sensor_min_range < 0.0 || sensor_max_range <= sensor_min_range ||
      sensor_min_z >= sensor_max_z || sub_voxel_size <= 0.0 ||
      voxel_depth == 0 || block_depth == 0 || raycast_max_range <= 0.0 ||
      counts_to_free == 0 || counts_to_revert == 0 || num_threads == 0 ||
      !std::isfinite(lidar_horizon_fov_rad) ||
      !std::isfinite(lidar_vertical_fov_upper_rad) ||
      !std::isfinite(lidar_vertical_fov_lower_rad) ||
      lidar_horizon_fov_rad <= 0.0 ||
      lidar_horizon_fov_rad > 2.0 * std::numbers::pi_v<double> ||
      lidar_vertical_fov_upper_rad <= lidar_vertical_fov_lower_rad ||
      depth_image_vertical_lines < 2) {
    throw std::invalid_argument("FreeDOM parameters are out of range");
  }
}
