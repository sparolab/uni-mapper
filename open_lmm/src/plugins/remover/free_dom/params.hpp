#include <config/document/config.hpp>
#include <cstdint>
#include <string>

struct FreeDomParams {
  explicit FreeDomParams(const open_lmm::Config& config);
  ~FreeDomParams() = default;

  // bool replace_intensity;
  // double voxel_size;
  // double occupancy_sigma;
  // double free_sigma;
  // double belief_threshold;
  // int conv_size;
  // int local_window_size;
  // int global_window_size;
  // int min_otsu;
  // double max_range;
  // double min_range;

  bool replace_intensity;
  uint8_t dynamic_removal_threshold;
  double sensor_min_range;
  double sensor_max_range;
  double sensor_min_z;
  double sensor_max_z;

  double sub_voxel_size;
  unsigned int voxel_depth;
  unsigned int block_depth;

  bool enable_local_map;
  double local_map_range;
  double local_map_min_z;
  double local_map_max_z;

  double raycast_max_range;
  double raycast_min_z;
  double raycast_max_z;

  unsigned int counts_to_free;
  unsigned int counts_to_revert;

  unsigned int conservative_connectivity;
  unsigned int aggressive_connectivity;

  bool enable_raycast_enhancement;

  // Legacy mode preserves the historical degree-as-radian projection.
  // Physical mode performs the mathematically correct degree-to-radian
  // conversion at this single boundary.
  double lidar_horizon_fov_rad;
  double lidar_vertical_fov_upper_rad;
  double lidar_vertical_fov_lower_rad;
  unsigned int depth_image_vertical_lines;

  double depth_image_min_range;
  double max_raycast_enhancement_range;
  double raycast_enhancement_depth_margin;

  unsigned int inpaint_size;
  unsigned int erosion_size;
  double min_raycast_enhancement_area;
  double depth_image_top_margin;

  bool learn_fov;
  bool enable_fov_mask;
  std::string fov_mask_path;

  unsigned int num_threads;
};
