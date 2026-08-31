
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-04 23:19
 * Description: Config header
 */


#pragma once
#include <iostream>
#include <string>
#include <pcl/point_types.h>
#include <config/document/config.hpp>


// CHANGE Point Type Here!!! If you want to use XYZI, change to pcl::PointXYZI
// typedef pcl::PointXYZ PointT;
typedef pcl::PointXYZI PointT;
// typedef pcl::PointXYZRGB PointT;

namespace common {
struct Config {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Config() = default;
  explicit Config(const open_lmm::Config& config);
  ~Config() = default;

 public:
  /**< Parameters of MapUpdater*/
  double map_voxel_size_ = 0.2;
  double query_voxel_size_ = 0.2;
  int global_voxelization_period_;

  /**< Params. of Volume of Interest (VoI) */
  double max_range_;
  int num_rings_, num_sectors_;
  double min_h_, max_h_;
  double scan_ratio_threshold;

  double submap_size_;
  double submap_center_x_;
  double submap_center_y_;

  double th_seeds_heights_ = 0.5;
  double th_dist_ = 0.05;
  int num_lprs_ = 10;
  int minimum_num_pts = 6;
  int iter_groundfilter_ = 3;
  int num_lowest_pts = 5;
  std::string mode = "naive";
  bool replace_intensity = true;
  int removal_interval_ = 2;

  // tf lidar to body
  double tf_x = 0.0;
  double tf_y = 0.0;
  double tf_z = 0.0;
  
  bool is_large_scale_ = false;
  int internal_cpu_threads = 1;
};

}  // namespace common
