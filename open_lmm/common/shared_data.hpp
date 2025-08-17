#pragma once

#include <map>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>
#include <vector>

// #include <open_lmm/core/loop_detector/database/database_base.hpp>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>

#include "data_types.hpp"

namespace open_lmm {

// TODO(gil) : refactor centralized shared_data structure
struct SharedDatabase {
  std::vector<Eigen::Vector3f> db_merged_map;  // pointcloud type
  std::map<char, std::vector<Eigen::Vector3f>> db_original_maps;
  std::map<char, pcl::PointCloud<pcl::PointXYZI>::Ptr> db_optimized_maps;
  std::map<char, std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>> db_scans;
  std::map<char, std::vector<Eigen::Isometry3d>> db_odom_poses;
  std::map<char, std::vector<std::pair<int, Eigen::Isometry3d>>>
      db_optimized_poses;
  std::map<char, pcl::PointCloud<pcl::PointXYZ>> db_kdtree_poses;
  std::map<char, std::shared_ptr<IDescriptorKdtree>> db_descriptors;
  DatabaseKdtree total_db_descriptors;

  // // descriptor type
  gtsam::NonlinearFactorGraph graph;  // gtsam
  gtsam::Values values;
};

}  // namespace open_lmm
