#pragma once

#include <open_lmm/common/data_types.hpp>

namespace open_lmm {

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateRegistrationSubmap(
    const ScanVec& scans, const PoseVec& poses, int key, int search_num);

pcl::PointCloud<pcl::PointXYZI>::Ptr TransformRegistrationScan(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const Eigen::Isometry3d& transform);

}  // namespace open_lmm
