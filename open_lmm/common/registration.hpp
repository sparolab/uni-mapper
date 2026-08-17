#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <optional>

#include "data_types.hpp"

namespace open_lmm {

// scans_to/poses_to: loop.to 에이전트의 스캔과 포즈
// scan_from: loop.from 에이전트의 소스 스캔 (loop.from.second 인덱스)
std::optional<Eigen::Isometry3d> registerPointCloud(
    const ScanVec& scans_to,
    const PoseVec& poses_to,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_from,
    const LoopPair& loop_pair,
    int search_num);

}  // namespace open_lmm
