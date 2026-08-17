#include "registration_pointcloud.hpp"

#include <pcl/common/transforms.h>

#include <memory>

#include <open_lmm/common/profiling.hpp>
#include <open_lmm/common/rigid_transform.hpp>

namespace open_lmm {

pcl::PointCloud<pcl::PointXYZI>::Ptr CreateRegistrationSubmap(
    const ScanVec& scans, const PoseVec& poses, int key, int search_num) {
  OPEN_LMM_ZONE_N("Registration.CreateSubmap");
  auto submap = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  const int scan_count = static_cast<int>(scans.size());

  for (int offset = -search_num; offset <= search_num; ++offset) {
    const int nearby_key = key + offset;
    if (nearby_key < 0 || nearby_key >= scan_count) continue;

    auto transformed_scan =
        std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::transformPointCloud(*scans[nearby_key], *transformed_scan,
                             poses[nearby_key].matrix());
    *submap += *transformed_scan;
  }

  const Eigen::Isometry3d inverse_pose = InvertRigidTransform(poses[key]);
  pcl::transformPointCloud(*submap, *submap, inverse_pose.matrix());
  return submap;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr TransformRegistrationScan(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    const Eigen::Isometry3d& transform) {
  auto transformed_scan =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::transformPointCloud(*scan, *transformed_scan, transform.matrix());
  return transformed_scan;
}

}  // namespace open_lmm
