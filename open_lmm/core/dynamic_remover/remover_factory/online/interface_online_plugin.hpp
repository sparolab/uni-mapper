#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class IOnlineRemoverPlugin {
 public:
  virtual ~IOnlineRemoverPlugin() = default;
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) = 0;
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() = 0;
};
