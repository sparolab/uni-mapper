#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

class IOfflineRemoverPlugin {
 public:
  virtual ~IOfflineRemoverPlugin() = default;
  virtual void run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
                   Eigen::Isometry3d& optimized_pose) = 0;
  virtual void setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) = 0;
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() = 0;
};
