#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

class IDescriptorKdtree {
 public:
  virtual ~IDescriptorKdtree() = default;
  virtual const Eigen::MatrixXd& getDescriptor() const = 0;
  virtual const Eigen::VectorXd& getDescriptorKey() const = 0;
  virtual std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const = 0;
  // virtual bool equals(const IDescriptorKdtree& other) const = 0;
  virtual std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) = 0;
};
