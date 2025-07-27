/** @brief Class definition for the SOLiD descriptor.
 * SOLiD is a lidar scan descriptor introduced by [1] used for place
 * recognition.
 *
 * [1] H. Kim et al, "Narrowing your FOV with SOLiD: Spatially Organized and
 * Lightweight Global Descriptor for FOV-constrained LiDAR Place Recognition,"
 * 2024 IEEE Robotics and Automation Letters,
 *
 * @author Modified by Gilhwan Kang
 */

#pragma once
#include <Eigen/Dense>
#include <cmath>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp>
#include <open_lmm/utils/config.hpp>
#include <vector>

// Parameters for SOLiD descriptor
struct SolidParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  SolidParams();
  // explicit Params(Config config);
  ~SolidParams() = default;

 public:
  double fov_u = 2.0;
  double fov_d = -24.8;
  size_t num_angle = 60;
  size_t num_range = 40;
  size_t num_height = 32;
  size_t min_distance = 3;
  size_t max_distance = 80;
  double voxel_size = 0.4f;
  size_t descriptor_vector_dim{num_range};
  bool equals(const SolidParams& other) const;
};

// SOLiD Descriptor
class SOLiD : public IDescriptorKdtree {
 public:
 protected:
  SolidParams params_;
  Eigen::MatrixXd descriptor_;
  Eigen::VectorXd r_solid_key_;
  Eigen::VectorXd a_solid_key_;

 public:
  SOLiD(const SolidParams& params);

  std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) override;

  const Eigen::MatrixXd& getDescriptor() const override;
  const Eigen::VectorXd& getDescriptorKey() const override;
  const Eigen::VectorXd& getASolidKey() const;
  const SolidParams& getParams() const;
  const Eigen::VectorXd getASolidKeyFromDescriptor(
      const Eigen::MatrixXd& descriptor) const;

  std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const override;

  double loopDetection(const std::shared_ptr<IDescriptorKdtree>& other) const;
  Eigen::Isometry3d poseEstimation(
      const std::shared_ptr<IDescriptorKdtree>& other) const;

  double shiftedDistance(const size_t offset, const Eigen::VectorXd& query,
                         const Eigen::VectorXd& candidate) const;
};