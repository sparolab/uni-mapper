#include "pose_conversion.hpp"

Eigen::Isometry3d kittiPoseToIsometry3d(std::vector<double>& values) {
  Eigen::Map<Eigen::Matrix<double, 3, 4, Eigen::RowMajor>> transform(
      values.data());
  return Eigen::Isometry3d(transform);
}

Eigen::Isometry3d tumPoseToIsometry3d(std::vector<double>& values) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.translation() = Eigen::Vector3d(values[1], values[2], values[3]);
  transform.rotate(
      Eigen::Quaterniond(values[7], values[4], values[5], values[6]));
  return transform;
}

// TODO(gil) : add custom pose conversion
[[maybe_unused]] Eigen::Isometry3d customPoseToIsometry3d(
    std::vector<double>& values) {
  for (auto& value : values) {
    value = 0;
  }
  return Eigen::Isometry3d::Identity();
}
