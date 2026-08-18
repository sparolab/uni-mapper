#pragma once

#include <open_lmm/common/result.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <string>
#include <string_view>

namespace open_lmm {

// KITTI text poses are commonly serialized with six digits after the decimal
// point. Quantizing a proper rotation at that precision can produce
// orthogonality and determinant residuals slightly above 1e-6. Validation
// deliberately accepts that representation error, but never normalizes the
// caller's transform.
inline constexpr double kRigidTransformRotationTolerance = 5e-6;
inline constexpr double kRigidTransformHomogeneousTolerance = 1e-9;

// This is the sole acceptance rule for transforms crossing an OpenLMM
// boundary. A transform must be finite, represent a proper SO(3) rotation,
// and retain the homogeneous row of a rigid 4x4 transform.
inline Result<void> ValidateRigidTransform(const Eigen::Isometry3d& transform,
                                           std::string_view context) {
  const Eigen::Matrix4d matrix = transform.matrix();
  if (!matrix.allFinite()) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": transform contains a non-finite value"));
  }
  if (!matrix.row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0),
                              kRigidTransformHomogeneousTolerance)) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": transform has an invalid homogeneous row"));
  }

  const Eigen::Matrix3d rotation = matrix.topLeftCorner<3, 3>();
  const double orthonormal_residual =
      (rotation.transpose() * rotation - Eigen::Matrix3d::Identity())
          .cwiseAbs()
          .maxCoeff();
  if (orthonormal_residual > kRigidTransformRotationTolerance ||
      std::abs(rotation.determinant() - 1.0) >
          kRigidTransformRotationTolerance) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) +
        ": transform rotation is not a proper orthonormal matrix"));
  }
  return Result<void>::Ok();
}

// Inverts an isometry without instantiating Eigen's general block-inverse
// expression. This requires the Isometry3d rotation to be orthonormal.
inline Eigen::Isometry3d InvertRigidTransform(
    const Eigen::Isometry3d& transform) {
  Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
  inverse.linear() = transform.linear().transpose();
  inverse.translation() = -(inverse.linear() * transform.translation());
  return inverse;
}

// Converts two scan poses expressed in the global frame into the transform
// expected by registration: target_scan_T_source_scan.
inline Eigen::Isometry3d TargetFromSourceScanTransform(
    const Eigen::Isometry3d& global_T_target_scan,
    const Eigen::Isometry3d& global_T_source_scan) {
  return InvertRigidTransform(global_T_target_scan) * global_T_source_scan;
}

}  // namespace open_lmm
