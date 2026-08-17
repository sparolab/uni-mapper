#pragma once

#include <Eigen/Geometry>

namespace open_lmm {

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
