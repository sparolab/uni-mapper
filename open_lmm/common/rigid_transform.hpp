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

}  // namespace open_lmm
