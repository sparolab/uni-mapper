#include <open_lmm/common/rigid_transform.hpp>

#include "property_generator.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iostream>

namespace {

using open_lmm::test::property::Fail;
using open_lmm::test::property::Generator;

Eigen::Isometry3d Transform(Generator& generator) {
  Eigen::Vector3d axis(generator.Between(-1.0, 1.0),
                       generator.Between(-1.0, 1.0),
                       generator.Between(-1.0, 1.0));
  if (axis.norm() < 1e-9) axis = Eigen::Vector3d::UnitX();
  axis.normalize();
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
      Eigen::AngleAxisd(generator.Between(-3.141592653589793,
                                         3.141592653589793),
                        axis)
          .toRotationMatrix();
  transform.translation() =
      Eigen::Vector3d(generator.Between(-1000.0, 1000.0),
                      generator.Between(-1000.0, 1000.0),
                      generator.Between(-1000.0, 1000.0));
  return transform;
}

}  // namespace

int main() {
  const uint64_t seed = open_lmm::test::property::Seed();
  const std::size_t cases = open_lmm::test::property::Cases(1000);
  Generator generator(seed);
  for (std::size_t index = 0; index < cases; ++index) {
    const Eigen::Isometry3d left = Transform(generator);
    const Eigen::Isometry3d right = Transform(generator);
    const Eigen::Isometry3d inverse = open_lmm::InvertRigidTransform(left);
    if (!open_lmm::ValidateRigidTransform(left, "property") ||
        !open_lmm::ValidateRigidTransform(inverse, "property inverse") ||
        !open_lmm::ValidateRigidTransform(left * right, "property compose")) {
      Fail("rigid-validity", seed, index, "generated rigid transform rejected");
    }
    if (!(left * inverse).matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-9)) {
      Fail("rigid-inverse", seed, index, "T*inverse(T) is not identity");
    }
    if (!open_lmm::InvertRigidTransform(inverse).matrix().isApprox(
            left.matrix(), 1e-9)) {
      Fail("rigid-double-inverse", seed, index,
           "inverse(inverse(T)) differs from T");
    }
    const auto relative = open_lmm::TargetFromSourceScanTransform(left, left);
    if (!relative.matrix().isApprox(Eigen::Matrix4d::Identity(), 1e-9)) {
      Fail("rigid-relative-identity", seed, index,
           "same target/source is not identity");
    }
  }
  std::cout << "rigid transform properties passed seed=" << seed
            << " cases=" << cases << '\n';
  return 0;
}
