#include "registration.hpp"

#include <foundation/diagnostics/profiling.hpp>
#include <domain/support/registration_log.hpp>
#include <domain/support/registration_pointcloud.hpp>
#include <open_lmm/common/rigid_transform.hpp>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace open_lmm {
small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> setupRegistration(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target) {
  OPEN_LMM_ZONE_N("Registration.SetupGICP");
  small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> reg;
  // The pinned small_gicp RegistrationPCL default is GICP. Avoid repeating
  // its string-based setter in this template-heavy translation unit.
  // reg.setVoxelResolution(1.0);
  reg.setNumThreads(16);
  reg.setMaxCorrespondenceDistance(150.0);
  reg.setInputSource(source);
  reg.setInputTarget(target);
  // gicp.setMaximumIterations(100);
  // gicp.setTransformationEpsilon(1e-6);
  // gicp.setEuclideanFitnessEpsilon(1e-6);
  // gicp.setRANSACIterations(0);
  return reg;
}

std::optional<Eigen::Isometry3d> calculateFinalTransform(
    small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI>& reg,
    const Eigen::Isometry3d& init_rel_pose) {
  OPEN_LMM_ZONE_N("Registration.FitnessCheck");
  if (!reg.hasConverged()) {
    LogRegistrationDidNotConverge();
    return std::nullopt;
  }
  const double fitness_score = reg.getFitnessScore();
  if (fitness_score > 0.5) {
    LogRegistrationFitnessRejected(fitness_score);
    return std::nullopt;
  }
  Eigen::Isometry3d T_to_rot;
  T_to_rot = reg.getFinalTransformation().cast<double>();
  const Eigen::Isometry3d final_pose = T_to_rot * init_rel_pose;
  return InvertRigidTransform(final_pose);
}

std::optional<Eigen::Isometry3d> registerPointCloud(
    const ScanVec& scans_to,
    const PoseVec& poses_to,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan_from,
    const LoopPair& loop_pair,
    int search_num) {
  OPEN_LMM_ZONE_N("Registration.Align");
  auto submap_to = CreateRegistrationSubmap(
      scans_to, poses_to, loop_pair.to.second, search_num);
  //! transform scan(from) based on init_rel_pose
  auto scan_init_from =
      TransformRegistrationScan(scan_from, loop_pair.init_rel_pose);

  auto reg = setupRegistration(scan_init_from, submap_to);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZI>);
  {
    OPEN_LMM_ZONE_N("Registration.GICPAlign");
    reg.align(*aligned);
  }

  return calculateFinalTransform(reg, loop_pair.init_rel_pose);
}

}  // namespace open_lmm
