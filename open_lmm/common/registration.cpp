#include "registration.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>

namespace open_lmm {
pcl::PointCloud<pcl::PointXYZI>::Ptr createSubmap(
    const std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>& scans_vec,
    const std::vector<Eigen::Isometry3d>& poses_vec, const int key,
    const int search_num) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr submap(
      new pcl::PointCloud<pcl::PointXYZI>);
  const int size = scans_vec.size();

  for (int i = -search_num; i <= search_num; ++i) {
    int key_near = key + i;
    if (key_near < 0 || key_near >= size) continue;
    pcl::PointCloud<pcl::PointXYZI>::Ptr key_near_scan(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*scans_vec[key_near], *key_near_scan,
                             poses_vec[key_near].matrix());
    *submap += *key_near_scan;
  }

  pcl::transformPointCloud(*submap, *submap, poses_vec[key].inverse().matrix());
  return submap;
}

small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> setupRegistration(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& source,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& target) {
  small_gicp::RegistrationPCL<pcl::PointXYZI, pcl::PointXYZI> reg;
  reg.setRegistrationType("GICP");  // "GICP" or "VGICP" (default = "GICP")
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
  if (reg.hasConverged() == false || reg.getFitnessScore() > 0.5) {
    return std::nullopt;
  }
  Eigen::Isometry3d T_to_rot;
  T_to_rot = reg.getFinalTransformation().cast<double>();
  return (T_to_rot * init_rel_pose).inverse();
}

std::optional<Eigen::Isometry3d> registerPointCloud(const SharedDatabase& db,
                                                    const LoopPair& loop_pair,
                                                    const int& search_num) {
  //! load scans and poses(to)
  const auto& scans_vec_to = db.db_scans.at(loop_pair.to.first);
  const auto& poses_vec_to = db.db_odom_poses.at(loop_pair.to.first);
  //! submap merging(to)
  auto submap_to =
      createSubmap(scans_vec_to, poses_vec_to, loop_pair.to.second, search_num);

  //! load scan(from)
  const auto& scan_from =
      db.db_scans.at(loop_pair.from.first)[loop_pair.from.second];
  //! transform scan(from) based on init_rel_pose
  pcl::PointCloud<pcl::PointXYZI>::Ptr scan_init_from(
      new pcl::PointCloud<pcl::PointXYZI>);
  pcl::transformPointCloud(*scan_from, *scan_init_from,
                           loop_pair.init_rel_pose.matrix());

  auto reg = setupRegistration(scan_init_from, submap_to);
  pcl::PointCloud<pcl::PointXYZI>::Ptr aligned(
      new pcl::PointCloud<pcl::PointXYZI>);
  reg.align(*aligned);

  return calculateFinalTransform(reg, loop_pair.init_rel_pose);
}

}  // namespace open_lmm
