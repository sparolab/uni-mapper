#pragma once

#include <Eigen/Core>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "erasor_core.hpp"
#include "utils.hpp"

// TODO(gil) : remove open_lmm dependency
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>

class ErasorServer : public IOfflineRemoverPlugin {
 public:
  ErasorServer(const common::Config& params);
  ~ErasorServer() = default;

  void run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
           Eigen::Isometry3d& optimized_pose) override;

  void initialize();
  void setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override;

 private:
  common::Config cfg_;
  ErasorCore erasor_core_;
  int scan_num_ = 0;

  //!
  pcl::PointCloud<PointT>::Ptr query_voi_;
  pcl::PointCloud<PointT>::Ptr map_voi_;
  pcl::PointCloud<PointT>::Ptr map_outskirts_;
  pcl::PointCloud<PointT>::Ptr map_arranged_;
  pcl::PointCloud<PointT>::Ptr map_arranged_global_, map_arranged_complement_;
  pcl::PointCloud<PointT>::Ptr map_static_estimate_;
  pcl::PointCloud<PointT>::Ptr map_staticAdynamic;
  pcl::PointCloud<PointT>::Ptr map_filtered_;
  pcl::PointCloud<PointT>::Ptr map_egocentric_complement_;

  /*** Outputs of ERASOR
   * map_filtered_ = map_static_estimate + map_egocentric_complement
   */
  void reassign_submap(double pose_x, double pose_y);
  void fetch_VoI(double x_criterion, double y_criterion,
                 pcl::PointCloud<PointT>& query_pcd);

  void set_submap(const pcl::PointCloud<pcl::PointXYZI>& map_global,
                  pcl::PointCloud<pcl::PointXYZI>& submap,
                  pcl::PointCloud<pcl::PointXYZI>& submap_complement, double x,
                  double y, double submap_size);

  double submap_size_ = 200.0;
  double submap_center_x_, submap_center_y_;
  int num_pcs_init_;
  bool is_submap_not_initialized_ = true;
};
