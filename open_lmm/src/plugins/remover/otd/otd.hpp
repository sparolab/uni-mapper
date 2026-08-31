#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
// #include <boost/circular_buffer.hpp>

#include "common_lib.h"
#include "ground_segmentation.h"
#include "otd_core.h"

// TODO(gil) : remove open_lmm dependency
#include <domain/dynamic_removal/plugin/online_plugin.hpp>
#include <config/document/config.hpp>

#include "params.hpp"

class OTD : public IOnlineRemoverPlugin {
 public:
  OTD(const OTDParams& params);
  ~OTD();

  pcl::PointCloud<pcl::PointXYZI>::Ptr run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) override;

  void initialize();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override;

 private:
  int scan_num_;

  OTDParams params_;

  std::shared_ptr<otd::Grd_Seg<PointType>> p_grdseg_;
  std::shared_ptr<otd::Otd3D<PointType>> p_otd_; 
  // auto p_otd_ = std::make_shared<Otd3D<PointType>>(tau_ratio, voxel_size);

  PointCloudType::Ptr ground_scan_;
  PointCloudType::Ptr nonground_scan_;
};
