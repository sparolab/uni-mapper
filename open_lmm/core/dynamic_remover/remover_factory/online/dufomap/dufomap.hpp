#pragma once

// STL
#include <boost/filesystem.hpp>

#include <iostream>
#include <thread>
// 3rdParty
#include <Eigen/Core>
#include <Eigen/Dense>
// UFO
#include <ufo/map/ufomap.hpp>
// PCL
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>

#include <math.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "params.hpp"

class DUFOMap : public IOnlineRemoverPlugin {
 public:
  DUFOMap(const DUFOMapParams& params);
  ~DUFOMap() = default;

  void initialize(const DUFOMapParams& params);

  void pclToUfo(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcd,
                ufo::PointCloudColor& cloud);
  void ufoToPcl(const ufo::PointCloudColor& ufo_cloud,
                pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud,
                bool is_dynamic);
  pcl::PointCloud<pcl::PointXYZI>::Ptr run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override;

 private:
  ufo::Map<ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
           ufo::MapType::LABEL>* ufo_map_;
  DUFOMapParams params_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_;
};