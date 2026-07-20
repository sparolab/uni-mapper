#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include <future>
#include <functional>

#include "utils.h"
#include "common_types.h"
#include "mrmap.h"
#include "scanmap.h"
#include "depth_image.h"

// TODO(gil) : remove open_lmm dependency
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>
#include <open_lmm/utils/config.hpp>

#include "params.hpp"

class FreeDom : public IOfflineRemoverPlugin {
 public:
  FreeDom(const FreeDomParams& params);
  // explicit FreeDom(Config config);
  ~FreeDom();

  void run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) override;

  void initialize();
  void setParams();
  void setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override;

 private:
  int scan_num_;

  FreeDomParams params_;

  // variables
  ScanMap scan_;
  DepthImage depth_image_;
  MRMap map_;
  // Timer timer_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_map_;
};
