#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <boost/circular_buffer.hpp>

#include "Map.hpp"
#include "Scan.hpp"
#include "utils.hpp"

// TODO(gil) : remove open_lmm dependency
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <open_lmm/utils/config.hpp>

#include "params.hpp"

class HmmMos : public IOnlineRemoverPlugin {
 public:
  HmmMos(const HmmMosParams& params);
  // explicit HmmMos(Config config);
  ~HmmMos();

  pcl::PointCloud<pcl::PointXYZI>::Ptr run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) override;

  void initialize();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override;

 private:
  int scan_num_;
  boost::circular_buffer<Scan> scan_history_;

  HmmMosParams params_;
  Scan scan_;
  Map map_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map_;
  pcl::PointCloud<pcl::PointXYZI>::Ptr dynamic_map_;
};
