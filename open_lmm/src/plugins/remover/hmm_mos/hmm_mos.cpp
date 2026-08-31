#include "hmm_mos.hpp"


HmmMos::HmmMos(const HmmMosParams& params)
    : params_(params), scan_(params_), map_(params_) {
  initialize();
}

HmmMos::~HmmMos() {}

void HmmMos::initialize() {
  scan_num_ = 0;
  scan_history_.set_capacity(params_.local_window_size);
  static_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  dynamic_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr HmmMos::run(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    Eigen::Isometry3d& optimized_pose) {
  scan_.readScanPCD(scan, optimized_pose);
  scan_.voxelizeScan();
  map_.update(scan_, scan_num_);

  if (scan_num_ > params_.local_window_size) {
    map_.findDynamicVoxels(scan_, scan_history_);
  } else {
    scan_history_.push_back(scan_);
  }

  auto static_scan = scan_.getGlobalStaticScan();
  *static_map_ += *static_scan;
  if (params_.replace_intensity) {
    *static_map_ += *scan_.getGlobalDynamicScan();
  }

  // TODO(gil) : save label

  scan_num_++;
  return static_scan;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr HmmMos::getStaticMap() {
  return static_map_;
}
