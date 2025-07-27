#include "erasor.hpp"

#include <pcl/filters/voxel_grid.h>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>

// TODO(gil) : remove define
#define NUM_PTS_LARGE_ENOUGH 200000
#define NUM_PTS_LARGE_ENOUGH_FOR_MAP 20000000

ErasorServer::ErasorServer(const common::Config& params) : cfg_(params) {
  initialize();
  erasor_core_.setConfig(cfg_);
}

// ErasorServer::~ErasorServer() {}

void ErasorServer::initialize() {
  map_static_estimate_.reset(new pcl::PointCloud<PointT>());
  map_egocentric_complement_.reset(new pcl::PointCloud<PointT>());
  map_staticAdynamic.reset(new pcl::PointCloud<PointT>());
  map_filtered_.reset(new pcl::PointCloud<PointT>());
  map_arranged_global_.reset(new pcl::PointCloud<PointT>());
  map_arranged_complement_.reset(new pcl::PointCloud<PointT>());
}

// TODO(gil) : remove
void VoxelPointCloud(const pcl::PointCloud<PointT>::Ptr& cloud,
                     pcl::PointCloud<PointT>::Ptr& cloud_voxelized,
                     const double voxel_size) {
  if (voxel_size <= 0.001) {
    *cloud_voxelized = *cloud;
    return;
  }
  pcl::VoxelGrid<PointT> voxel_grid;
  voxel_grid.setInputCloud(cloud);
  voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);
  voxel_grid.filter(*cloud_voxelized);
}

void ErasorServer::setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) {
  // copy raw map to map_arranged
  map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  // TODO(gil) : use small gicp
  //  VoxelPointCloud(raw_map, map_arranged_, cfg_.map_voxel_size_);
  map_arranged_ =
      small_gicp::voxelgrid_sampling_tbb(*raw_map, cfg_.map_voxel_size_);
  num_pcs_init_ = map_arranged_->points.size();
  if (cfg_.is_large_scale_) {
    map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
    *map_arranged_global_ = *map_arranged_;
  }
}

void ErasorServer::run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
                       Eigen::Isometry3d& optimized_pose) {
  scan_num_++;
  if (scan_num_ % cfg_.removal_interval_ != 0) {
    return;
  }

  pcl::PointCloud<PointT>::Ptr filter_pc(new pcl::PointCloud<PointT>());
  // TODO(gil) : use small gicp
  //  VoxelPointCloud(scan, filter_pc, cfg_.query_voxel_size_);
  filter_pc = small_gicp::voxelgrid_sampling_tbb(*scan, cfg_.query_voxel_size_);
  // read pose in VIEWPOINT Field in pcd
  float x_curr = optimized_pose.translation().x();
  float y_curr = optimized_pose.translation().y();
  float z_curr = optimized_pose.translation().z();

  if (cfg_.is_large_scale_) {
    reassign_submap(x_curr, y_curr);
  }

  // 1. Fetch VoI
  fetch_VoI(
      x_curr, y_curr,
      *filter_pc);  // query_voi_ and map_voi_ are ready in the same world frame

  erasor_core_.setCenter(x_curr, y_curr, z_curr);
  erasor_core_.set_inputs(*map_voi_, *query_voi_);
  // 2. Compare VoI
  erasor_core_.compare_vois_and_revert_ground_w_block();
  // 3. Get StaticPts
  erasor_core_.get_static_estimate(*map_static_estimate_, *map_staticAdynamic,
                                   *map_egocentric_complement_);

  *map_arranged_ =
      *map_static_estimate_ + *map_outskirts_ + *map_egocentric_complement_;
}

void ErasorServer::fetch_VoI(double x_criterion, double y_criterion,
                             pcl::PointCloud<PointT>& query_pcd) {
  query_voi_.reset(new pcl::PointCloud<PointT>());
  map_voi_.reset(new pcl::PointCloud<PointT>());
  map_outskirts_.reset(new pcl::PointCloud<PointT>());

  if (cfg_.mode == "naive") {
    double max_dist_square = pow(cfg_.max_range_, 2);
    // find query voi
    for (auto const& pt : query_pcd.points) {
      double dist_square =
          pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
      if (dist_square < max_dist_square) {
        query_voi_->points.emplace_back(pt);
      }
    }

    // find map voi
    for (auto& pt : map_arranged_->points) {
      double dist_square =
          pow(pt.x - x_criterion, 2) + pow(pt.y - y_criterion, 2);
      if (dist_square < max_dist_square) {
        map_voi_->points.emplace_back(pt);
      } else {
        if (cfg_.replace_intensity) pt.intensity = 0;
        map_outskirts_->points.emplace_back(pt);
      }
    }
  }
}

void ErasorServer::reassign_submap(double pose_x, double pose_y) {
  if (is_submap_not_initialized_) {
    set_submap(*map_arranged_global_, *map_arranged_, *map_arranged_complement_,
               pose_x, pose_y, submap_size_);
    submap_center_x_ = pose_x;
    submap_center_y_ = pose_y;
    is_submap_not_initialized_ = false;

  } else {
    double diff_x = abs(submap_center_x_ - pose_x);
    double diff_y = abs(submap_center_y_ - pose_y);
    static double half_size = submap_size_ / 2.0;
    if ((diff_x > half_size) || (diff_y > half_size)) {
      // Reassign submap
      map_arranged_global_.reset(new pcl::PointCloud<pcl::PointXYZI>());
      map_arranged_global_->reserve(num_pcs_init_);
      *map_arranged_global_ = *map_arranged_ + *map_arranged_complement_;

      set_submap(*map_arranged_global_, *map_arranged_,
                 *map_arranged_complement_, pose_x, pose_y, submap_size_);
      submap_center_x_ = pose_x;
      submap_center_y_ = pose_y;
    }
  }
}
void ErasorServer::set_submap(
    const pcl::PointCloud<pcl::PointXYZI>& map_global,
    pcl::PointCloud<pcl::PointXYZI>& submap,
    pcl::PointCloud<pcl::PointXYZI>& submap_complement, double x, double y,
    double submap_size) {
  submap.clear();
  submap.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
  submap_complement.clear();
  submap_complement.reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);

  for (const auto pt : map_global.points) {
    double diff_x = fabs(x - pt.x);
    double diff_y = fabs(y - pt.y);
    if ((diff_x < submap_size) && (diff_y < submap_size)) {
      submap.points.emplace_back(pt);
    } else {
      submap_complement.points.emplace_back(pt);
    }
  }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr ErasorServer::getStaticMap() {
  pcl::PointCloud<PointT>::Ptr ptr_src(new pcl::PointCloud<PointT>);
  ptr_src->reserve(num_pcs_init_);

  if (cfg_.is_large_scale_) {
    *ptr_src = *map_arranged_ + *map_arranged_complement_;
  } else {
    *ptr_src = *map_arranged_;
  }
  // save map_static_estimate_
  // if (ptr_src->size() == 0) {
  //   return;
  // }
  if (map_staticAdynamic->size() > 0 && cfg_.replace_intensity) {
    *ptr_src += *map_staticAdynamic;
  }
  // pcl::io::savePCDFileBinary("/home/gil/erasor_output.pcd", *ptr_src);
  return ptr_src;
}