#include "free_dom.hpp"

FreeDom::FreeDom(const FreeDomParams& params) : params_(params) {
  initialize();
}

FreeDom::~FreeDom() {}

void FreeDom::initialize() {
  scan_num_ = 0;
  // scan_history_.set_capacity(params_.local_window_size);
  static_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  dynamic_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  setParams();
}

void FreeDom::setParams() {
  // set params
  ScanMap::ScanMapConfig scan_config{
      params_.sub_voxel_size,   params_.voxel_depth,
      params_.block_depth,      params_.enable_local_map,
      params_.local_map_range,  params_.local_map_min_z,
      params_.local_map_max_z,  params_.num_threads,
      params_.sensor_min_range, params_.sensor_max_range,
      params_.sensor_min_z,     params_.sensor_max_z};
  scan_.set_params(scan_config);

  MRMap::MRMapConfig map_config{params_.sub_voxel_size,
                                params_.voxel_depth,
                                params_.block_depth,
                                params_.enable_local_map,
                                params_.local_map_range,
                                params_.local_map_min_z,
                                params_.local_map_max_z,
                                params_.num_threads,
                                params_.sensor_max_range,
                                params_.sensor_min_z,
                                params_.sensor_max_z,
                                params_.raycast_max_range,
                                params_.raycast_min_z,
                                params_.raycast_max_z,
                                params_.counts_to_free,
                                params_.counts_to_revert,
                                params_.conservative_connectivity,
                                params_.aggressive_connectivity};
  map_.set_params(map_config);

  if (params_.enable_raycast_enhancement) {
    DepthImage::DepthImageConfig depth_image_config{
        params_.lidar_horizon_fov_rad,
        params_.lidar_vertical_fov_upper_rad,
        params_.lidar_vertical_fov_lower_rad,
        params_.depth_image_vertical_lines,
        params_.depth_image_min_range,
        params_.max_raycast_enhancement_range,
        params_.raycast_enhancement_depth_margin,
        params_.inpaint_size,
        params_.erosion_size,
        params_.min_raycast_enhancement_area,
        params_.depth_image_top_margin,
        params_.learn_fov,
        params_.enable_fov_mask,
        params_.fov_mask_path,
        params_.num_threads};
    depth_image_.set_params(depth_image_config);
  }
}

void FreeDom::run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
                  Eigen::Isometry3d& optimized_pose) {
  scan_.build_scan_map(*scan, optimized_pose);
  // TODO
  //  scan_removal_callback(scan);

  map_.scan_removal(scan_);
  if (params_.enable_raycast_enhancement) {
    depth_image_.raycast_enhancement(*scan, optimized_pose);
    // TODO
    //  raycast_enhancement_callback(depth_image);
  }
  Indices freespace_incremental;
  // FreeSpace estimation
  // timer_["raycast"].start();
  map_.freespace_estimation(scan_, depth_image_, freespace_incremental);
  // timer_["raycast"].stop();

  // map clearing
  // timer_["map removal"].start();
  map_.map_removal(freespace_incremental);
  // timer_["map removal"].stop();

  // map integration
  // timer_["staticspace integration"].start();
  map_.staticspace_integration(scan_, scan_num_);
  // timer_["staticspace integration"].stop();

  // timer_["remove map out of bound"].start();
  map_.remove_map_out_of_bound();
  // timer_["remove map out of bound"].stop();

  // TODO
  // 集成地图后立即触发回调
  // if (map_removal_callback) map_removal_callback(map);

  // timer_["reset"].start();
  scan_.reset();
  map_.reset();
  // timer_["reset"].stop();
  // ++scan_num_; // 아래에 있음

  // scan_.writeLabel("/home/gil/labels/", scan_num_);

  // *static_map_ += *scan_.getGlobalStaticScan();
  // if (params_.replace_intensity) {
  //   *static_map_ += *scan_.getGlobalDynamicScan();
  // }

  // TODO(gil) : save label

  scan_num_++;
  // return scan_.getGlobalStaticScan();
  // return scan;
}

void FreeDom::setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr& raw_map) {
  // // copy raw map to map_arranged
  // map_arranged_.reset(new pcl::PointCloud<pcl::PointXYZI>());
  // // TODO(gil) : use small gicp
  // //  VoxelPointCloud(raw_map, map_arranged_, cfg_.map_voxel_size_);
  // map_arranged_ =
  //     small_gicp::voxelgrid_sampling_tbb(*raw_map, cfg_.map_voxel_size_);
  // num_pcs_init_ = map_arranged_->points.size();
  // if (cfg_.is_large_scale_) {
  //   map_arranged_global_->reserve(NUM_PTS_LARGE_ENOUGH_FOR_MAP);
  //   *map_arranged_global_ = *map_arranged_;
  // }
  // std::cout << "tmp" << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr FreeDom::getStaticMap() {
  // pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_voxel(new
  // pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud_point(
      new pcl::PointCloud<pcl::PointXYZI>);

  unsigned int block_idx_size = map_.getVoxel2blockMultiples();
  unsigned int voxel_num = map_.getVoxel2blockMultiplesCubed();
  unsigned int sub_voxel_num = map_.getSubvoxel2voxelMultiplesCubed();

  double block_size = map_.getBlockSize();
  double voxel_size = map_.getVoxelSize();

  for (const auto& block_pair : map_.get_static_blocks()) {
    PointBias block_bias = block_size * block_pair.first.cast<double>();
    const StaticBlock& static_block = block_pair.second;

    Index local_voxel_idx(0, 0, 0);
    for (unsigned int i = 0; i < voxel_num; ++i) {
      if (!static_block.is_static_voxel_allocated(i)) {
        incrementIdx(local_voxel_idx, block_idx_size);
        continue;
      }

      const StaticVoxel& static_voxel = block_pair.second.getStaticVoxel(i);

      PointBias voxel_bias = voxel_size * local_voxel_idx.cast<double>();

      for (unsigned int j = 0; j < sub_voxel_num; ++j) {
        if (static_voxel.scan_in_subvoxel[j] == StaticVoxel::NOT_A_SCAN) {
          continue;
        }
        const bool is_dynamic =
            static_cast<uint8_t>(static_voxel.dynamic_level[j]) >=
            params_.dynamic_removal_threshold;
        Point point =
            block_bias + voxel_bias + static_voxel.points[j].cast<double>();

        pcl::PointXYZI output;
        output.x = static_cast<float>(point.x());
        output.y = static_cast<float>(point.y());
        output.z = static_cast<float>(point.z());
        output.intensity = is_dynamic ? 1.0F : 0.0F;
        if (is_dynamic && !params_.replace_intensity) {
          continue;
        }
        pointcloud_point->push_back(output);
      }
      incrementIdx(local_voxel_idx, block_idx_size);
    }
  }

  return pointcloud_point;
}
