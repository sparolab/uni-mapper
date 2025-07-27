
/**
 * Copyright (C) 2022-now, RPL, KTH Royal Institute of Technology
 * Only this file in under MIT License
 * Author: Kin ZHANG (https://kin-zhang.github.io/)
 * Date: 2023-04-05 11:19
 * Description: No ROS version, speed up the process
 *
 * Please reference official ERASOR paper for more details
 * This modified version is for the purpose of benchmarking no ROS! speed up! by
 * Kin, Reference the LICENSE file in origin repo.
 */

#pragma once
#include <math.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <mutex>
#include <thread>

#include "utils.hpp"

#define INF 10000000000000.0
#define PI 3.1415926535
#define ENOUGH_NUM 8000

#define EMPTY 0
#define MAP 1
#define PC_CURR 2
// COLORS:
// 0 -> BLUE
#define MAP_IS_HIGHER 0.5
#define CURR_IS_HIGHER 1.0
#define LITTLE_NUM 0.0  // For viz: blue - not activated
#define BLOCKED 0.8     // For viz

#define MERGE_BINS 0.25
#define NOT_ASSIGNED 0.0
// ground params

using namespace std;

struct Bin {
  double max_h;
  double min_h;
  double x;
  double y;
  double status;
  bool is_occupied;

  pcl::PointCloud<PointT> points;
};

struct DynamicBinIdx {
  int r;
  int theta;
};

typedef vector<vector<Bin> > R_POD;
typedef vector<Bin> Ring;

class ErasorCore {
 public:
  ErasorCore();
  virtual ~ErasorCore() = default;

  void setConfig(common::Config& cfg);

  void set_inputs(const pcl::PointCloud<PointT>& map_voi,
                  const pcl::PointCloud<PointT>& query_voi);
  void compare_vois_and_revert_ground_w_block();
  void get_static_estimate(pcl::PointCloud<PointT>& arranged,
                           pcl::PointCloud<PointT>& dynamic_pts,
                           pcl::PointCloud<PointT>& complement);

  void setCenter(double x, double y, double z);

 private:
  pcl::PointCloud<PointT> piecewise_ground_, non_ground_;
  pcl::PointCloud<PointT> ground_pc_, non_ground_pc_;
  void init(R_POD& r_pod);
  void extract_ground(pcl::PointCloud<PointT>& src,
                      pcl::PointCloud<PointT>& dst,
                      pcl::PointCloud<PointT>& outliers);
  bool is_dynamic_obj_close(R_POD& r_pod_selected, int r_target,
                            int theta_target);
  void extract_initial_seeds_(const pcl::PointCloud<PointT>& p_sorted,
                              pcl::PointCloud<PointT>& init_seeds);
  void estimate_plane_(const pcl::PointCloud<PointT>& ground);

  void clear_bin(Bin& bin);
  void clear(pcl::PointCloud<PointT>& pt_cloud);

  void pt2r_pod(const PointT& pt, Bin& bin);

  void voi2r_pod(const pcl::PointCloud<PointT>& src, R_POD& r_pod);

  void voi2r_pod(const pcl::PointCloud<PointT>& src, R_POD& r_pod,
                 pcl::PointCloud<PointT>& complement);

  double xy2theta(const double& x, const double& y);

  double xy2radius(const double& x, const double& y);
  void r_pod2pc(const R_POD& sc, pcl::PointCloud<PointT>& pc);

  common::Config cfg_;
  R_POD r_pod_map;       // R_POD of Map
  R_POD r_pod_curr;      // R_POD of current pointcloud
  R_POD r_pod_selected;  // R_POD of current pointcloud
  Eigen::MatrixXf normal_;
  double th_dist_d_, d_;
  double ring_size, sector_size;
  double center_x = 0, center_y = 0, center_z = 0;
  pcl::PointCloud<PointT> map_complement;
  pcl::PointCloud<PointT> dynamic_viz, ground_viz;  // Visualized in pcs_v2!

  std::mutex m_data;
};
