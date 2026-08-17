#ifndef OTD_H
#define OTD_H

#include <omp.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <iostream>
#include <list>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace otd {

using namespace std;

struct M_POINT {
  float xyz[3];
  int count = 0;
};

template <typename PointT>
class OtdNode {
 private:
  std::vector<PointT> ground_points_;
  std::vector<PointT> nonground_points_;
  std::vector<PointT> dynamic_points_;

  std::set<int> ground_times_;
  std::set<int> nonground_times_;
  double max_height_ = 0.;
  double sum_height_ = 0.;
  double mean_height_ = 0.;
  double z_num_ = 0;

 public:
  OtdNode() {}
  ~OtdNode() {}

  void InsertGroundPoints(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
                          const int frame_num);
  void InsertGroundPoints(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
                          double mean_height, const int frame_num);

  void InsertNongroundPoints(
      typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const int frame_num);

  void GetNongroundPoints(typename pcl::PointCloud<PointT>::Ptr cloud_ptr);
  void GetGroundPoints(typename pcl::PointCloud<PointT>::Ptr cloud_ptr);
  void GetDynamicPoints(typename pcl::PointCloud<PointT>::Ptr cloud_ptr);

  std::set<int> GetGroundTimes();
  std::set<int> GetNongroundTimes();

  void RemovePoints();
  void RevertPoints();

  void OTD(const std::set<int>& ground_times, const double& tau_ob_ratio);
};

template <typename PointT>
void OtdNode<PointT>::InsertGroundPoints(
    typename pcl::PointCloud<PointT>::Ptr ground_ptr, const int frame_num) {
  ground_points_.insert(ground_points_.end(), ground_ptr->points.begin(),
                        ground_ptr->points.end());
  ground_times_.insert(frame_num);
}

template <typename PointT>
void OtdNode<PointT>::InsertGroundPoints(
    typename pcl::PointCloud<PointT>::Ptr ground_ptr, double mean_height,
    const int frame_num) {
  ground_points_.insert(ground_points_.end(), ground_ptr->points.begin(),
                        ground_ptr->points.end());

  // if(abs(mean_height - mean_height_) > 1.) {
  //     ground_times_.clear();
  //     mean_height_ = mean_height;
  // } else {
  //     mean_height_ = mean_height;
  // }
  ground_times_.insert(frame_num);
}

template <typename PointT>
void OtdNode<PointT>::InsertNongroundPoints(
    typename pcl::PointCloud<PointT>::Ptr nonground_ptr, const int frame_num) {
  nonground_points_.insert(nonground_points_.end(),
                           nonground_ptr->points.begin(),
                           nonground_ptr->points.end());
  nonground_times_.insert(frame_num);
}

template <typename PointT>
void OtdNode<PointT>::GetNongroundPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud_ptr) {
  cloud_ptr->points.insert(cloud_ptr->points.end(), nonground_points_.begin(),
                           nonground_points_.end());
}

template <typename PointT>
void OtdNode<PointT>::GetGroundPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud_ptr) {
  cloud_ptr->points.insert(cloud_ptr->points.end(), ground_points_.begin(),
                           ground_points_.end());
}

template <typename PointT>
void OtdNode<PointT>::GetDynamicPoints(
    typename pcl::PointCloud<PointT>::Ptr cloud_ptr) {
  cloud_ptr->points.insert(cloud_ptr->points.end(), dynamic_points_.begin(),
                           dynamic_points_.end());
}

template <typename PointT>
std::set<int> OtdNode<PointT>::GetGroundTimes() {
  return ground_times_;
}

template <typename PointT>
std::set<int> OtdNode<PointT>::GetNongroundTimes() {
  return nonground_times_;
}

template <typename PointT>
void OtdNode<PointT>::RemovePoints() {
  dynamic_points_.insert(dynamic_points_.end(), nonground_points_.begin(),
                         nonground_points_.end());
  std::vector<PointT>().swap(nonground_points_);
}

template <typename PointT>
void OtdNode<PointT>::RevertPoints() {
  nonground_points_.insert(nonground_points_.end(), dynamic_points_.begin(),
                           dynamic_points_.end());
  std::vector<PointT>().swap(dynamic_points_);
}

template <typename PointT>
void OtdNode<PointT>::OTD(const std::set<int>& ground_times,
                          const double& tau_ob_ratio) {
  if (!ground_times.empty() && !nonground_times_.empty()) {
    int total_nogrd_ob_time = nonground_times_.size();
    int total_grd_ob_time = ground_times.size();

    if (total_nogrd_ob_time < tau_ob_ratio * total_grd_ob_time
        // && nogrd_ob_range < tau_ob_ratio * grd_ob_range
    ) {
      RemovePoints();
    } else {
      RevertPoints();
    }
  }
}

//////////////////////////////////////////////////////
//////////////////////////////////////////////////////
struct hash_2d {
  inline size_t operator()(const Eigen::Vector2i& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943)) % 10000000;
  }
};

struct hash_3d {
  inline size_t operator()(const Eigen::Vector3i& v) const {
    return size_t(((v[0]) * 73856093) ^ ((v[1]) * 471943) ^
                  ((v[2]) * 83492791)) %
           10000000;
  }
};

template <typename PointT>
class Otd3D {
 public:
  using KeyType2D = Eigen::Vector2i;
  using KeyType3D = Eigen::Vector3i;

 private:
  std::unordered_map<
      KeyType3D,
      typename std::list<std::pair<KeyType3D, OtdNode<PointT>>>::iterator,
      hash_3d>
      nonground_map_;
  std::list<std::pair<KeyType3D, OtdNode<PointT>>> nonground_cache_;

  std::unordered_map<
      KeyType2D,
      typename std::list<std::pair<KeyType2D, OtdNode<PointT>>>::iterator,
      hash_2d>
      ground_map_;
  std::list<std::pair<KeyType2D, OtdNode<PointT>>> ground_cache_;

  std::unordered_map<KeyType2D, int, hash_2d> minz_map_;

  double resolution_, inv_resolution_;

  double tau_ob_ratio_;

  std::unordered_map<int, long long> timestamps_;
  std::unordered_map<int, Eigen::Matrix4d> poses_;
  double t_otd_sum_;
  int frame_sum_;
  std::vector<KeyType3D> nearby_grids_;

 public:
  typename pcl::PointCloud<PointT>::Ptr ground_tosave_;
  typename pcl::PointCloud<PointT>::Ptr nonground_tosave_;
  typename pcl::PointCloud<PointT>::Ptr dynamic_tosave_;

  Otd3D(const double tau_ob_ratio, const double resolution);
  ~Otd3D() {}
  void Run(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
           typename pcl::PointCloud<PointT>::Ptr nonground_ptr,
           const int frame_num);
  void PruningVoxel(long long map_capacity);

  void SaveMap(std::string savepath);
  pcl::PointCloud<PointT>::Ptr GetMap(bool replace_intensity);
  void Downsamplemap(typename pcl::PointCloud<PointT>::Ptr ptr_todown,
                     double voxel_size);

  // bool isDynLabel(PointT pt, std::vector<int> dynamic_label);
  bool isVox(PointT pt_ori, PointT pt_cess);
  // void ComputeRateByKDTree(std::vector<int> dynamic_label);

  void GetTimeConsume();

  Eigen::Vector2i Pos2Grid2D(const PointT pt);
  Eigen::Vector3i Pos2Grid3D(const PointT pt);

  void GetTimeFromPoint(PointT pt, std::string savefile);
};

template <typename PointT>
Otd3D<PointT>::Otd3D(const double tau_ob_ratio, const double resolution)
    : resolution_(resolution),
      inv_resolution_(1. / resolution),
      tau_ob_ratio_(tau_ob_ratio),
      t_otd_sum_(0.),
      frame_sum_(0) {
  printf("The tau_ob_ratio is %lf\n", tau_ob_ratio_);
  printf("The resolution and inv_resolution is %lf and %lf\n", resolution_,
         inv_resolution_);

  ground_tosave_.reset(new pcl::PointCloud<PointT>);
  nonground_tosave_.reset(new pcl::PointCloud<PointT>);
  dynamic_tosave_.reset(new pcl::PointCloud<PointT>);

  nearby_grids_ = {KeyType3D(0, 0, 0), KeyType3D(-1, 0, 0), KeyType3D(1, 0, 0),
                   KeyType3D(0, 1, 0), KeyType3D(0, -1, 0)};
}

template <typename PointT>
void Otd3D<PointT>::Run(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
                        typename pcl::PointCloud<PointT>::Ptr nonground_ptr,
                        const int frame_num) {
  std::unordered_map<KeyType2D, typename pcl::PointCloud<PointT>::Ptr, hash_2d>
      ground_grid_curr;
  std::unordered_map<KeyType2D, int, hash_2d> ground_minz_curr;
  std::unordered_set<KeyType2D, hash_2d> groundkey_set;
  std::vector<KeyType2D> groundkey_vec;

  std::unordered_map<KeyType3D, typename pcl::PointCloud<PointT>::Ptr, hash_3d>
      nonground_grid_curr;
  std::unordered_set<KeyType3D, hash_3d> nongroundkey_set;
  std::vector<KeyType3D> nongroundkey_vec;

  // printf("Start to transform pc\n");
  double t_start = omp_get_wtime();

  // printf("Process for ground points set\n");
  /////////////////////////////////////////////////////////////////////////////
  // Process for ground points set
  // Compute grid pos
  for (size_t i = 0; i < ground_ptr->size(); i++) {
    PointT pt = ground_ptr->points[i];
    auto key3d = Pos2Grid3D(pt);

    KeyType2D key(key3d[0], key3d[1]);
    groundkey_set.insert(key);

    auto iter = ground_grid_curr.find(key);
    if (iter != ground_grid_curr.end()) {
      iter->second->points.push_back(pt);
      ground_minz_curr[key] = min(key3d[2], ground_minz_curr[key]);
    } else {
      typename pcl::PointCloud<PointT>::Ptr cloudptr(
          new pcl::PointCloud<PointT>);
      cloudptr->points.push_back(pt);
      ground_grid_curr.insert({key, cloudptr});
      ground_minz_curr.insert({key, key3d[2]});
    }
  }

  // printf("Create new Node\n");
  // Create new Node (prepare for parallel)
  for (auto key : groundkey_set) {
    if (ground_map_.find(key) == ground_map_.end()) {
      ground_cache_.push_front({key, OtdNode<PointT>()});
      ground_map_.insert({key, ground_cache_.begin()});
    }
    groundkey_vec.push_back(key);
  }

// printf("Insert points to Node\n");
// Insert points to Node
#pragma omp parallel for
  for (int i = 0; i < groundkey_vec.size(); i++) {
    auto key = groundkey_vec[i];
    ground_map_[key]->second.InsertGroundPoints(
        ground_grid_curr[key], float(ground_minz_curr[key]) * resolution_,
        frame_num);
  }

  //////////////////////////////////////////////////////////////////////////////
  // printf("Process for nonground points set\n");
  // Process for nonground points set
  // Compute grid pos
  for (size_t i = 0; i < nonground_ptr->size(); i++) {
    PointT pt = nonground_ptr->points[i];
    auto key = Pos2Grid3D(pt);
    nongroundkey_set.insert(key);

    auto iter = nonground_grid_curr.find(key);
    if (iter != nonground_grid_curr.end())
      iter->second->points.push_back(pt);
    else {
      typename pcl::PointCloud<PointT>::Ptr cloudptr(
          new pcl::PointCloud<PointT>);
      cloudptr->points.push_back(pt);
      nonground_grid_curr.insert({key, cloudptr});
    }
  }
  // printf("The nonground voxel num is %d\n", nonground_grid_curr.size());

  // printf("Create new Node\n");
  // Create new Node (prepare for parallel)
  for (auto key : nongroundkey_set) {
    if (nonground_map_.find(key) == nonground_map_.end()) {
      nonground_cache_.push_front({key, OtdNode<PointT>()});
      nonground_map_.insert({key, nonground_cache_.begin()});
    }

    nongroundkey_vec.push_back(key);
  }

// printf("Insert points and DownRetrival\n");
// Insert points to Node
#pragma omp parallel for
  for (int i = 0; i < nongroundkey_vec.size(); i++) {
    auto key = nongroundkey_vec[i];
    auto iter = nonground_map_.find(key);
    iter->second->second.InsertNongroundPoints(nonground_grid_curr[key],
                                               frame_num);

    std::set<int> ground_times;
    for (auto nearbykey : nearby_grids_) {
      KeyType2D groundkey(key[0] + nearbykey[0], key[1] + nearbykey[1]);
      // KeyType2D groundkey(key[0], key[1]);

      auto it_ground = ground_map_.find(groundkey);
      if (it_ground != ground_map_.end()) {
        ground_times = it_ground->second->second.GetGroundTimes();
      }

      if (ground_times.size() > 3) break;
    }

    nonground_map_[key]->second.OTD(ground_times, tau_ob_ratio_);
  }

// printf("UpRetrival\n");
#pragma omp parallel for
  for (int i = 0; i < groundkey_vec.size(); i++) {
    auto key = groundkey_vec[i];
    std::set<int> ground_times = ground_map_[key]->second.GetGroundTimes();

    // for(auto nearbykey : nearby_grids_)
    {
      // KeyType3D key_ground(key[0] + nearbykey[0], key[1] + nearbykey[1],
      // ground_minz_curr[key]);
      KeyType3D key_ground(key[0], key[1], ground_minz_curr[key]);
      for (size_t i = 0; i < 3. * inv_resolution_; i++) {
        KeyType3D key_nonground = key_ground + KeyType3D(0, 0, i);
        auto it_nonground = nonground_map_.find(key_nonground);

        if (it_nonground != nonground_map_.end())
          it_nonground->second->second.OTD(ground_times, tau_ob_ratio_);
      }
    }
  }

  double t_otd = omp_get_wtime();

  t_otd_sum_ += t_otd - t_start;
  frame_sum_++;
}

template <typename PointT>
void Otd3D<PointT>::PruningVoxel(long long map_capacity) {
  while (nonground_map_.size() > map_capacity) {
    nonground_cache_.back().second.GetNongroundPoints(nonground_tosave_);
    nonground_cache_.back().second.GetDynamicPoints(dynamic_tosave_);

    nonground_map_.erase(nonground_cache_.back().first);
    nonground_cache_.pop_back();
  }

  Downsamplemap(nonground_tosave_, 0.1);
  Downsamplemap(dynamic_tosave_, 0.1);
}

template <typename PointT>
void Otd3D<PointT>::SaveMap(std::string savepath) {
  for (auto iter = ground_map_.begin(); iter != ground_map_.end(); iter++) {
    iter->second->second.GetGroundPoints(ground_tosave_);
  }
  for (auto iter = nonground_map_.begin(); iter != nonground_map_.end();
       iter++) {
    iter->second->second.GetNongroundPoints(nonground_tosave_);
    iter->second->second.GetDynamicPoints(dynamic_tosave_);
  }
  Downsamplemap(ground_tosave_, 0.1);
  Downsamplemap(nonground_tosave_, 0.1);
  Downsamplemap(dynamic_tosave_, 0.1);

  printf("The ground nonground and dynamic map size is %ld, %ld and %ld\n",
         ground_tosave_->size(), nonground_tosave_->size(),
         dynamic_tosave_->size());

  pcl::io::savePCDFileBinaryCompressed(savepath + "/ground_map.pcd",
                                       *ground_tosave_);
  pcl::io::savePCDFileBinaryCompressed(savepath + "/nonground_map.pcd",
                                       *nonground_tosave_);
  pcl::io::savePCDFileBinaryCompressed(savepath + "/dynamic_map.pcd",
                                       *dynamic_tosave_);
}

// pcl::PointCloud<PointT>::Ptr GetMap();

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr Otd3D<PointT>::GetMap(
    bool replace_intensity) {
  auto ground_map = std::make_shared<pcl::PointCloud<PointT>>();
  auto nonground_map = std::make_shared<pcl::PointCloud<PointT>>();
  auto dynamic_map = std::make_shared<pcl::PointCloud<PointT>>();
  for (auto iter = ground_map_.begin(); iter != ground_map_.end(); iter++) {
    iter->second->second.GetGroundPoints(ground_map);
  }
  for (auto iter = nonground_map_.begin(); iter != nonground_map_.end();
       iter++) {
    iter->second->second.GetNongroundPoints(nonground_map);
    iter->second->second.GetDynamicPoints(dynamic_map);
  }

  auto global_static_map = std::make_shared<pcl::PointCloud<PointT>>();
  for (auto& point : *ground_map) {
    point.intensity = 0.0F;
  }
  for (auto& point : *nonground_map) {
    point.intensity = 0.0F;
  }
  *global_static_map += *ground_map;
  *global_static_map += *nonground_map;
  if (replace_intensity) {
    for (auto& point : *dynamic_map) {
      point.intensity = 1.0F;
    }
    *global_static_map += *dynamic_map;
  }

  return global_static_map;

  // Downsamplemap(ground_map, 0.1);
  // Downsamplemap(nonground_map, 0.1);
  // Downsamplemap(dynamic_map, 0.1);

  // printf("The ground nonground and dynamic map size is %ld, %ld and %ld\n",
  //        ground_map->size(), nonground_map->size(), dynamic_map->size());
}

template <typename PointT>
void Otd3D<PointT>::Downsamplemap(
    typename pcl::PointCloud<PointT>::Ptr ptr_todown, double voxel_size) {
  if (ptr_todown->empty()) return;
  double voxel_size_inv = 1. / voxel_size;

  std::unordered_map<Eigen::Vector3i, M_POINT, hash_3d> feature_map;

  feature_map.reserve(ptr_todown->size());
  Eigen::Vector3i voxel_index;

  for (int i = 0; i < (int)ptr_todown->size(); ++i) {
    const auto& pt = ptr_todown->points[i];
    voxel_index = (Eigen::Vector3d(pt.x, pt.y, pt.z) * voxel_size_inv)
                      .array()
                      .round()
                      .template cast<int>();

    auto iter = feature_map.find(voxel_index);
    if (iter != feature_map.end()) {
      iter->second.xyz[0] += pt.x;
      iter->second.xyz[1] += pt.y;
      iter->second.xyz[2] += pt.z;
      iter->second.count++;
    } else {
      M_POINT anp;
      anp.xyz[0] = pt.x;
      anp.xyz[1] = pt.y;
      anp.xyz[2] = pt.z;
      anp.count = 1;
      feature_map[voxel_index] = anp;
    }
  }

  pcl::PointCloud<PointT> feature_cloud;
  feature_cloud.reserve(feature_map.size());

  for (auto iter = feature_map.begin(); iter != feature_map.end(); iter++) {
    PointT pt;
    pt.x = iter->second.xyz[0] / iter->second.count;
    pt.y = iter->second.xyz[1] / iter->second.count;
    pt.z = iter->second.xyz[2] / iter->second.count;
    feature_cloud.push_back(pt);
  }

  *ptr_todown = feature_cloud;
}

// template <typename PointT>
// bool Otd3D<PointT>::isDynLabel(PointT pt, std::vector<int> dynamic_label) {
//   int label = int(pt.label);

//   for (size_t dyna_i = 0; dyna_i < dynamic_label.size(); dyna_i++) {
//     if (label == dynamic_label[dyna_i]) return true;
//   }
//   return false;
// }

template <typename PointT>
bool Otd3D<PointT>::isVox(PointT pt_ori, PointT pt_cess) {
  Eigen::Vector3d vec_ori(pt_ori.x, pt_ori.y, pt_ori.z);
  Eigen::Vector3d vec_cess(pt_cess.x, pt_cess.y, pt_cess.z);

  Eigen::Vector3i vec_res = (vec_ori * 5).array().round().cast<int>() -
                            (vec_cess * 5).array().round().cast<int>();
  return vec_res.norm() < 0.1;
}

// template <typename PointT>
// void Otd3D<PointT>::ComputeRateByKDTree(std::vector<int> dynamic_label) {
//   typename pcl::PointCloud<PointT>::Ptr static_ptr(new
//   pcl::PointCloud<PointT>); typename pcl::PointCloud<PointT>::Ptr all_ptr(new
//   pcl::PointCloud<PointT>); printf("Computing PR and RR...\n");

//   for (auto iter = ground_map_.begin(); iter != ground_map_.end(); iter++) {
//     iter->second->second.GetGroundPoints(static_ptr);
//   }
//   for (auto iter = nonground_map_.begin(); iter != nonground_map_.end();
//        iter++) {
//     iter->second->second.GetNongroundPoints(static_ptr);
//     iter->second->second.GetDynamicPoints(all_ptr);
//   }

//   *all_ptr += *static_ptr;

//   pcl::KdTreeFLANN<PointT> kdtree;
//   kdtree.setInputCloud(static_ptr);
//   int sta_num_in_origin = 0, dyn_num_in_origin = 0;
//   int sta_num_preserved = 0, dyn_num_preserved = 0;

//   // #pragma omp parallel for
//   for (size_t i = 0; i < all_ptr->size(); i++) {
//     PointT pt_ori = all_ptr->points[i];
//     std::vector<int> pointIdxNKNSearch(1);
//     std::vector<float> pointNKNSquaredDistance(1);
//     if (kdtree.nearestKSearch(pt_ori, 1, pointIdxNKNSearch,
//                               pointNKNSquaredDistance) > 0) {
//       PointT pt_cess = static_ptr->points[pointIdxNKNSearch[0]];
//       if (isDynLabel(pt_ori, dynamic_label)) {
//         dyn_num_in_origin++;
//         if (isDynLabel(pt_cess, dynamic_label) && isVox(pt_ori, pt_cess)) {
//           dyn_num_preserved++;
//         }
//       } else {
//         sta_num_in_origin++;
//         if (!isDynLabel(pt_cess, dynamic_label) && isVox(pt_ori, pt_cess))
//           sta_num_preserved++;
//       }
//     } else {
//       if (isDynLabel(pt_ori, dynamic_label))
//         dyn_num_in_origin++;
//       else
//         sta_num_in_origin++;
//     }
//   }
//   printf("The num of sta and dyn in origin map is %d, %d\n",
//   sta_num_in_origin,
//          dyn_num_in_origin);
//   printf("The num of sta and dyn in static map is %d, %d\n",
//   sta_num_preserved,
//          dyn_num_preserved);

//   float PR = float(float(sta_num_preserved) / float(sta_num_in_origin));
//   float RR = float(1 - float(dyn_num_preserved) / float(dyn_num_in_origin));
//   printf("The PR is %f\n", PR * 100);
//   printf("The RR is %f\n", RR * 100);
//   printf("The F1 score is %f\n", (2 * PR * RR) / (PR + RR));
// }

template <typename PointT>
void Otd3D<PointT>::GetTimeConsume() {
  printf("The average otd time is %lf\n", (t_otd_sum_ / frame_sum_) * 1000);
}

template <typename PointT>
Eigen::Vector2i Otd3D<PointT>::Pos2Grid2D(const PointT pt) {
  return (Eigen::Vector2d(pt.x, pt.y) * inv_resolution_)
      .array()
      .round()
      .template cast<int>();
}

template <typename PointT>
Eigen::Vector3i Otd3D<PointT>::Pos2Grid3D(const PointT pt) {
  return (Eigen::Vector3d(pt.x, pt.y, pt.z) * inv_resolution_)
      .array()
      .round()
      .template cast<int>();
}

template <typename PointT>
void Otd3D<PointT>::GetTimeFromPoint(PointT pt, std::string savefile) {
  std::set<int> ground_times, nonground_times;

  std::ofstream file_stream(savefile, std::ios::app);

  auto key = Pos2Grid3D(pt);
  file_stream << key[0] << " " << key[1] << " " << key[2] << ",";

  auto iter = nonground_map_.find(key);
  if (iter != nonground_map_.end()) {
    nonground_times = iter->second->second.GetNongroundTimes();
  }

  for (auto i : nonground_times) {
    file_stream << " " << i;
  }
  file_stream << ",";

  KeyType2D groundkey(key[0], key[1]);
  auto it_ground = ground_map_.find(groundkey);
  if (it_ground != ground_map_.end()) {
    ground_times = it_ground->second->second.GetGroundTimes();
  }

  for (auto i : ground_times) {
    file_stream << " " << i;
  }
  file_stream << std::endl;
  file_stream.close();
}

///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
template <typename PointT>
class Otd2D {
 public:
  using KeyType = Eigen::Vector2i;

 private:
  std::unordered_map<
      KeyType,
      typename std::list<std::pair<KeyType, OtdNode<PointT>>>::iterator,
      hash_2d>
      grids_map_;  // voxel hash map
  std::list<std::pair<KeyType, OtdNode<PointT>>> grids_cache_;  // voxel cache
  double resolution_, inv_resolution_ = 2;

  double tau_ob_ratio_;

 public:
  Otd2D(const double tau_ob_ratio, const double resolution);
  ~Otd2D() {}
  void Run(typename pcl::PointCloud<PointT>::Ptr ground_pc,
           typename pcl::PointCloud<PointT>::Ptr nonground_ptr,
           const int frame_num);
  void GetMap(typename pcl::PointCloud<PointT>::Ptr ground_map,
              typename pcl::PointCloud<PointT>::Ptr nonground_map,
              typename pcl::PointCloud<PointT>::Ptr dynamic_map);
  Eigen::Vector2i Pos2Grid(const PointT pt);

  void GroundSegByPCA(typename pcl::PointCloud<PointT>::Ptr body_ptr,
                      typename pcl::PointCloud<PointT>::Ptr ground_ptr,
                      typename pcl::PointCloud<PointT>::Ptr nonground_ptr);
};

template <typename PointT>
Otd2D<PointT>::Otd2D(const double tau_ob_ratio, const double resolution)
    : resolution_(resolution),
      inv_resolution_(1. / resolution),
      tau_ob_ratio_(tau_ob_ratio) {
  printf("The tau_ob_ratio is %lf\n", tau_ob_ratio_);
  printf("The resolution and inv_resolution is %lf and %lf\n", resolution_,
         inv_resolution_);
}

template <typename PointT>
void Otd2D<PointT>::Run(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
                        typename pcl::PointCloud<PointT>::Ptr nonground_ptr,
                        const int frame_num) {
  std::unordered_map<KeyType, typename pcl::PointCloud<PointT>::Ptr, hash_2d>
      ground_grid_curr;
  std::unordered_map<KeyType, typename pcl::PointCloud<PointT>::Ptr, hash_2d>
      nonground_grid_curr;
  std::unordered_map<KeyType, float, hash_2d> minz_grid_curr;
  std::unordered_map<KeyType, float, hash_2d> maxz_grid_curr;
  std::unordered_set<KeyType, hash_2d> key_set;
  std::vector<KeyType> key_vec;
  double t_start = omp_get_wtime();
  for (size_t i = 0; i < ground_ptr->size(); i++) {
    PointT pt = ground_ptr->points[i];
    auto key = Pos2Grid(pt);
    key_set.insert(key);

    auto iter = ground_grid_curr.find(key);
    if (iter != ground_grid_curr.end())
      iter->second->points.push_back(pt);
    else {
      typename pcl::PointCloud<PointT>::Ptr cloudptr(
          new pcl::PointCloud<PointT>);
      cloudptr->points.push_back(pt);
      ground_grid_curr.insert({key, cloudptr});
    }
  }

  for (size_t i = 0; i < nonground_ptr->size(); i++) {
    PointT pt = nonground_ptr->points[i];
    auto key = Pos2Grid(pt);
    key_set.insert(key);

    auto iter = nonground_grid_curr.find(key);
    if (iter != nonground_grid_curr.end()) {
      iter->second->points.push_back(pt);
      minz_grid_curr[key] = min(pt.z, minz_grid_curr[key]);
      maxz_grid_curr[key] = max(pt.z, maxz_grid_curr[key]);
    } else {
      typename pcl::PointCloud<PointT>::Ptr cloudptr(
          new pcl::PointCloud<PointT>);
      cloudptr->points.push_back(pt);
      nonground_grid_curr.insert({key, cloudptr});
      maxz_grid_curr.insert({key, pt.z});
      minz_grid_curr.insert({key, pt.z});
    }
  }
  double t_calgrid = omp_get_wtime();
  // printf("The ground and nonground grid num is %d, %d\n",
  // ground_grid_curr.size(), nonground_grid_curr.size());

  for (auto key : key_set) {
    if (grids_map_.find(key) == grids_map_.end()) {
      grids_cache_.push_front({key, OtdNode<PointT>()});
      grids_map_.insert({key, grids_cache_.begin()});
    }
    key_vec.push_back(key);
  }
  double t_createnode = omp_get_wtime();

#pragma omp parallel for
  for (int i = 0; i < key_vec.size(); i++) {
    auto key = key_vec[i];
    typename pcl::PointCloud<PointT>::Ptr groundptr(
        new pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr nongroundptr(
        new pcl::PointCloud<PointT>);
    double z_diff = 0.;

    if (nonground_grid_curr.find(key) != nonground_grid_curr.end()) {
      *nongroundptr = *nonground_grid_curr[key];
      z_diff = maxz_grid_curr[key] - minz_grid_curr[key];
    }

    if (ground_grid_curr.find(key) != ground_grid_curr.end()) {
      *groundptr = *ground_grid_curr[key];
    } else {
      z_diff = 0.;
    }

    // grids_map_[key]->second.InsertPoints_OTD(groundptr, nongroundptr,
    // frame_num, z_diff, tau_ob_ratio_);
  }
  double t_otd = omp_get_wtime();

  printf("The calculate grid pos time is %lfms\n",
         (t_calgrid - t_start) * 1000);
  printf("The create new node time is %lfms\n",
         (t_createnode - t_calgrid) * 1000);
  printf("The otd time is %lfms\n", (t_otd - t_createnode) * 1000);
}

template <typename PointT>
void Otd2D<PointT>::GetMap(typename pcl::PointCloud<PointT>::Ptr ground_map,
                           typename pcl::PointCloud<PointT>::Ptr nonground_map,
                           typename pcl::PointCloud<PointT>::Ptr dynamic_map) {
  for (auto iter = grids_map_.begin(); iter != grids_map_.end(); iter++) {
    iter->second->second.GetGroundPoints(ground_map);
    iter->second->second.GetNongroundPoints(nonground_map);
    iter->second->second.GetDynamicPoints(dynamic_map);
  }
}

template <typename PointT>
Eigen::Vector2i Otd2D<PointT>::Pos2Grid(const PointT pt) {
  return (Eigen::Vector2d(pt.x, pt.y) * inv_resolution_)
      .array()
      .round()
      .template cast<int>();
}

template <typename PointT>
void Otd2D<PointT>::GroundSegByPCA(
    typename pcl::PointCloud<PointT>::Ptr body_ptr,
    typename pcl::PointCloud<PointT>::Ptr ground_ptr,
    typename pcl::PointCloud<PointT>::Ptr nonground_ptr) {
  int num_horizon = 1800, lidar_scan = 128;
  double tau_seeds = 0.9, tau_dist = 0.2;
  bool preprocess = false;
  pcl::PointCloud<PointT> ground_pc_orig, cloud_noground;
  double t_start = omp_get_wtime();

  if (preprocess) {
    float horizonAngle;
    size_t columnIdn, rowIdn, index;
    double ang_res_x = 360. / float(num_horizon);
    std::vector<int> has_pt(num_horizon * lidar_scan, 0);
    std::vector<PointT> pc_with_scans;
    pc_with_scans.resize(num_horizon * lidar_scan);

    for (size_t i = 0; i < num_horizon * lidar_scan; i++) {
      pc_with_scans[i].x = 0;
      pc_with_scans[i].y = 0;
      pc_with_scans[i].z = 0;
    }

    int fine_count = 0;
    for (size_t i = 0; i < body_ptr->size(); i++) {
      int rowIdn = int(body_ptr->points[i].intensity);
      if (rowIdn >= 0 && rowIdn < lidar_scan) {
        // std::cout << rowIdn << " ";
        horizonAngle =
            atan2(body_ptr->points[i].x, body_ptr->points[i].y) * 180 / M_PI;
        columnIdn = -round((horizonAngle - 90.0) / ang_res_x) + num_horizon / 2;
        if (columnIdn >= num_horizon) columnIdn -= num_horizon;

        if (columnIdn < 0 || columnIdn >= num_horizon) {
          nonground_ptr->points.push_back(body_ptr->points[i]);
          continue;
        }
        fine_count++;
        index = columnIdn + rowIdn * num_horizon;
        pc_with_scans[index] = body_ptr->points[i];
        has_pt[index] = 1;
      } else
        nonground_ptr->points.push_back(body_ptr->points[i]);
    }

    size_t lowerInd, upperInd;
    float diffX, diffY, diffZ, angle;
    int find_num = 0;
    std::vector<bool> ground_true(num_horizon * lidar_scan, false);

    for (size_t j = 0; j < num_horizon; ++j) {
      // cout << "||||||||||||||||||||| ";
      for (size_t i = 0; i < lidar_scan - 1; ++i) {
        lowerInd = j + (i)*num_horizon;
        upperInd = j + (i + 1) * num_horizon;
        // cout << pc_with_scans[lowerInd].z << " ";
        // 图像内没有点投过来
        if (has_pt[lowerInd] == 1 && has_pt[upperInd] == 1) {
          find_num++;
          diffX = pc_with_scans[upperInd].x - pc_with_scans[lowerInd].x;
          diffY = pc_with_scans[upperInd].y - pc_with_scans[lowerInd].y;
          diffZ = pc_with_scans[upperInd].z - pc_with_scans[lowerInd].z;
          angle =
              atan2(diffZ, sqrt(diffX * diffX + diffY * diffY)) * 180 / M_PI;

          if (i < lidar_scan - 2) {
            if (abs(angle) <= 15)  // 是地面点
              ground_true[lowerInd] = true;
          } else {
            if (abs(angle) <= 15)  // 是地面点
            {
              ground_true[lowerInd] = true;
              ground_true[upperInd] = true;
            }
          }
        }
      }
      cout << endl;
    }

    for (size_t i = 0; i < ground_true.size(); i++) {
      PointT pt_temp = pc_with_scans[i];
      if (ground_true[i] && pt_temp.z > -tau_seeds && pt_temp.z < tau_seeds)
        ground_pc_orig.points.push_back(pt_temp);
      else
        nonground_ptr->points.push_back(pt_temp);
    }
    // printf("The fine, find and ground num is %d, %d and %ld\n", fine_count,
    // find_num, ground_pc_orig.size());
  } else {
    for (int i = 0; i < body_ptr->points.size(); i++) {
      if (body_ptr->points[i].z > -0.9) {
        if (body_ptr->points[i].z < 0.9)
          ground_pc_orig.points.push_back(body_ptr->points[i]);
        else
          nonground_ptr->points.push_back(body_ptr->points[i]);
      }
    }
  }
  double t_preprocess = omp_get_wtime();

  std::sort(ground_pc_orig.points.begin(), ground_pc_orig.points.end(),
            [](PointT a, PointT b) { return a.z < b.z; });
  ground_ptr->clear();

  for (int i = 0; i < ground_pc_orig.points.size(); i++) {
    if (ground_pc_orig.points[i].z > -(tau_seeds / 3)) {
      if (ground_pc_orig.points[i].z < tau_seeds / 3)
        ground_ptr->points.push_back(ground_pc_orig.points[i]);
      else
        break;
    }
  }

  Eigen::MatrixXf points(ground_pc_orig.points.size(), 3);
  int j = 0;
  for (auto p : ground_pc_orig.points) {
    points.row(j++) << p.x, p.y, p.z;
  }

  for (int i = 0; i < 3; i++) {
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*ground_ptr, cov, pc_mean);
    // Singular Value Decomposition: SVD
    Eigen::JacobiSVD<Eigen::MatrixXf> svd(
        cov, Eigen::DecompositionOptions::ComputeFullU);
    // use the least singular vector as normal
    Eigen::MatrixXf normal_ = (svd.matrixU().col(2));
    // mean ground seeds value
    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    // according to normal.T*[x,y,z] = -d
    float d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    // set distance threhold to `th_dist - d`
    float tau_disd_ = tau_dist - d_;

    ground_ptr->clear();
    cloud_noground.clear();
    // pointcloud to matrix

    // ground plane model
    Eigen::VectorXf result = points * normal_;
    // threshold filter
    for (int r = 0; r < result.rows(); r++) {
      if (result[r] < tau_disd_) {
        ground_ptr->points.push_back(ground_pc_orig[r]);
      } else {
        cloud_noground.points.push_back(ground_pc_orig[r]);
      }
    }
  }

  *nonground_ptr += cloud_noground;
  double t_refine = omp_get_wtime();
  printf("The preprocess time is %lfms\n", (t_preprocess - t_start) * 1000);
  printf("The refine time is %lfms\n", (t_refine - t_preprocess) * 1000);
}

}  // namespace otd

#endif  // OTD_H
