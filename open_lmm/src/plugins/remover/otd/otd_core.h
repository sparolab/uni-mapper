#ifndef OTD_H
#define OTD_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <list>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace otd {

using namespace std;

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

  std::vector<KeyType3D> nearby_grids_;

 public:
  Otd3D(const double tau_ob_ratio, const double resolution);
  ~Otd3D() {}
  void Run(typename pcl::PointCloud<PointT>::Ptr ground_ptr,
           typename pcl::PointCloud<PointT>::Ptr nonground_ptr,
           const int frame_num);
  typename pcl::PointCloud<PointT>::Ptr GetMap(bool replace_intensity);
  Eigen::Vector3i Pos2Grid3D(const PointT pt);
};

template <typename PointT>
Otd3D<PointT>::Otd3D(const double tau_ob_ratio, const double resolution)
    : resolution_(resolution),
      inv_resolution_(1. / resolution),
      tau_ob_ratio_(tau_ob_ratio) {
  printf("The tau_ob_ratio is %lf\n", tau_ob_ratio_);
  printf("The resolution and inv_resolution is %lf and %lf\n", resolution_,
         inv_resolution_);
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
}

template <typename PointT>
Eigen::Vector3i Otd3D<PointT>::Pos2Grid3D(const PointT pt) {
  return (Eigen::Vector3d(pt.x, pt.y, pt.z) * inv_resolution_)
      .array()
      .round()
      .template cast<int>();
}

}  // namespace otd

#endif  // OTD_H
