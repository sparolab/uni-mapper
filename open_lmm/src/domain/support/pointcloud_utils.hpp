#pragma once

#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>

#include <Eigen/Dense>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <open_lmm/common/data_types.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#define HASH_X 73856093
#define HASH_Y 19349669
#define HASH_Z 83492791

// for down sample function
struct M_POINT {
  float xyz[3];
  float intensity;
  std::size_t count = 0;
};

class VOXEL_LOC {
 public:
  uint32_t x, y, z;

  VOXEL_LOC(uint32_t vx = 0, uint32_t vy = 0, uint32_t vz = 0)
      : x(vx), y(vy), z(vz) {}

  bool operator==(const VOXEL_LOC& other) const {
    return (x == other.x && y == other.y && z == other.z);
  }
};

// Hash value
template <>
struct std::hash<VOXEL_LOC> {
  std::uint32_t operator()(const VOXEL_LOC& s) const {
    return (s.x * HASH_X) ^ (s.y * HASH_Y) ^ (s.z * HASH_Z);
  }
};

namespace open_lmm {

// Reusable one-pass voxel reducer. Callers may feed points from a complete
// cloud or incrementally as scans become available; both paths therefore use
// exactly the same voxel-key and centroid semantics.
class IncrementalVoxelAccumulator {
 public:
  IncrementalVoxelAccumulator(float voxel_size, float min_range = 0.0F,
                              float max_range = 0.0F,
                              bool use_range_filter = false);

  void Add(const pcl::PointXYZI& point);
  [[nodiscard]] std::size_t Size() const noexcept { return voxels_.size(); }
  [[nodiscard]] std::size_t SourcePointCount() const noexcept {
    return source_point_count_;
  }

  template <typename Consumer>
  void ConsumeAverages(Consumer&& consumer) && {
    for (const auto& [voxel, accumulated] : voxels_) {
      (void)voxel;
      const float inverse_count = 1.0F / accumulated.count;
      consumer(accumulated.xyz[0] * inverse_count,
               accumulated.xyz[1] * inverse_count,
               accumulated.xyz[2] * inverse_count,
               accumulated.intensity * inverse_count);
    }
    voxels_.clear();
  }

 private:
  float inverse_voxel_size_ = 1.0F;
  float minimum_range_squared_ = 0.0F;
  float maximum_range_squared_ = 0.0F;
  bool use_range_filter_ = false;
  std::size_t source_point_count_ = 0;
  std::unordered_map<VOXEL_LOC, M_POINT> voxels_;
};

// Helper methods for transformations
std::vector<Eigen::Isometry3f> transformEigenPoses(
    const std::vector<Eigen::Isometry3d>& poses,
    const Eigen::Matrix4f& transform_matrix);

std::vector<Eigen::Vector3f> transformEigenPoints(
    const std::vector<Eigen::Vector3f>& map_points,
    const Eigen::Matrix4f& transform_matrix);

pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleWithRangeFilter(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr p_cloud, const float voxel_size,
    const float min_range = 2.0, const float max_range = 100.0,
    const bool use_range_filter = true);

void pclToEigen(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                std::vector<Eigen::Vector3f>& points);

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromPCD(std::string scan_file);
pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromBin(std::string scan_file);
pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromCustomType(std::string scan_file);

}  // namespace open_lmm
