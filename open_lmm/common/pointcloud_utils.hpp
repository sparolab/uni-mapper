#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/shared_data.hpp>

#include <Eigen/Dense>
#include <memory>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_sort.h>
#include <unordered_map>
#include <vector>

#include "data_types.hpp"

#define HASH_X 73856093
#define HASH_Y 19349669
#define HASH_Z 83492791

// for down sample function
struct M_POINT {
  float xyz[3];
  float intensity;
  uint16_t count = 0;
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
pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromCustomType(
    std::string scan_file);

}  // namespace open_lmm