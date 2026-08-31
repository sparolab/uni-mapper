#include "pointcloud_utils.hpp"

#include <Eigen/Dense>
#include <array>
#include <cmath>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <open_lmm/common/data_types.hpp>
#include <domain/support/validation.hpp>
#include <vector>

namespace open_lmm {

IncrementalVoxelAccumulator::IncrementalVoxelAccumulator(
    float voxel_size, float min_range, float max_range,
    bool use_range_filter)
    : minimum_range_squared_(min_range * min_range),
      maximum_range_squared_(max_range * max_range),
      use_range_filter_(use_range_filter) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    throw std::invalid_argument("voxel size must be finite and positive");
  }
  inverse_voxel_size_ = 1.0F / voxel_size;
  if (use_range_filter_ &&
      (!std::isfinite(min_range) || !std::isfinite(max_range) ||
       min_range < 0.0F || max_range <= min_range)) {
    throw std::invalid_argument(
        "range filter must satisfy 0 <= min_range < max_range");
  }
}

void IncrementalVoxelAccumulator::Add(const pcl::PointXYZI& point) {
  const float range_squared =
      point.x * point.x + point.y * point.y + point.z * point.z;
  if (use_range_filter_ &&
      (range_squared < minimum_range_squared_ ||
       range_squared > maximum_range_squared_)) {
    return;
  }
  ++source_point_count_;
  float location[3];
  for (int axis = 0; axis < 3; ++axis) {
    location[axis] = point.data[axis] * inverse_voxel_size_;
    if (location[axis] < 0.0F) location[axis] -= 1.0F;
  }
  const VOXEL_LOC voxel(
      static_cast<uint32_t>(static_cast<int64_t>(location[0])),
      static_cast<uint32_t>(static_cast<int64_t>(location[1])),
      static_cast<uint32_t>(static_cast<int64_t>(location[2])));
  auto entry = voxels_.find(voxel);
  if (entry != voxels_.end()) {
    entry->second.xyz[0] += point.x;
    entry->second.xyz[1] += point.y;
    entry->second.xyz[2] += point.z;
    entry->second.intensity += point.intensity;
    ++entry->second.count;
    return;
  }
  M_POINT accumulated;
  accumulated.xyz[0] = point.x;
  accumulated.xyz[1] = point.y;
  accumulated.xyz[2] = point.z;
  accumulated.intensity = point.intensity;
  accumulated.count = 1;
  voxels_.emplace(voxel, accumulated);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr downsampleWithRangeFilter(
    pcl::PointCloud<pcl::PointXYZI>::Ptr p_scan, const float voxel_size,
    const float min_range, const float max_range, const bool use_range_filter) {
  auto valid = ValidateVoxelizationInput(p_scan, voxel_size, min_range,
                                         max_range, use_range_filter,
                                         "point-cloud downsample");
  if (!valid) throw std::invalid_argument(valid.GetError().Message());
  if (voxel_size < 0.01f) return p_scan;

  IncrementalVoxelAccumulator accumulator(voxel_size, min_range, max_range,
                                          use_range_filter);
  for (const auto& point : *p_scan) accumulator.Add(point);

  const std::size_t num_scan_filtered = accumulator.Size();
  pcl::PointCloud<pcl::PointXYZI>::Ptr p_scan_filtered =
      std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  p_scan_filtered->resize(num_scan_filtered);
  auto output = p_scan_filtered->begin();
  std::move(accumulator).ConsumeAverages(
      [&output](float x, float y, float z, float intensity) {
        *output++ = pcl::PointXYZI{x, y, z, intensity};
      });

  return p_scan_filtered;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromPCD(std::string scan_file) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (pcl::io::loadPCDFile(scan_file, *cloud) != 0) {
    throw std::runtime_error("failed to load PCD file: " + scan_file);
  }
  if (cloud->empty()) {
    throw std::runtime_error("PCD file contains no points: " + scan_file);
  }
  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr readPointsFromBin(
    const std::string scan_file) {
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  std::ifstream input(scan_file, std::ios::binary);
  if (!input) {
    throw std::runtime_error("failed to open BIN file: " + scan_file);
  }

  std::array<float, 4> values{};
  while (input.read(reinterpret_cast<char*>(values.data()), sizeof(values))) {
    cloud->emplace_back(values[0], values[1], values[2], values[3]);
  }
  if (!input.eof() || input.gcount() != 0) {
    throw std::runtime_error("truncated BIN point record: " + scan_file);
  }
  if (cloud->empty()) {
    throw std::runtime_error("BIN file contains no points: " + scan_file);
  }
  return cloud;
}

}  // namespace open_lmm
