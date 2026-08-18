#pragma once

#include <cstddef>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace open_lmm {

inline Result<void> ValidateFinitePoint(const pcl::PointXYZI& point,
                                        std::string_view context) {
  if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
      !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": point contains a non-finite value"));
  }
  return Result<void>::Ok();
}

inline Result<void> ValidatePointCloud(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    std::string_view context, bool allow_empty = false) {
  if (!cloud) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": scan is null"));
  }
  if (!allow_empty && cloud->empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": scan is empty"));
  }
  for (std::size_t index = 0; index < cloud->size(); ++index) {
    auto valid = ValidateFinitePoint((*cloud)[index],
                                     std::string(context) + " point " +
                                         std::to_string(index));
    if (!valid) return valid;
  }
  return Result<void>::Ok();
}

inline Result<void> ValidateVoxelizationInput(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, float voxel_size,
    float min_range, float max_range, bool use_range_filter,
    std::string_view context) {
  if (!std::isfinite(voxel_size) || voxel_size <= 0.0F) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": voxel size must be finite and positive"));
  }
  if (!std::isfinite(min_range) || !std::isfinite(max_range) ||
      min_range < 0.0F ||
      (use_range_filter && max_range <= min_range)) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) +
        ": range must be finite and satisfy 0 <= min < max"));
  }
  auto valid_cloud = ValidatePointCloud(cloud, context);
  if (!valid_cloud) return valid_cloud;

  const long double inverse = 1.0L / static_cast<long double>(voxel_size);
  constexpr long double kIntegerMin =
      static_cast<long double>(std::numeric_limits<int64_t>::min()) + 1.0L;
  constexpr long double kIntegerMax =
      static_cast<long double>(std::numeric_limits<int64_t>::max()) - 1.0L;
  for (std::size_t point_index = 0; point_index < cloud->size(); ++point_index) {
    const auto& point = (*cloud)[point_index];
    for (int axis = 0; axis < 3; ++axis) {
      const long double coordinate =
          static_cast<long double>(point.data[axis]) * inverse;
      if (!std::isfinite(coordinate) || coordinate < kIntegerMin ||
          coordinate > kIntegerMax) {
        return Result<void>::Failure(Error::InvalidArgument(
            std::string(context) + ": point " +
            std::to_string(point_index) +
            " voxel coordinate cannot be represented as int64"));
      }
    }
  }
  return Result<void>::Ok();
}

inline Result<void> ValidateScanPoseCount(std::size_t scan_count,
                                          std::size_t pose_count,
                                          std::string_view context) {
  if (scan_count == 0 || pose_count == 0) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(context) + ": scans and poses must not be empty"));
  }
  if (scan_count == pose_count) return Result<void>::Ok();
  return Result<void>::Failure(Error::InvalidArgument(
      std::string(context) + ": scan count (" + std::to_string(scan_count) +
      ") does not match pose count (" + std::to_string(pose_count) + ")"));
}

inline Result<void> ValidateAgentIndex(std::size_t index,
                                       std::size_t agent_count,
                                       std::string_view context) {
  if (index < agent_count) return Result<void>::Ok();
  return Result<void>::Failure(Error::InvalidArgument(
      std::string(context) + ": agent index " + std::to_string(index) +
      " is out of range for " + std::to_string(agent_count) + " agents"));
}

inline Result<std::size_t> ValidateNearestNeighborResult(
    int found, const std::vector<int>& indices, std::size_t candidate_count,
    std::string_view context) {
  if (found <= 0 || indices.empty()) {
    return Result<std::size_t>::Failure(Error::InvalidArgument(
        std::string(context) + ": nearest-neighbor search returned no result"));
  }
  if (indices.front() < 0 ||
      static_cast<std::size_t>(indices.front()) >= candidate_count) {
    return Result<std::size_t>::Failure(Error::InvalidArgument(
        std::string(context) + ": nearest-neighbor index is out of range"));
  }
  return Result<std::size_t>::Ok(static_cast<std::size_t>(indices.front()));
}

}  // namespace open_lmm
