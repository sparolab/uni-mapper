#pragma once

#include <cstdint>
#include <limits>

#include <open_lmm/common/agent_data.hpp>

namespace open_lmm::data_loader_memory {

inline uint64_t SaturatingAdd(uint64_t lhs, uint64_t rhs) noexcept {
  return rhs > std::numeric_limits<uint64_t>::max() - lhs
             ? std::numeric_limits<uint64_t>::max()
             : lhs + rhs;
}

inline uint64_t SaturatingMultiply(uint64_t lhs, uint64_t rhs) noexcept {
  if (lhs == 0 || rhs == 0) return 0;
  return rhs > std::numeric_limits<uint64_t>::max() / lhs
             ? std::numeric_limits<uint64_t>::max()
             : lhs * rhs;
}

inline uint64_t ResidentRawDataBaseBytes(const AgentId& agent_id,
                                         const PoseVec& poses,
                                         const ScanVec& scans) noexcept {
  uint64_t bytes = sizeof(AgentRawData);
  bytes = SaturatingAdd(bytes, agent_id.Value().capacity());
  bytes = SaturatingAdd(
      bytes, SaturatingMultiply(poses.capacity(), sizeof(PoseVec::value_type)));
  bytes = SaturatingAdd(
      bytes, SaturatingMultiply(scans.capacity(), sizeof(ScanVec::value_type)));
  return bytes;
}

inline uint64_t ResidentFilteredScanBytes(
    const pcl::PointCloud<pcl::PointXYZI>& scan) noexcept {
  return SaturatingAdd(
      sizeof(scan),
      SaturatingMultiply(scan.points.capacity(), sizeof(pcl::PointXYZI)));
}

inline uint64_t ResidentRawDataBytes(const AgentRawData& raw) noexcept {
  uint64_t bytes =
      ResidentRawDataBaseBytes(raw.agent_id, raw.odom_poses,
                               raw.filtered_scans);
  for (const auto& scan : raw.filtered_scans) {
    if (scan) bytes = SaturatingAdd(bytes, ResidentFilteredScanBytes(*scan));
  }
  return bytes == 0 ? 1 : bytes;
}

}  // namespace open_lmm::data_loader_memory
