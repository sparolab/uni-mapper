#pragma once

#include <cstddef>
#include <string>
#include <string_view>
#include <vector>

#include <open_lmm/common/result.hpp>

namespace open_lmm {

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
