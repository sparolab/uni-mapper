#include "alignment_map_builder.hpp"

#include <open_lmm/core/algorithm_invariants.hpp>

#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <string>

namespace open_lmm {
namespace {

struct VoxelKey {
  int64_t x = 0;
  int64_t y = 0;
  int64_t z = 0;

  friend bool operator<(const VoxelKey& lhs, const VoxelKey& rhs) {
    if (lhs.x != rhs.x) return lhs.x < rhs.x;
    if (lhs.y != rhs.y) return lhs.y < rhs.y;
    return lhs.z < rhs.z;
  }
};

struct VoxelAccumulator {
  Eigen::Vector3d sum = Eigen::Vector3d::Zero();
  uint64_t count = 0;
};

Result<int64_t> VoxelCoordinate(double value, double inverse_voxel_size) {
  const double coordinate = std::floor(value * inverse_voxel_size);
  constexpr double kMinimum =
      static_cast<double>(std::numeric_limits<int64_t>::min());
  constexpr double kMaximum =
      static_cast<double>(std::numeric_limits<int64_t>::max());
  if (!std::isfinite(coordinate) || coordinate < kMinimum ||
      coordinate > kMaximum) {
    return Result<int64_t>::Failure(
        Error::InvalidArgument("alignment map voxel coordinate overflow"));
  }
  return Result<int64_t>::Ok(static_cast<int64_t>(coordinate));
}

}  // namespace

Result<VoxelizedAgentMap> BuildAlignmentMap(
    const AlgorithmExecutionContext& context, const AgentRawData& raw,
    float voxel_size_m) {
  if (!std::isfinite(voxel_size_m) || voxel_size_m <= 0.0F) {
    return Result<VoxelizedAgentMap>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(
            "alignment map voxel size must be finite and greater than zero"),
        context));
  }
  auto valid = ValidateAgentRawData(raw, "alignment map input");
  if (!valid) {
    return Result<VoxelizedAgentMap>::Failure(
        WithAlgorithmContext(valid.GetError(), context));
  }

  const double inverse_voxel_size = 1.0 / static_cast<double>(voxel_size_m);
  std::map<VoxelKey, VoxelAccumulator> voxels;
  for (std::size_t frame = 0; frame < raw.filtered_scans.size(); ++frame) {
    auto active =
        CheckAlgorithmCancellation(context, "while building alignment map");
    if (!active) {
      return Result<VoxelizedAgentMap>::Failure(active.GetError());
    }
    const auto& scan = raw.filtered_scans[frame];
    const auto& pose = raw.odom_poses[frame];
    for (const auto& point : scan->points) {
      const Eigen::Vector3d transformed =
          pose * Eigen::Vector3d(point.x, point.y, point.z);
      auto x = VoxelCoordinate(transformed.x(), inverse_voxel_size);
      auto y = VoxelCoordinate(transformed.y(), inverse_voxel_size);
      auto z = VoxelCoordinate(transformed.z(), inverse_voxel_size);
      if (!x || !y || !z) {
        const Error error = !x ? x.GetError() : !y ? y.GetError() : z.GetError();
        return Result<VoxelizedAgentMap>::Failure(
            WithAlgorithmContext(error, context));
      }
      auto& voxel = voxels[{x.Value(), y.Value(), z.Value()}];
      voxel.sum += transformed;
      ++voxel.count;
    }
  }
  if (voxels.empty()) {
    return Result<VoxelizedAgentMap>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("alignment map contains no occupied voxels"),
        context));
  }

  VoxelizedAgentMap output;
  output.voxel_size_m = voxel_size_m;
  output.points.reserve(voxels.size());
  for (const auto& [key, voxel] : voxels) {
    (void)key;
    output.points.push_back(
        (voxel.sum / static_cast<double>(voxel.count)).cast<float>());
  }
  auto completed =
      CheckAlgorithmCancellation(context, "after building alignment map");
  if (!completed) {
    return Result<VoxelizedAgentMap>::Failure(completed.GetError());
  }
  return Result<VoxelizedAgentMap>::Ok(std::move(output));
}

}  // namespace open_lmm
