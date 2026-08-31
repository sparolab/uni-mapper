#pragma once

#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/rigid_transform.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/common/validation.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm {

inline Error InvariantError(std::string_view context, std::string detail) {
  return Error::InvalidArgument(std::string(context) + ": " +
                                std::move(detail));
}

inline Result<void> ValidateAgentRawData(const AgentRawData& raw,
                                         std::string_view context) {
  if (!raw.agent_id.IsValid()) {
    return Result<void>::Failure(
        InvariantError(context, "agent ID is invalid"));
  }
  if (raw.odom_poses.empty() || raw.filtered_scans.empty()) {
    return Result<void>::Failure(
        InvariantError(context, "poses and scans must not be empty"));
  }
  if (raw.odom_poses.size() != raw.filtered_scans.size()) {
    return Result<void>::Failure(InvariantError(
        context, "pose/scan cardinality mismatch: " +
                     std::to_string(raw.odom_poses.size()) + " poses, " +
                     std::to_string(raw.filtered_scans.size()) + " scans"));
  }
  for (std::size_t index = 0; index < raw.odom_poses.size(); ++index) {
    auto valid_pose = ValidateRigidTransform(
        raw.odom_poses[index],
        std::string(context) + " pose frame " + std::to_string(index));
    if (!valid_pose) return valid_pose;
    auto valid_scan = ValidatePointCloud(
        raw.filtered_scans[index],
        std::string(context) + " scan frame " + std::to_string(index));
    if (!valid_scan) return valid_scan;
  }
  return Result<void>::Ok();
}

inline Result<std::size_t> ValidateFrameIndex(
    const AgentRawDataMap& raw_data, const AgentRawData& current,
    const AgentId& agent, std::size_t frame, std::string_view context) {
  const AgentRawData* raw = nullptr;
  if (agent == current.agent_id) {
    raw = &current;
  } else {
    const auto found = raw_data.find(agent);
    if (found != raw_data.end() && found->second) raw = found->second.get();
  }
  if (!raw) {
    return Result<std::size_t>::Failure(InvariantError(
        context, "loop references unknown agent '" + agent.Value() + "'"));
  }
  if (frame >= raw->filtered_scans.size() || frame >= raw->odom_poses.size()) {
    return Result<std::size_t>::Failure(InvariantError(
        context, "frame " + std::to_string(frame) +
                     " is out of range for agent '" + agent.Value() + "'"));
  }
  return Result<std::size_t>::Ok(frame);
}

inline Result<void> ValidateReferencedAgentRawData(
    const AgentRawDataMap& raw_data, const AgentRawData& current,
    const AgentId& agent, std::string_view context) {
  if (agent == current.agent_id) return Result<void>::Ok();
  const auto found = raw_data.find(agent);
  if (found == raw_data.end() || !found->second) {
    return Result<void>::Failure(InvariantError(
        context, "loop references missing raw data for agent '" +
                     agent.Value() + "'"));
  }
  if (found->second->agent_id != agent) {
    return Result<void>::Failure(InvariantError(
        context, "raw-data map key/agent ID mismatch for agent '" +
                     agent.Value() + "'"));
  }
  return ValidateAgentRawData(*found->second, context);
}

inline Result<void> ValidateLoopPairs(
    const AgentRawData& current, const LoopPairVec& intra_loops,
    const LoopPairVec& inter_loops, const AgentRawDataMap& all_raw_data,
    std::string_view context) {
  auto validate = [&](const LoopPair& loop, bool intra,
                      std::size_t ordinal) -> Result<void> {
    const std::string loop_context = std::string(context) +
        (intra ? " intra loop " : " inter loop ") +
        std::to_string(ordinal);
    if (!loop.from.first.IsValid() || !loop.to.first.IsValid()) {
      return Result<void>::Failure(
          InvariantError(loop_context, "loop agent ID is invalid"));
    }
    if (intra) {
      if (loop.from.first != loop.to.first ||
          loop.from.first != current.agent_id) {
        return Result<void>::Failure(InvariantError(
            loop_context, "intra loop must join frames of the current agent"));
      }
    } else if (loop.from.first == loop.to.first ||
               loop.from.first != current.agent_id) {
      return Result<void>::Failure(InvariantError(
          loop_context,
          "inter loop must join the current source to a different agent"));
    }
    auto valid_source_raw = ValidateReferencedAgentRawData(
        all_raw_data, current, loop.from.first, loop_context + " source raw");
    if (!valid_source_raw) return valid_source_raw;
    auto valid_target_raw = ValidateReferencedAgentRawData(
        all_raw_data, current, loop.to.first, loop_context + " target raw");
    if (!valid_target_raw) return valid_target_raw;
    auto from = ValidateFrameIndex(all_raw_data, current, loop.from.first,
                                   loop.from.second, loop_context + " source");
    if (!from) return Result<void>::Failure(from.GetError());
    auto to = ValidateFrameIndex(all_raw_data, current, loop.to.first,
                                 loop.to.second, loop_context + " target");
    if (!to) return Result<void>::Failure(to.GetError());
    return ValidateRigidTransform(loop.init_rel_pose,
                                   loop_context + " transform");
  };
  for (std::size_t index = 0; index < intra_loops.size(); ++index) {
    auto result = validate(intra_loops[index], true, index);
    if (!result) return result;
  }
  for (std::size_t index = 0; index < inter_loops.size(); ++index) {
    auto result = validate(inter_loops[index], false, index);
    if (!result) return result;
  }
  return Result<void>::Ok();
}

inline bool FrameGapAtLeast(std::size_t lhs, std::size_t rhs,
                            std::size_t minimum_gap) noexcept {
  const std::size_t gap = lhs >= rhs ? lhs - rhs : rhs - lhs;
  return gap >= minimum_gap;
}

inline Result<void> ValidateOptimizedData(const AgentOptimizedData& optimized,
                                          const AgentRawData& raw,
                                          std::string_view context) {
  if (!optimized.agent_id.IsValid() || optimized.agent_id != raw.agent_id) {
    return Result<void>::Failure(
        InvariantError(context, "optimized/raw agent ID mismatch"));
  }
  if (optimized.optimized_poses.size() != raw.filtered_scans.size()) {
    return Result<void>::Failure(InvariantError(
        context, "optimized pose coverage differs from raw scan count"));
  }
  std::vector<bool> covered(raw.filtered_scans.size(), false);
  for (const auto& [frame_id, pose] : optimized.optimized_poses) {
    if (frame_id < 0 ||
        static_cast<std::size_t>(frame_id) >= raw.filtered_scans.size()) {
      return Result<void>::Failure(InvariantError(
          context, "optimized frame ID " + std::to_string(frame_id) +
                       " is out of range"));
    }
    if (covered[static_cast<std::size_t>(frame_id)]) {
      return Result<void>::Failure(InvariantError(
          context, "duplicate optimized frame ID " +
                       std::to_string(frame_id)));
    }
    covered[static_cast<std::size_t>(frame_id)] = true;
    auto valid = ValidateRigidTransform(
        pose, std::string(context) + " optimized frame " +
                  std::to_string(frame_id));
    if (!valid) return valid;
  }
  for (std::size_t frame = 0; frame < covered.size(); ++frame) {
    if (!covered[frame]) {
      return Result<void>::Failure(InvariantError(
          context, "missing optimized frame ID " + std::to_string(frame)));
    }
  }
  if (!optimized.kdtree_poses.empty() &&
      optimized.kdtree_poses.size() != optimized.optimized_poses.size()) {
    return Result<void>::Failure(InvariantError(
        context, "optimized pose/KD-tree cardinality mismatch"));
  }
  for (const auto& point : optimized.kdtree_poses) {
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z)) {
      return Result<void>::Failure(InvariantError(
          context, "optimized KD-tree contains a non-finite point"));
    }
  }
  return Result<void>::Ok();
}

inline Result<PoseVec> OrderOptimizedPosesByFrameId(
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
    std::size_t scan_count, std::string_view context) {
  if (scan_count == 0 || optimized_poses.size() != scan_count) {
    return Result<PoseVec>::Failure(InvariantError(
        context, "optimized pose count must exactly match scan count"));
  }
  PoseVec ordered(scan_count, Eigen::Isometry3d::Identity());
  std::vector<bool> covered(scan_count, false);
  for (const auto& [frame_id, pose] : optimized_poses) {
    if (frame_id < 0 || static_cast<std::size_t>(frame_id) >= scan_count) {
      return Result<PoseVec>::Failure(InvariantError(
          context, "optimized frame ID " + std::to_string(frame_id) +
                       " is out of range"));
    }
    const std::size_t index = static_cast<std::size_t>(frame_id);
    if (covered[index]) {
      return Result<PoseVec>::Failure(InvariantError(
          context, "duplicate optimized frame ID " +
                       std::to_string(frame_id)));
    }
    auto valid = ValidateRigidTransform(
        pose, std::string(context) + " optimized frame " +
                  std::to_string(frame_id));
    if (!valid) return Result<PoseVec>::Failure(valid.GetError());
    ordered[index] = pose;
    covered[index] = true;
  }
  for (std::size_t frame = 0; frame < covered.size(); ++frame) {
    if (!covered[frame]) {
      return Result<PoseVec>::Failure(InvariantError(
          context, "missing optimized frame ID " + std::to_string(frame)));
    }
  }
  return Result<PoseVec>::Ok(std::move(ordered));
}

}  // namespace open_lmm
