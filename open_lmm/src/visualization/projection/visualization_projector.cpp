#include "visualization_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/cancellation.hpp>
#include <domain/support/pointcloud_utils.hpp>

namespace open_lmm {
namespace {
std::shared_ptr<const VisualizationPointPreview> FinishVoxelPreview(
    IncrementalVoxelAccumulator accumulator, uint32_t voxel_millimeters) {
  auto preview = std::make_shared<VisualizationPointPreview>();
  preview->voxel_millimeters = voxel_millimeters;
  preview->source_point_count = accumulator.SourcePointCount();
  preview->points.reserve(accumulator.Size());
  std::move(accumulator).ConsumeAverages(
      [&preview](float x, float y, float z, float intensity) {
        preview->points.push_back({x, y, z, intensity});
        const Eigen::Vector3f position(x, y, z);
        if (!preview->has_bounds) {
          preview->min_bound = preview->max_bound = position;
          preview->has_bounds = true;
        } else {
          preview->min_bound = preview->min_bound.cwiseMin(position);
          preview->max_bound = preview->max_bound.cwiseMax(position);
        }
      });
  return preview;
}

std::vector<Eigen::Isometry3d> PreviewPoses(
    const AgentRawData& raw, const AgentOptimizedDataHandle& optimized,
    const std::shared_ptr<const LoopDetectorOutput>& loop_output,
    VisualizationPhase phase) {
  if (optimized && phase != VisualizationPhase::kDataLoad &&
      phase != VisualizationPhase::kLoopDetection) {
    std::vector<Eigen::Isometry3d> poses(raw.odom_poses.size(),
                                         Eigen::Isometry3d::Identity());
    for (const auto& [frame, pose] : optimized->optimized_poses) {
      if (frame >= 0 && static_cast<std::size_t>(frame) < poses.size()) {
        poses[static_cast<std::size_t>(frame)] = pose;
      }
    }
    return poses;
  }
  Eigen::Isometry3d global_T_agent = Eigen::Isometry3d::Identity();
  if (phase == VisualizationPhase::kLoopDetection && loop_output &&
      loop_output->accepted_global_T_agent) {
    global_T_agent = *loop_output->accepted_global_T_agent;
  }
  auto poses = raw.odom_poses;
  for (auto& pose : poses) pose = global_T_agent * pose;
  return poses;
}

}  // namespace

void VisualizationProjector::ClearPointCacheLocked() const {
  point_cache_.clear();
  point_cache_bytes_ = 0;
}

void VisualizationProjector::ClearPointCacheForAgentLocked(
    const AgentId& agent) const {
  for (auto entry = point_cache_.begin(); entry != point_cache_.end();) {
    if (entry->first.agent == agent) {
      point_cache_bytes_ -= entry->second.bytes;
      entry = point_cache_.erase(entry);
    } else {
      ++entry;
    }
  }
}

void VisualizationProjector::Clear(uint64_t runtime_revision,
                                   float preview_voxel_size_m) {
  auto state = std::make_shared<State>();
  state->revision = runtime_revision;
  state->preview_voxel_size_m = preview_voxel_size_m;
  std::lock_guard lock(mutex_);
  state_ = std::move(state);
  candidate_base_revision_.reset();
  candidate_rollback_state_.reset();
  ClearPointCacheLocked();
}

std::shared_ptr<VisualizationProjector::State>
VisualizationProjector::MakeState(
    VisualizationSource source, VisualizationPhase phase,
    bool include_maps) {
  auto state = std::make_shared<State>();
  state->revision = source.revision;
  state->phase = phase;
  state->preview_voxel_size_m = source.preview_voxel_size_m;
  for (const auto& agent_source : source.agents) {
    const auto& agent = agent_source.agent;
    const auto& raw = agent_source.raw_data;
    if (!raw) continue;
    auto& snapshot = state->agents[agent];
    snapshot.agent = agent;
    snapshot.revision = source.revision;
    snapshot.phase = phase;
    snapshot.pose_kind = VisualizationPoseKind::kOdometry;
    snapshot.point_kind = phase == VisualizationPhase::kDataLoad
                              ? VisualizationPointKind::kFilteredScanPreview
                              : VisualizationPointKind::kOptimizationMapPreview;

    const auto& optimized_handle = agent_source.optimized_data;
    auto poses =
        PreviewPoses(*raw, optimized_handle, agent_source.loop_output, phase);
    if (optimized_handle && phase != VisualizationPhase::kDataLoad &&
        phase != VisualizationPhase::kLoopDetection) {
      snapshot.pose_kind = VisualizationPoseKind::kOptimized;
      snapshot.poses.reserve(optimized_handle->optimized_poses.size());
      for (const auto& [index, pose] : optimized_handle->optimized_poses) {
        snapshot.poses.push_back({index, pose.cast<float>()});
      }
    } else {
      snapshot.poses.reserve(poses.size());
      for (std::size_t index = 0; index < poses.size(); ++index) {
        snapshot.poses.push_back(
            {static_cast<int>(index), poses[index].cast<float>()});
      }
    }
    for (std::size_t index = 1; index < snapshot.poses.size(); ++index) {
      snapshot.edges.push_back(
          {agent, static_cast<std::size_t>(snapshot.poses[index - 1].index),
           agent, static_cast<std::size_t>(snapshot.poses[index].index),
           VisualizationEdgeType::kTrajectory});
    }
    if (agent_source.loop_output) {
      const auto append_loops = [&snapshot](const LoopPairVec& loops,
                                            VisualizationEdgeType type) {
        for (const auto& loop : loops) {
          snapshot.edges.push_back({loop.from.first, loop.from.second,
                                    loop.to.first, loop.to.second, type});
        }
      };
      append_loops(agent_source.loop_output->intra_loops,
                   VisualizationEdgeType::kIntraLoop);
      append_loops(agent_source.loop_output->inter_loops,
                   VisualizationEdgeType::kInterLoop);
    }

    state->raw_data.emplace(agent, raw);
    state->point_poses.emplace(agent, std::move(poses));
    snapshot.points_available = !raw->filtered_scans.empty();
    if (include_maps) {
      const auto path = source.output_directory /
                        ("global_map_" + agent.Value() + ".pcd");
      std::error_code error;
      if (std::filesystem::is_regular_file(path, error) && !error) {
        state->map_paths.emplace(agent, path);
        snapshot.point_kind = VisualizationPointKind::kFinalStaticMap;
        snapshot.points_available = true;
      }
    }
  }

  return state;
}

void VisualizationProjector::Publish(
    VisualizationSource source, VisualizationPhase phase,
    bool include_maps) {
  auto state = MakeState(std::move(source), phase, include_maps);
  std::lock_guard lock(mutex_);
  if (phase == VisualizationPhase::kDataLoad && candidate_base_revision_) {
    for (const auto& [agent, raw] : state->raw_data) {
      const auto candidate_raw = state_->raw_data.find(agent);
      const auto candidate_preview = state_->data_load_previews.find(agent);
      if (candidate_raw != state_->raw_data.end() &&
          candidate_raw->second == raw &&
          candidate_preview != state_->data_load_previews.end() &&
          candidate_preview->second) {
        state->data_load_previews.emplace(agent, candidate_preview->second);
      }
    }
  }
  state_ = std::move(state);
  candidate_base_revision_.reset();
  candidate_rollback_state_.reset();
  ClearPointCacheLocked();
}

void VisualizationProjector::PublishAlignmentCandidate(
    VisualizationSource source) {
  const uint64_t base_revision = source.revision;
  auto state = MakeState(std::move(source),
                         VisualizationPhase::kOptimization, false);
  std::lock_guard lock(mutex_);
  if (state_->revision > base_revision) return;
  if (candidate_base_revision_ != base_revision) {
    candidate_base_revision_ = base_revision;
    candidate_rollback_state_ = state_;
  }
  state_ = std::move(state);
  ClearPointCacheLocked();
}

void VisualizationProjector::RollbackAlignmentCandidate(
    uint64_t base_revision) {
  std::lock_guard lock(mutex_);
  if (candidate_base_revision_ != base_revision ||
      !candidate_rollback_state_ ||
      state_->phase != VisualizationPhase::kOptimization) {
    return;
  }
  state_ = std::move(candidate_rollback_state_);
  candidate_base_revision_.reset();
  ClearPointCacheLocked();
}

void VisualizationProjector::PublishDataLoadCandidate(
    uint64_t base_revision, const AgentId& agent,
    const AgentRawDataHandle& raw, VisualizationPointPreviewHandle preview) {
  if (!agent.IsValid() || !raw) return;

  std::lock_guard lock(mutex_);
  if (state_->revision > base_revision) return;
  std::shared_ptr<State> next;
  if (candidate_base_revision_ == base_revision) {
    next = std::make_shared<State>(*state_);
  } else {
    candidate_base_revision_ = base_revision;
    candidate_rollback_state_ = state_;
    next = std::make_shared<State>();
    next->preview_voxel_size_m = state_->preview_voxel_size_m;
  }
  next->revision = base_revision;
  next->phase = VisualizationPhase::kDataLoad;

  VisualizationSnapshot snapshot;
  snapshot.agent = agent;
  snapshot.revision = base_revision;
  snapshot.phase = VisualizationPhase::kDataLoad;
  snapshot.pose_kind = VisualizationPoseKind::kOdometry;
  snapshot.point_kind = VisualizationPointKind::kFilteredScanPreview;
  snapshot.poses.reserve(raw->odom_poses.size());
  for (std::size_t index = 0; index < raw->odom_poses.size(); ++index) {
    snapshot.poses.push_back(
        {static_cast<int>(index), raw->odom_poses[index].cast<float>()});
    if (index != 0) {
      snapshot.edges.push_back(
          {agent, index - 1, agent, index,
           VisualizationEdgeType::kTrajectory});
    }
  }
  snapshot.points_available = !raw->filtered_scans.empty();
  next->agents[agent] = std::move(snapshot);
  next->raw_data[agent] = raw;
  next->point_poses[agent] = raw->odom_poses;
  if (preview) {
    next->preview_voxel_size_m =
        static_cast<float>(preview->voxel_millimeters) / 1000.0F;
    next->data_load_previews[agent] = std::move(preview);
  } else {
    next->data_load_previews.erase(agent);
  }
  next->map_paths.erase(agent);
  state_ = std::move(next);
  ClearPointCacheForAgentLocked(agent);
}

void VisualizationProjector::RollbackDataLoadCandidate(
    uint64_t base_revision) {
  std::lock_guard lock(mutex_);
  if (candidate_base_revision_ != base_revision ||
      !candidate_rollback_state_) {
    return;
  }
  state_ = std::move(candidate_rollback_state_);
  candidate_base_revision_.reset();
  ClearPointCacheLocked();
}

Result<VisualizationSnapshot> VisualizationProjector::Project(
    const VisualizationQuery& query) const {
  const auto cancellation = CurrentCancellationToken();
  const auto cancelled = [&]() -> std::optional<Result<VisualizationSnapshot>> {
    if (cancellation && cancellation->IsCancellationRequested()) {
      return Result<VisualizationSnapshot>::Failure(
          Error::Cancelled("visualization projection was superseded"));
    }
    return std::nullopt;
  };
  if (auto stopped = cancelled()) return std::move(*stopped);
  if (!query.agent.IsValid()) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("visualization query agent is invalid"));
  }
  if (!std::isfinite(query.preview_voxel_size_m) ||
      query.preview_voxel_size_m < 0.0F) {
    return Result<VisualizationSnapshot>::Failure(Error::InvalidArgument(
        "visualization query voxel size must be non-negative"));
  }

  std::shared_ptr<const State> state;
  {
    std::lock_guard lock(mutex_);
    state = state_;
  }
  const auto found = state->agents.find(query.agent);
  if (found == state->agents.end()) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("visualization is not available for agent"));
  }
  VisualizationSnapshot snapshot = found->second;
  if (!query.include_points || !snapshot.points_available) {
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }

  const float preview_voxel_size_m =
      query.preview_voxel_size_m > 0.0F ? query.preview_voxel_size_m
                                       : state->preview_voxel_size_m;
  if (!std::isfinite(preview_voxel_size_m) ||
      preview_voxel_size_m <= 0.0F) {
    return Result<VisualizationSnapshot>::Failure(Error::InvalidArgument(
        "runtime visualization voxel size is unavailable"));
  }

  const auto map_path = state->map_paths.find(query.agent);
  const bool final_static_map = map_path != state->map_paths.end();
  const uint32_t voxel_millimeters = static_cast<uint32_t>(std::clamp(
      std::llround(static_cast<double>(preview_voxel_size_m) * 1000.0),
      1LL,
      static_cast<long long>(std::numeric_limits<uint32_t>::max())));
  const PointCacheKey key{state->revision, state->phase, query.agent,
                          voxel_millimeters};
  std::shared_ptr<const PointCacheEntry> cached;
  {
    std::lock_guard lock(mutex_);
    const auto entry = point_cache_.find(key);
    if (entry != point_cache_.end()) {
      entry->second.last_used = ++point_cache_access_;
      cached = entry->second.entry;
    }
  }
  bool cache_new_entry = false;
  if (!cached && !final_static_map) {
    const auto preview = state->data_load_previews.find(query.agent);
    if (preview != state->data_load_previews.end() && preview->second &&
        preview->second->voxel_millimeters == voxel_millimeters) {
      cached = preview->second;
      cache_new_entry = true;
    }
  }
  if (!cached) {
    if (final_static_map) {
      pcl::PointCloud<pcl::PointXYZI> cloud;
      if (pcl::io::loadPCDFile(map_path->second.string(), cloud) < 0) {
        return Result<VisualizationSnapshot>::Failure(
            Error::IoError("failed to read visualization map " +
                           map_path->second.string()));
      }
      if (auto stopped = cancelled()) return std::move(*stopped);
      IncrementalVoxelAccumulator accumulator(
          static_cast<float>(voxel_millimeters) / 1000.0F);
      std::size_t visited = 0;
      for (const auto& point : cloud) {
        if ((++visited & 4095U) == 0U) {
          if (auto stopped = cancelled()) return std::move(*stopped);
        }
        if (!pcl::isFinite(point)) continue;
        accumulator.Add(point);
      }
      cached = FinishVoxelPreview(std::move(accumulator), voxel_millimeters);
    } else {
      IncrementalVoxelAccumulator accumulator(
          static_cast<float>(voxel_millimeters) / 1000.0F);
      const auto raw = state->raw_data.find(query.agent);
      const auto poses = state->point_poses.find(query.agent);
      if (raw == state->raw_data.end() || poses == state->point_poses.end()) {
        return Result<VisualizationSnapshot>::Failure(
            Error::InvalidArgument("visualization point source is unavailable"));
      }
      const std::size_t count =
          std::min(raw->second->filtered_scans.size(), poses->second.size());
      for (std::size_t frame = 0; frame < count; ++frame) {
        if (auto stopped = cancelled()) return std::move(*stopped);
        const auto& scan = raw->second->filtered_scans[frame];
        if (!scan) continue;
        std::size_t visited = 0;
        for (const auto& point : *scan) {
          if ((++visited & 4095U) == 0U) {
            if (auto stopped = cancelled()) return std::move(*stopped);
          }
          if (!pcl::isFinite(point)) continue;
          const Eigen::Vector3f transformed =
              (poses->second[frame] *
               Eigen::Vector3d(point.x, point.y, point.z))
                  .cast<float>();
          if (!transformed.allFinite()) continue;
          pcl::PointXYZI global_point;
          global_point.x = transformed.x();
          global_point.y = transformed.y();
          global_point.z = transformed.z();
          global_point.intensity = point.intensity;
          accumulator.Add(global_point);
        }
      }
      cached = FinishVoxelPreview(std::move(accumulator), voxel_millimeters);
    }
    cache_new_entry = true;
  }
  if (cache_new_entry) {
    if (auto stopped = cancelled()) return std::move(*stopped);
    std::lock_guard lock(mutex_);
    if (state_ == state) {
      const auto existing = point_cache_.find(key);
      if (existing != point_cache_.end()) {
        existing->second.last_used = ++point_cache_access_;
        cached = existing->second.entry;
      } else {
        const std::size_t bytes =
            cached->points.size() * sizeof(VisualizationPoint);
        point_cache_.emplace(
            key, CachedPointEntry{cached, bytes, ++point_cache_access_});
        point_cache_bytes_ += bytes;
        while (point_cache_.size() > kMaximumPointCacheEntries) {
          const auto oldest = std::min_element(
              point_cache_.begin(), point_cache_.end(),
              [](const auto& left, const auto& right) {
                return left.second.last_used < right.second.last_used;
              });
          point_cache_bytes_ -= oldest->second.bytes;
          point_cache_.erase(oldest);
        }
      }
    }
  }

  snapshot.points = cached->points;
  snapshot.min_bound = cached->min_bound;
  snapshot.max_bound = cached->max_bound;
  snapshot.has_bounds = cached->has_bounds;
  snapshot.source_point_count = cached->source_point_count;
  snapshot.displayed_point_count = cached->points.size();
  snapshot.points_complete = true;
  snapshot.map_available = snapshot.point_kind ==
                           VisualizationPointKind::kFinalStaticMap;
  return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
}

std::size_t VisualizationProjector::PointCacheEntryCount() const {
  std::lock_guard lock(mutex_);
  return point_cache_.size();
}

std::size_t VisualizationProjector::PointCacheBytes() const {
  std::lock_guard lock(mutex_);
  return point_cache_bytes_;
}

}  // namespace open_lmm
