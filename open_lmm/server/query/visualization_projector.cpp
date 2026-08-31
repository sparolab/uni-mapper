#include "visualization_projector.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <map>
#include <set>
#include <tuple>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>

namespace open_lmm {
namespace {
constexpr std::size_t kMaximumVisualizationPoints = 1'000'000;

using VoxelKey = std::tuple<int64_t, int64_t, int64_t>;

struct BuiltPointCache {
  std::vector<VisualizationPoint> points;
  Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
  bool has_bounds = false;
  std::size_t source_point_count = 0;
};

uint64_t HashVoxel(const VoxelKey& key) {
  uint64_t hash = 14695981039346656037ULL;
  const auto mix = [&hash](int64_t value) {
    const uint64_t bits = static_cast<uint64_t>(value);
    for (unsigned shift = 0; shift != 64; shift += 8) {
      hash ^= (bits >> shift) & 0xffU;
      hash *= 1099511628211ULL;
    }
  };
  mix(std::get<0>(key));
  mix(std::get<1>(key));
  mix(std::get<2>(key));
  return hash;
}

class BoundedVoxelPreview {
 public:
  BoundedVoxelPreview(float voxel_size, std::size_t maximum_points)
      : inverse_voxel_(1.0 / static_cast<double>(voxel_size)),
        maximum_points_(maximum_points) {}

  void Add(const Eigen::Vector3f& point, float intensity) {
    if (!point.allFinite()) return;
    ++source_point_count_;
    const VoxelKey voxel{
        static_cast<int64_t>(std::floor(point.x() * inverse_voxel_)),
        static_cast<int64_t>(std::floor(point.y() * inverse_voxel_)),
        static_cast<int64_t>(std::floor(point.z() * inverse_voxel_))};
    if (selected_voxels_.contains(voxel)) return;
    const std::pair<uint64_t, VoxelKey> priority{HashVoxel(voxel), voxel};
    if (points_.size() >= maximum_points_ &&
        priority >= points_.rbegin()->first) {
      return;
    }
    points_.emplace(priority,
                    VisualizationPoint{point.x(), point.y(), point.z(),
                                       intensity});
    selected_voxels_.insert(voxel);
    if (points_.size() > maximum_points_) {
      const auto worst = std::prev(points_.end());
      selected_voxels_.erase(worst->first.second);
      points_.erase(worst);
    }
  }

  BuiltPointCache Finish() && {
    BuiltPointCache result;
    result.source_point_count = source_point_count_;
    result.points.reserve(points_.size());
    for (const auto& [priority, point] : points_) {
      (void)priority;
      result.points.push_back(point);
      const Eigen::Vector3f position(point.x, point.y, point.z);
      if (!result.has_bounds) {
        result.min_bound = result.max_bound = position;
        result.has_bounds = true;
      } else {
        result.min_bound = result.min_bound.cwiseMin(position);
        result.max_bound = result.max_bound.cwiseMax(position);
      }
    }
    return result;
  }

 private:
  double inverse_voxel_;
  std::size_t maximum_points_;
  std::size_t source_point_count_ = 0;
  // Voxel identity is exact. Hashes only provide deterministic bounded
  // selection priority; collisions are ordered by the full voxel key.
  std::set<VoxelKey> selected_voxels_;
  std::map<std::pair<uint64_t, VoxelKey>, VisualizationPoint> points_;
};

const AgentPipelineCtx* FindContext(const RuntimePayload& payload,
                                    const AgentId& agent) {
  const auto found = std::find_if(
      payload.contexts.begin(), payload.contexts.end(),
      [&agent](const AgentPipelineCtx& item) { return item.agent.id == agent; });
  return found == payload.contexts.end() ? nullptr : &*found;
}

std::vector<Eigen::Isometry3d> PreviewPoses(
    const AgentRawData& raw, const AgentOptimizedDataHandle& optimized,
    const AgentPipelineCtx* context, VisualizationPhase phase) {
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
  if (phase == VisualizationPhase::kLoopDetection && context &&
      context->loop_output && context->loop_output->accepted_global_T_agent) {
    global_T_agent = *context->loop_output->accepted_global_T_agent;
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

void VisualizationProjector::Clear(uint64_t runtime_revision) {
  auto state = std::make_shared<State>();
  state->revision = runtime_revision;
  std::lock_guard lock(mutex_);
  state_ = std::move(state);
  candidate_base_revision_.reset();
  candidate_rollback_state_.reset();
  ClearPointCacheLocked();
}

void VisualizationProjector::Publish(
    std::shared_ptr<const RuntimeState> runtime, VisualizationPhase phase,
    bool include_maps) {
  if (!runtime || !runtime->config || !runtime->payload ||
      !runtime->payload->database) {
    Clear(runtime ? runtime->revision : 0);
    return;
  }

  auto state = std::make_shared<State>();
  state->revision = runtime->revision;
  state->phase = phase;
  const auto& database = *runtime->payload->database;
  for (const auto& [agent, raw] : database.raw_data) {
    if (!raw) continue;
    auto& snapshot = state->agents[agent];
    snapshot.agent = agent;
    snapshot.revision = runtime->revision;
    snapshot.phase = phase;
    snapshot.pose_kind = VisualizationPoseKind::kOdometry;
    snapshot.point_kind = phase == VisualizationPhase::kDataLoad
                              ? VisualizationPointKind::kFilteredScanPreview
                              : VisualizationPointKind::kOptimizationMapPreview;

    const auto optimized = database.optimized_data.find(agent);
    const AgentOptimizedDataHandle optimized_handle =
        optimized == database.optimized_data.end() ? nullptr
                                                   : optimized->second;
    const auto* context = FindContext(*runtime->payload, agent);
    auto poses = PreviewPoses(*raw, optimized_handle, context, phase);
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
    if (context && context->loop_output) {
      const auto append_loops = [&snapshot](const LoopPairVec& loops,
                                            VisualizationEdgeType type) {
        for (const auto& loop : loops) {
          snapshot.edges.push_back({loop.from.first, loop.from.second,
                                    loop.to.first, loop.to.second, type});
        }
      };
      append_loops(context->loop_output->intra_loops,
                   VisualizationEdgeType::kIntraLoop);
      append_loops(context->loop_output->inter_loops,
                   VisualizationEdgeType::kInterLoop);
    }

    state->raw_data.emplace(agent, raw);
    state->point_poses.emplace(agent, std::move(poses));
    snapshot.points_available = !raw->filtered_scans.empty();
    if (include_maps) {
      const auto path = runtime->config->root.output_directory /
                        ("global_map_" + agent.Value() + ".pcd");
      std::error_code error;
      if (std::filesystem::is_regular_file(path, error) && !error) {
        state->map_paths.emplace(agent, path);
        snapshot.point_kind = VisualizationPointKind::kFinalStaticMap;
        snapshot.points_available = true;
      }
    }
  }

  std::lock_guard lock(mutex_);
  state_ = std::move(state);
  candidate_base_revision_.reset();
  candidate_rollback_state_.reset();
  ClearPointCacheLocked();
}

void VisualizationProjector::PublishDataLoadCandidate(
    uint64_t base_revision, const AgentId& agent,
    const AgentRawDataHandle& raw) {
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
  next->map_paths.erase(agent);
  state_ = std::move(next);
  ClearPointCacheLocked();
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
      query.preview_voxel_size_m <= 0.0F || query.maximum_points == 0) {
    return Result<VisualizationSnapshot>::Failure(Error::InvalidArgument(
        "visualization query requires a positive voxel size and point limit"));
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

  const auto map_path = state->map_paths.find(query.agent);
  const bool final_static_map = map_path != state->map_paths.end();
  const uint32_t voxel_millimeters = static_cast<uint32_t>(std::clamp(
      std::llround(static_cast<double>(query.preview_voxel_size_m) * 1000.0),
      1LL,
      static_cast<long long>(std::numeric_limits<uint32_t>::max())));
  const std::size_t maximum_points =
      std::min(query.maximum_points, kMaximumVisualizationPoints);
  const PointCacheKey key{state->revision, state->phase, query.agent,
                          voxel_millimeters, maximum_points};
  std::shared_ptr<const PointCacheEntry> cached;
  {
    std::lock_guard lock(mutex_);
    const auto entry = point_cache_.find(key);
    if (entry != point_cache_.end()) {
      entry->second.last_used = ++point_cache_access_;
      cached = entry->second.entry;
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
      BoundedVoxelPreview builder(
          static_cast<float>(voxel_millimeters) / 1000.0F,
          maximum_points);
      std::size_t visited = 0;
      for (const auto& point : cloud) {
        if ((++visited & 4095U) == 0U) {
          if (auto stopped = cancelled()) return std::move(*stopped);
        }
        if (!pcl::isFinite(point)) continue;
        builder.Add(Eigen::Vector3f(point.x, point.y, point.z),
                    point.intensity);
      }
      auto built = std::move(builder).Finish();
      cached = std::make_shared<const PointCacheEntry>(PointCacheEntry{
          std::move(built.points), built.min_bound, built.max_bound,
          built.has_bounds, built.source_point_count});
    } else {
      BoundedVoxelPreview builder(
          static_cast<float>(voxel_millimeters) / 1000.0F,
          maximum_points);
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
          builder.Add((poses->second[frame] *
                       Eigen::Vector3d(point.x, point.y, point.z))
                          .cast<float>(),
                      point.intensity);
        }
      }
      auto built = std::move(builder).Finish();
      cached = std::make_shared<const PointCacheEntry>(PointCacheEntry{
          std::move(built.points), built.min_bound, built.max_bound,
          built.has_bounds, built.source_point_count});
    }
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
        while (point_cache_.size() > kMaximumPointCacheEntries ||
               point_cache_bytes_ > kMaximumPointCacheBytes) {
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
