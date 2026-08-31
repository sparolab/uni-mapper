#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <visualization/projection/visualization_source.hpp>

namespace open_lmm {

// Read model derived from one immutable committed runtime state or an isolated
// DataLoad candidate that can be discarded without mutating that state.
class VisualizationProjector {
 public:
  void Clear(uint64_t runtime_revision);
  void Publish(VisualizationSource source, VisualizationPhase phase,
               bool include_maps);
  // Publishes an uncommitted DataLoad read model at the base revision. It is
  // intentionally isolated from RuntimeStateStore and replaced on commit or
  // rollback by Publish().
  void PublishDataLoadCandidate(uint64_t base_revision, const AgentId& agent,
                                const AgentRawDataHandle& raw);
  void RollbackDataLoadCandidate(uint64_t base_revision);
  [[nodiscard]] Result<VisualizationSnapshot> Project(
      const VisualizationQuery& query) const;
  [[nodiscard]] std::size_t PointCacheEntryCount() const;
  [[nodiscard]] std::size_t PointCacheBytes() const;

 private:
  struct State {
    uint64_t revision = 0;
    VisualizationPhase phase = VisualizationPhase::kDataLoad;
    std::map<AgentId, VisualizationSnapshot> agents;
    std::map<AgentId, std::filesystem::path> map_paths;
    std::map<AgentId, AgentRawDataHandle> raw_data;
    std::map<AgentId, std::vector<Eigen::Isometry3d>> point_poses;
  };

  struct PointCacheKey {
    uint64_t revision = 0;
    VisualizationPhase phase = VisualizationPhase::kDataLoad;
    AgentId agent;
    uint32_t voxel_millimeters = 0;
    std::size_t maximum_points = 0;

    friend bool operator<(const PointCacheKey& left,
                          const PointCacheKey& right) {
      return std::tie(left.revision, left.phase, left.agent,
                      left.voxel_millimeters, left.maximum_points) <
             std::tie(right.revision, right.phase, right.agent,
                      right.voxel_millimeters, right.maximum_points);
    }
  };

  struct PointCacheEntry {
    std::vector<VisualizationPoint> points;
    Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
    Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
    bool has_bounds = false;
    std::size_t source_point_count = 0;
  };

  struct CachedPointEntry {
    std::shared_ptr<const PointCacheEntry> entry;
    std::size_t bytes = 0;
    uint64_t last_used = 0;
  };

  static constexpr std::size_t kMaximumPointCacheEntries = 16;
  static constexpr std::size_t kMaximumPointCacheBytes = 128U * 1024U * 1024U;
  void ClearPointCacheLocked() const;

  mutable std::mutex mutex_;
  std::shared_ptr<const State> state_ = std::make_shared<State>();
  std::optional<uint64_t> candidate_base_revision_;
  std::shared_ptr<const State> candidate_rollback_state_;
  mutable std::map<PointCacheKey, CachedPointEntry> point_cache_;
  mutable std::size_t point_cache_bytes_ = 0;
  mutable uint64_t point_cache_access_ = 0;
};

}  // namespace open_lmm
