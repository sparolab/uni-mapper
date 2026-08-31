#pragma once

#include <map>
#include <memory>
#include <mutex>
#include <optional>

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <visualization/projection/visualization_source.hpp>

namespace open_lmm {

struct VisualizationProjectorDiagnostics {
  std::size_t entries = 0;
  std::size_t bytes = 0;
  uint64_t hits = 0;
  uint64_t misses = 0;
  uint64_t insertions = 0;
  uint64_t evictions = 0;
  uint64_t clears = 0;
};

// Read model derived from one immutable committed runtime state or an isolated
// DataLoad candidate that can be discarded without mutating that state.
class VisualizationProjector {
 public:
  void Clear(uint64_t runtime_revision, float preview_voxel_size_m = 0.0F);
  void Publish(VisualizationSource source, VisualizationPhase phase,
               bool include_maps);
  // Publishes the latest optimizer-owned read model without committing the
  // runtime. The first update retains the committed projector state so a
  // failed or cancelled Alignment can restore it exactly.
  void PublishAlignmentCandidate(VisualizationSource source);
  void RollbackAlignmentCandidate(uint64_t base_revision);
  // Publishes an uncommitted DataLoad read model at the base revision. It is
  // intentionally isolated from RuntimeStateStore and replaced on commit or
  // rollback by Publish().
  void PublishDataLoadCandidate(uint64_t base_revision, const AgentId& agent,
                                const AgentRawDataHandle& raw,
                                VisualizationPointPreviewHandle preview = {});
  void RollbackDataLoadCandidate(uint64_t base_revision);
  [[nodiscard]] Result<VisualizationSnapshot> Project(
      const VisualizationQuery& query) const;
  [[nodiscard]] std::size_t PointCacheEntryCount() const;
  [[nodiscard]] std::size_t PointCacheBytes() const;
  [[nodiscard]] VisualizationProjectorDiagnostics Diagnostics() const;

 private:
  struct State {
    uint64_t revision = 0;
    VisualizationPhase phase = VisualizationPhase::kDataLoad;
    float preview_voxel_size_m = 0.0F;
    std::map<AgentId, VisualizationSnapshot> agents;
    std::map<AgentId, std::filesystem::path> map_paths;
    std::map<AgentId, AgentRawDataHandle> raw_data;
    std::map<AgentId, std::vector<Eigen::Isometry3d>> point_poses;
    std::map<AgentId, VisualizationPointPreviewHandle> data_load_previews;
  };

  struct PointCacheKey {
    uint64_t revision = 0;
    VisualizationPhase phase = VisualizationPhase::kDataLoad;
    AgentId agent;
    uint32_t voxel_millimeters = 0;

    friend bool operator<(const PointCacheKey& left,
                          const PointCacheKey& right) {
      return std::tie(left.revision, left.phase, left.agent,
                      left.voxel_millimeters) <
             std::tie(right.revision, right.phase, right.agent,
                      right.voxel_millimeters);
    }
  };

  using PointCacheEntry = VisualizationPointPreview;

  struct CachedPointEntry {
    std::shared_ptr<const PointCacheEntry> entry;
    std::size_t bytes = 0;
    uint64_t last_used = 0;
  };

  static constexpr std::size_t kMaximumPointCacheEntries = 16;
  static std::shared_ptr<State> MakeState(VisualizationSource source,
                                          VisualizationPhase phase,
                                          bool include_maps);
  void ClearPointCacheLocked() const;
  void ClearPointCacheForAgentLocked(const AgentId& agent) const;

  mutable std::mutex mutex_;
  std::shared_ptr<const State> state_ = std::make_shared<State>();
  std::optional<uint64_t> candidate_base_revision_;
  std::shared_ptr<const State> candidate_rollback_state_;
  mutable std::map<PointCacheKey, CachedPointEntry> point_cache_;
  mutable std::size_t point_cache_bytes_ = 0;
  mutable uint64_t point_cache_access_ = 0;
  mutable uint64_t point_cache_hits_ = 0;
  mutable uint64_t point_cache_misses_ = 0;
  mutable uint64_t point_cache_insertions_ = 0;
  mutable uint64_t point_cache_evictions_ = 0;
  mutable uint64_t point_cache_clears_ = 0;
};

}  // namespace open_lmm
