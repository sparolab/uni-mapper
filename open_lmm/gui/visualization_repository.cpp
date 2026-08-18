#include <open_lmm/gui/visualization_repository.hpp>

#include <algorithm>

namespace open_lmm {
namespace {
std::string RevisionName(const std::string& prefix, uint64_t revision) {
  return prefix + "/" + std::to_string(revision);
}
std::shared_ptr<const VisualizationSnapshot> MetadataOnly(
    const VisualizationSnapshot& source) {
  auto snapshot = std::make_shared<VisualizationSnapshot>();
  snapshot->agent = source.agent;
  snapshot->revision = source.revision;
  snapshot->poses = source.poses;
  snapshot->edges = source.edges;
  snapshot->min_bound = source.min_bound;
  snapshot->max_bound = source.max_bound;
  snapshot->has_bounds = source.has_bounds;
  snapshot->map_available = source.map_available;
  return snapshot;
}
}  // namespace

VisualizationUpdate VisualizationRepository::Commit(
    std::shared_ptr<const VisualizationSnapshot> snapshot) {
  VisualizationUpdate update;
  if (!snapshot || !snapshot->agent.IsValid()) return update;
  const auto found = snapshots_.find(snapshot->agent);
  if (found != snapshots_.end()) {
    if (found->second->revision >= snapshot->revision) return update;
    update.remove_drawables = {
        MapName(snapshot->agent, found->second->revision),
        TrajectoryName(snapshot->agent, found->second->revision),
        IntraLoopName(snapshot->agent, found->second->revision),
        InterLoopName(snapshot->agent, found->second->revision),
    };
    for (std::size_t i = 0; i < found->second->poses.size(); ++i) {
      update.remove_drawables.push_back(
          PoseName(snapshot->agent, i, found->second->revision));
    }
  }
  const AgentId agent = snapshot->agent;
  snapshots_[agent] = MetadataOnly(*snapshot);
  const auto& current = snapshots_.at(agent);
  update.add_drawables = {
      MapName(current->agent, current->revision),
      TrajectoryName(current->agent, current->revision),
      IntraLoopName(current->agent, current->revision),
      InterLoopName(current->agent, current->revision),
  };
  update.changed = true;
  return update;
}

std::shared_ptr<const VisualizationSnapshot> VisualizationRepository::Latest(
    const AgentId& agent) const {
  const auto found = snapshots_.find(agent);
  return found == snapshots_.end() ? nullptr : found->second;
}

std::vector<std::shared_ptr<const VisualizationSnapshot>>
VisualizationRepository::Snapshots() const {
  std::vector<std::shared_ptr<const VisualizationSnapshot>> result;
  result.reserve(snapshots_.size());
  for (const auto& [agent, snapshot] : snapshots_) {
    (void)agent;
    result.push_back(snapshot);
  }
  return result;
}

std::size_t VisualizationRepository::ApproximateBytes() const {
  std::size_t bytes = 0;
  for (const auto& [agent, snapshot] : snapshots_) {
    (void)agent;
    bytes += sizeof(VisualizationSnapshot);
    bytes += snapshot->poses.size() * sizeof(VisualizationPose);
    bytes += snapshot->edges.size() * sizeof(VisualizationEdge);
    bytes += snapshot->points.size() * sizeof(VisualizationPoint);
  }
  return bytes;
}

std::string VisualizationRepository::MapName(const AgentId& agent, uint64_t revision) {
  return RevisionName("agent/" + agent.Value() + "/map", revision);
}

std::string VisualizationRepository::TrajectoryName(const AgentId& agent,
                                                    uint64_t revision) {
  return RevisionName("agent/" + agent.Value() + "/trajectory", revision);
}

std::string VisualizationRepository::PoseName(const AgentId& agent,
                                              std::size_t pose_index,
                                              uint64_t revision) {
  return RevisionName("agent/" + agent.Value() + "/pose/" +
                          std::to_string(pose_index),
                      revision);
}

std::string VisualizationRepository::IntraLoopName(const AgentId& agent,
                                                   uint64_t revision) {
  return RevisionName("loops/intra/" + agent.Value(), revision);
}

std::string VisualizationRepository::InterLoopName(const AgentId& agent,
                                                   uint64_t revision) {
  return RevisionName("loops/inter/" + agent.Value(), revision);
}

}  // namespace open_lmm
