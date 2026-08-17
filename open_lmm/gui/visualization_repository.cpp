#include <open_lmm/gui/visualization_repository.hpp>

#include <algorithm>

namespace open_lmm {
namespace {
std::string RevisionName(const std::string& prefix, uint64_t revision) {
  return prefix + "/" + std::to_string(revision);
}
}  // namespace

VisualizationUpdate VisualizationRepository::Commit(
    std::shared_ptr<const VisualizationSnapshot> snapshot) {
  VisualizationUpdate update;
  if (!snapshot || snapshot->agent == 0) return update;
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
  const char agent = snapshot->agent;
  snapshots_[agent] = std::move(snapshot);
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
    char agent) const {
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

std::string VisualizationRepository::MapName(char agent, uint64_t revision) {
  return RevisionName("agent/" + std::string{agent} + "/map", revision);
}

std::string VisualizationRepository::TrajectoryName(char agent,
                                                    uint64_t revision) {
  return RevisionName("agent/" + std::string{agent} + "/trajectory", revision);
}

std::string VisualizationRepository::PoseName(char agent,
                                              std::size_t pose_index,
                                              uint64_t revision) {
  return RevisionName("agent/" + std::string{agent} + "/pose/" +
                          std::to_string(pose_index),
                      revision);
}

std::string VisualizationRepository::IntraLoopName(char agent,
                                                   uint64_t revision) {
  return RevisionName("loops/intra/" + std::string{agent}, revision);
}

std::string VisualizationRepository::InterLoopName(char agent,
                                                   uint64_t revision) {
  return RevisionName("loops/inter/" + std::string{agent}, revision);
}

}  // namespace open_lmm
