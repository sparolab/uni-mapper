#pragma once

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>

#include <Eigen/Core>

#include <map>
#include <optional>
#include <vector>

namespace open_lmm {

// Keeps accepted alignment transforms alive independently of the review
// broker/model lifetime. A delayed odometry snapshot may redraw a trajectory
// after Apply; the accepted transform remains authoritative for presentation
// until an optimized snapshot for that agent is actually installed.
class AlignmentPresentationState {
 public:
  void RememberAccepted(const AgentId& agent,
                        const Eigen::Matrix4f& transform) {
    if (!agent.IsValid()) return;
    accepted_transforms_[agent] = transform;
  }

  [[nodiscard]] std::optional<Eigen::Matrix4f> TransformForOdometry(
      const AgentId& agent) const {
    const auto found = accepted_transforms_.find(agent);
    return found == accepted_transforms_.end()
               ? std::nullopt
               : std::optional<Eigen::Matrix4f>(found->second);
  }

  void ObserveSnapshot(const AgentId& agent, VisualizationPoseKind pose_kind) {
    if (pose_kind == VisualizationPoseKind::kOptimized) {
      accepted_transforms_.erase(agent);
    }
  }

  [[nodiscard]] std::vector<AgentId> Agents() const {
    std::vector<AgentId> agents;
    agents.reserve(accepted_transforms_.size());
    for (const auto& [agent, transform] : accepted_transforms_) {
      (void)transform;
      agents.push_back(agent);
    }
    return agents;
  }

  void Clear() noexcept { accepted_transforms_.clear(); }

 private:
  std::map<AgentId, Eigen::Matrix4f> accepted_transforms_;
};

}  // namespace open_lmm
