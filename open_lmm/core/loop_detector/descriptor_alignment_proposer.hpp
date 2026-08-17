#pragma once

#include <open_lmm/common/agent_data.hpp>

#include <map>
#include <optional>

namespace open_lmm {

class DescriptorAlignmentProposer {
 public:
  DescriptorAlignmentProposer(double pcm_translation_threshold,
                              double pcm_rotation_threshold_rad);

  std::optional<MapAlignmentProposal> Propose(
      char target_agent, char source_agent,
      const PoseVec& source_odometry,
      const std::map<char, AgentOptimizedData>& optimized_agents,
      const LoopPairVec& descriptor_loops,
      const std::vector<Eigen::Vector3f>& target_map,
      const std::vector<Eigen::Vector3f>& source_map) const;

 private:
  double pcm_translation_threshold_;
  double pcm_rotation_threshold_rad_;
};

}  // namespace open_lmm
