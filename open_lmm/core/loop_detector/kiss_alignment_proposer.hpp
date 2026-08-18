#pragma once

#include <open_lmm/common/alignment_types.hpp>

#include <Eigen/Core>
#include <optional>
#include <vector>

namespace open_lmm {

class KissAlignmentProposer {
 public:
  std::optional<MapAlignmentProposal> Propose(
      const std::vector<Eigen::Vector3f>& target_map,
      const std::vector<Eigen::Vector3f>& source_map,
      AgentId target_agent, AgentId source_agent,
      float leaf_size, bool use_quatro = false) const;
};

}  // namespace open_lmm
