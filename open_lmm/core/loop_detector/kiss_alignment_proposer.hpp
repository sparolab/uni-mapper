#pragma once

#include <open_lmm/common/alignment_types.hpp>

#include <Eigen/Core>
#include <functional>
#include <optional>
#include <vector>

namespace open_lmm {

struct KissMatcherEstimate {
  Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  std::size_t rotation_inliers = 0;
  std::size_t final_inliers = 0;
};

class KissAlignmentProposer {
 public:
  using MatcherRunner = std::function<KissMatcherEstimate(
      const std::vector<Eigen::Vector3f>& source_map,
      const std::vector<Eigen::Vector3f>& target_map, float leaf_size,
      bool use_quatro)>;

  explicit KissAlignmentProposer(MatcherRunner runner = {});

  std::optional<MapAlignmentProposal> Propose(
      const std::vector<Eigen::Vector3f>& target_map,
      const std::vector<Eigen::Vector3f>& source_map,
      AgentId target_agent, AgentId source_agent,
      float leaf_size, bool use_quatro = false) const;

 private:
  MatcherRunner runner_;
};

}  // namespace open_lmm
