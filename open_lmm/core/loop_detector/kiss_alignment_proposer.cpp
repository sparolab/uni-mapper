#include "kiss_alignment_proposer.hpp"
#include "map_alignment_refiner.hpp"

#include <kiss_matcher/KISSMatcher.hpp>

#include <open_lmm/utils/logging.hpp>
#include <sstream>

namespace open_lmm {

std::optional<MapAlignmentProposal> KissAlignmentProposer::Propose(
    const std::vector<Eigen::Vector3f>& target_map,
    const std::vector<Eigen::Vector3f>& source_map,
    AgentId target_agent, AgentId source_agent,
    float leaf_size, bool use_quatro) const {
  kiss_matcher::KISSMatcherConfig config(leaf_size);
  config.use_quatro_ = use_quatro;
  kiss_matcher::KISSMatcher matcher(config);
  const auto solution = matcher.estimate(source_map, target_map);

  const std::size_t rotation_inliers = matcher.getNumRotationInliers();
  const std::size_t final_inliers = matcher.getNumFinalInliers();
  constexpr std::size_t kMinimumFinalInliers = 5;
  if (final_inliers < kMinimumFinalInliers) {
    std::ostringstream message;
    message << "[alignment] KISS-Matcher rejected agents " << source_agent
            << "->" << target_agent << ": final_inliers=" << final_inliers
            << " minimum=" << kMinimumFinalInliers;
    LogWarning(message.str());
    return std::nullopt;
  }
  std::ostringstream message;
  message << "[alignment] KISS-Matcher accepted agents " << source_agent
          << "->" << target_agent << ": rotation_inliers="
          << rotation_inliers << " final_inliers=" << final_inliers;
  LogInfo(message.str());

  MapAlignmentProposal proposal;
  proposal.target_agent = target_agent;
  proposal.source_agent = source_agent;
  proposal.method = AlignmentMethod::kKissMatcher;
  proposal.target_T_source.linear() = solution.rotation;
  proposal.target_T_source.translation() = solution.translation;
  proposal.metrics.rotation_inliers = rotation_inliers;
  proposal.metrics.final_inliers = final_inliers;
  const auto quality = RefineMapAlignment(
      target_map, source_map, proposal.target_T_source, false);
  proposal.metrics.correspondence_count = quality.correspondence_count;
  proposal.metrics.fitness = quality.fitness;
  proposal.metrics.overlap_ratio = quality.overlap_ratio;
  return proposal;
}

}  // namespace open_lmm
