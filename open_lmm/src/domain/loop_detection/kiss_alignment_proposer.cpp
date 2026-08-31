#include "kiss_alignment_proposer.hpp"
#include "map_alignment_refiner.hpp"

#include <kiss_matcher/KISSMatcher.hpp>

#include <foundation/logging/logging.hpp>
#include <sstream>
#include <utility>

namespace open_lmm {
namespace {

KissMatcherEstimate RunKissMatcher(
    const std::vector<Eigen::Vector3f>& source_map,
    const std::vector<Eigen::Vector3f>& target_map, float leaf_size,
    bool use_quatro) {
  kiss_matcher::KISSMatcherConfig config(leaf_size);
  config.use_quatro_ = use_quatro;
  kiss_matcher::KISSMatcher matcher(config);
  const auto solution = matcher.estimate(source_map, target_map);
  return {solution.rotation, solution.translation,
          matcher.getNumRotationInliers(), matcher.getNumFinalInliers()};
}

}  // namespace

KissAlignmentProposer::KissAlignmentProposer(MatcherRunner runner)
    : runner_(runner ? std::move(runner) : MatcherRunner(RunKissMatcher)) {}

std::optional<MapAlignmentProposal> KissAlignmentProposer::Propose(
    const std::vector<Eigen::Vector3f>& target_map,
    const std::vector<Eigen::Vector3f>& source_map,
    AgentId target_agent, AgentId source_agent,
    float leaf_size, bool use_quatro) const {
  const KissMatcherEstimate solution =
      runner_(source_map, target_map, leaf_size, use_quatro);
  const std::size_t rotation_inliers = solution.rotation_inliers;
  const std::size_t final_inliers = solution.final_inliers;
  constexpr std::size_t kMinimumFinalInliers = 5;
  if (final_inliers < kMinimumFinalInliers) {
    std::ostringstream message;
    message << "[alignment] KISS-Matcher rejected agents " << source_agent
            << "->" << target_agent << ": final_inliers=" << final_inliers
            << " minimum=" << kMinimumFinalInliers
            << " voxel_size_m=" << leaf_size
            << " source_points=" << source_map.size()
            << " target_points=" << target_map.size();
    LogWarning(message.str());
    return std::nullopt;
  }
  std::ostringstream message;
  message << "[alignment] KISS-Matcher accepted agents " << source_agent
          << "->" << target_agent << ": rotation_inliers="
          << rotation_inliers << " final_inliers=" << final_inliers
          << " voxel_size_m=" << leaf_size
          << " source_points=" << source_map.size()
          << " target_points=" << target_map.size();
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
