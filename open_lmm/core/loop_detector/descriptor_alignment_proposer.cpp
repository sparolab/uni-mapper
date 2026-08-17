#include "descriptor_alignment_proposer.hpp"
#include "map_alignment_refiner.hpp"

#include <gkcm/gkcm.hpp>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace open_lmm {
namespace {

struct RelativePoseMeasurement {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  double translation_threshold = 0.0;
  double rotation_threshold_rad = 0.0;
};

using GkcmMeasurement = GkCM::Measurement<RelativePoseMeasurement>;
using GkcmCombination = GkCM::Combination<GkcmMeasurement, 2>;

struct RelativePoseConsistencyFunctor {
  bool operator()(const GkcmCombination& combination) const {
    const auto lhs = combination.get_meas(0).get_value();
    const auto rhs = combination.get_meas(1).get_value();
    const auto& lhs_transform = lhs.transform;
    const auto& rhs_transform = rhs.transform;
    const Eigen::Isometry3d delta = lhs_transform.inverse() * rhs_transform;
    const double cosine = std::clamp(
        (delta.linear().trace() - 1.0) * 0.5, -1.0, 1.0);
    return delta.translation().norm() <= lhs.translation_threshold &&
           std::acos(cosine) <= lhs.rotation_threshold_rad;
  }
};

using RelativePoseConsistencyEvaluator =
    GkCM::ConsistencyEvaluator<GkcmMeasurement, GkcmCombination,
                               RelativePoseConsistencyFunctor>;

std::vector<std::size_t> SelectPairwiseConsistentInliers(
    const std::vector<Eigen::Isometry3d>& candidates,
    double translation_threshold, double rotation_threshold_rad) {
  std::vector<GkcmMeasurement> measurements;
  measurements.reserve(candidates.size());
  for (const auto& candidate : candidates) {
    measurements.emplace_back(RelativePoseMeasurement{
        candidate, translation_threshold, rotation_threshold_rad});
  }

  GMC::MaxCliqueSolverExact<int> maximum_clique_solver(1, 1);
  RelativePoseConsistencyEvaluator evaluator;
  GkCM::GkCMSolver<2, GkcmMeasurement, RelativePoseConsistencyEvaluator>
      gkcm(maximum_clique_solver, evaluator);
  gkcm.add_measurements(std::move(measurements));
  const auto cliques = gkcm.solve();
  if (cliques.empty()) return {};
  const auto largest = std::max_element(
      cliques.begin(), cliques.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() < rhs.size();
      });
  std::vector<std::size_t> indices;
  indices.reserve(largest->size());
  for (const int index : *largest) {
    if (index >= 0) indices.push_back(static_cast<std::size_t>(index));
  }
  return indices;
}

}  // namespace

DescriptorAlignmentProposer::DescriptorAlignmentProposer(
    double pcm_translation_threshold, double pcm_rotation_threshold_rad)
    : pcm_translation_threshold_(pcm_translation_threshold),
      pcm_rotation_threshold_rad_(pcm_rotation_threshold_rad) {
  if (pcm_translation_threshold_ <= 0.0 ||
      pcm_rotation_threshold_rad_ <= 0.0) {
    throw std::invalid_argument("PCM consistency thresholds must be positive");
  }
}

std::optional<MapAlignmentProposal> DescriptorAlignmentProposer::Propose(
    char target_agent, char source_agent,
    const PoseVec& source_odometry,
    const std::map<char, AgentOptimizedData>& optimized_agents,
    const LoopPairVec& descriptor_loops,
    const std::vector<Eigen::Vector3f>& target_map,
    const std::vector<Eigen::Vector3f>& source_map) const {
  std::vector<Eigen::Isometry3d> candidates;
  candidates.reserve(descriptor_loops.size());
  for (const auto& loop : descriptor_loops) {
    if (loop.from.second >= source_odometry.size()) continue;
    const auto optimized = optimized_agents.find(loop.to.first);
    if (optimized == optimized_agents.end()) continue;
    const auto pose = std::find_if(
        optimized->second.optimized_poses.begin(),
        optimized->second.optimized_poses.end(),
        [&loop](const auto& value) {
          return value.first == static_cast<int>(loop.to.second);
        });
    if (pose == optimized->second.optimized_poses.end()) continue;
    candidates.push_back(pose->second * loop.init_rel_pose *
                         source_odometry[loop.from.second].inverse());
  }
  if (candidates.empty()) return std::nullopt;

  const auto inliers = SelectPairwiseConsistentInliers(
      candidates, pcm_translation_threshold_, pcm_rotation_threshold_rad_);
  if (inliers.empty()) return std::nullopt;

  Eigen::Vector3d translation_sum = Eigen::Vector3d::Zero();
  Eigen::Vector4d quaternion_sum = Eigen::Vector4d::Zero();
  const Eigen::Quaterniond reference(candidates[inliers.front()].linear());
  for (const auto index : inliers) {
    const auto& candidate = candidates[index];
    translation_sum += candidate.translation();
    Eigen::Quaterniond quaternion(candidate.linear());
    if (reference.dot(quaternion) < 0.0) quaternion.coeffs() *= -1.0;
    quaternion_sum += quaternion.coeffs();
  }
  const std::size_t consensus_size = inliers.size();

  Eigen::Quaterniond mean_rotation;
  mean_rotation.coeffs() = quaternion_sum.normalized();
  MapAlignmentProposal proposal;
  proposal.target_agent = target_agent;
  proposal.source_agent = source_agent;
  proposal.method = AlignmentMethod::kDescriptor;
  proposal.target_T_source.linear() =
      mean_rotation.normalized().toRotationMatrix();
  proposal.target_T_source.translation() =
      translation_sum / static_cast<double>(consensus_size);
  proposal.metrics.consensus_size = consensus_size;
  const auto refinement = RefineMapAlignment(
      target_map, source_map, proposal.target_T_source, true);
  proposal.target_T_source = refinement.target_T_source;
  proposal.metrics.correspondence_count = refinement.correspondence_count;
  proposal.metrics.fitness = refinement.fitness;
  proposal.metrics.overlap_ratio = refinement.overlap_ratio;
  return proposal;
}

}  // namespace open_lmm
