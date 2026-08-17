#include "descriptor_alignment_proposer.hpp"

#include <gkcm/gkcm.hpp>

#include <open_lmm/utils/logging.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <numeric>
#include <queue>
#include <sstream>
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
    const DescriptorAlignmentOptions& options) {
  std::vector<std::size_t> selected_indices;
  const std::size_t selected_count = options.max_candidates == 0
      ? candidates.size()
      : std::min(candidates.size(), options.max_candidates);
  selected_indices.reserve(selected_count);
  if (selected_count == candidates.size()) {
    selected_indices.resize(candidates.size());
    std::iota(selected_indices.begin(), selected_indices.end(), 0);
  } else {
    // Preserve coverage across the complete trajectory instead of keeping only
    // its first candidates. Both endpoints are included.
    for (std::size_t i = 0; i < selected_count; ++i) {
      selected_indices.push_back(selected_count == 1
          ? 0
          : i * (candidates.size() - 1) / (selected_count - 1));
    }
  }

  std::vector<GkcmMeasurement> measurements;
  measurements.reserve(selected_indices.size());
  for (const auto index : selected_indices) {
    measurements.emplace_back(RelativePoseMeasurement{
        candidates[index], options.pcm_translation_threshold,
        options.pcm_rotation_threshold_rad});
  }

  std::unique_ptr<GMC::GeneralizedMaxCliqueSolver<int>> maximum_clique_solver;
  if (options.solver == "exact") {
    maximum_clique_solver = std::make_unique<GMC::MaxCliqueSolverExact<int>>(
        1, options.threads);
  } else {
    maximum_clique_solver =
        std::make_unique<GMC::MaxCliqueSolverHeuristic<int>>(
            1, options.threads);
  }
  RelativePoseConsistencyEvaluator evaluator;
  GkCM::GkCMSolver<2, GkcmMeasurement, RelativePoseConsistencyEvaluator>
      gkcm(*maximum_clique_solver, evaluator);

  std::vector<std::vector<std::size_t>> adjacency(selected_count);
  RelativePoseConsistencyFunctor consistent;
  for (std::size_t i = 0; i < selected_count; ++i) {
    for (std::size_t j = i + 1; j < selected_count; ++j) {
      GkcmCombination pair({static_cast<int>(i), static_cast<int>(j)},
                           measurements);
      if (consistent(pair)) {
        adjacency[i].push_back(j);
        adjacency[j].push_back(i);
      }
    }
  }
  std::size_t component_count = 0;
  std::size_t largest_component = 0;
  std::vector<bool> visited(selected_count, false);
  for (std::size_t root = 0; root < selected_count; ++root) {
    if (visited[root]) continue;
    ++component_count;
    std::size_t component_size = 0;
    std::queue<std::size_t> pending;
    pending.push(root);
    visited[root] = true;
    while (!pending.empty()) {
      const auto node = pending.front();
      pending.pop();
      ++component_size;
      for (const auto neighbor : adjacency[node]) {
        if (!visited[neighbor]) {
          visited[neighbor] = true;
          pending.push(neighbor);
        }
      }
    }
    largest_component = std::max(largest_component, component_size);
  }
  std::size_t min_degree = selected_count == 0 ? 0 : selected_count;
  std::size_t max_degree = 0;
  std::size_t degree_sum = 0;
  for (const auto& neighbors : adjacency) {
    min_degree = std::min(min_degree, neighbors.size());
    max_degree = std::max(max_degree, neighbors.size());
    degree_sum += neighbors.size();
  }

  const auto graph_start = std::chrono::steady_clock::now();
  gkcm.add_measurements(std::move(measurements));
  const auto clique_start = std::chrono::steady_clock::now();
  const auto cliques = gkcm.solve();
  const auto clique_end = std::chrono::steady_clock::now();
  const auto graph_ms = std::chrono::duration<double, std::milli>(
                            clique_start - graph_start)
                            .count();
  const auto clique_ms = std::chrono::duration<double, std::milli>(
                             clique_end - clique_start)
                             .count();
  if (cliques.empty()) {
    LogWarning("[gkcm] solver returned no clique");
    return {};
  }
  const auto largest = std::max_element(
      cliques.begin(), cliques.end(), [](const auto& lhs, const auto& rhs) {
        return lhs.size() < rhs.size();
      });
  std::vector<std::size_t> indices;
  indices.reserve(largest->size());
  for (const int index : *largest) {
    if (index >= 0 && static_cast<std::size_t>(index) < selected_indices.size()) {
      indices.push_back(selected_indices[static_cast<std::size_t>(index)]);
    }
  }
  std::ostringstream message;
  message << "[gkcm] solver=" << options.solver
          << " threads=" << options.threads
          << " candidates=" << candidates.size()
          << " selected=" << selected_indices.size()
          << " cap=" << options.max_candidates
          << " components=" << component_count
          << " largest_component=" << largest_component
          << " degree_min=" << min_degree
          << " degree_avg="
          << (selected_count == 0 ? 0.0
                                  : static_cast<double>(degree_sum) /
                                        static_cast<double>(selected_count))
          << " degree_max=" << max_degree
          << " graph_ms=" << graph_ms << " clique_ms=" << clique_ms
          << " consensus=" << indices.size();
  LogInfo(message.str());
  return indices;
}

}  // namespace

DescriptorAlignmentProposer::DescriptorAlignmentProposer(
    double pcm_translation_threshold, double pcm_rotation_threshold_rad)
    : DescriptorAlignmentProposer(DescriptorAlignmentOptions{
          pcm_translation_threshold, pcm_rotation_threshold_rad}) {}

DescriptorAlignmentProposer::DescriptorAlignmentProposer(
    DescriptorAlignmentOptions options)
    : options_(std::move(options)) {
  if (options_.pcm_translation_threshold <= 0.0 ||
      options_.pcm_rotation_threshold_rad <= 0.0 ||
      (options_.solver != "heuristic" && options_.solver != "exact") ||
      options_.threads <= 0) {
    throw std::invalid_argument("PCM consistency thresholds must be positive");
  }
}

std::optional<MapAlignmentProposal> DescriptorAlignmentProposer::Propose(
    char target_agent, char source_agent,
    const PoseVec& source_odometry,
    const std::map<char, AgentOptimizedData>& optimized_agents,
    const LoopPairVec& descriptor_loops,
    DescriptorAlignmentDiagnostics* diagnostics) const {
  const auto candidate_start = std::chrono::steady_clock::now();
  std::vector<Eigen::Isometry3d> candidates;
  std::vector<std::size_t> candidate_loop_indices;
  candidates.reserve(descriptor_loops.size());
  candidate_loop_indices.reserve(descriptor_loops.size());
  for (std::size_t loop_index = 0; loop_index < descriptor_loops.size();
       ++loop_index) {
    const auto& loop = descriptor_loops[loop_index];
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
    candidate_loop_indices.push_back(loop_index);
  }
  const auto candidate_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() -
                                candidate_start)
                                .count();
  {
    std::ostringstream message;
    message << "[descriptor] loop_candidates=" << descriptor_loops.size()
            << " valid_candidates=" << candidates.size()
            << " candidate_ms=" << candidate_ms;
    LogInfo(message.str());
  }
  if (candidates.empty()) return std::nullopt;

  const auto inliers = SelectPairwiseConsistentInliers(
      candidates, options_);
  if (inliers.empty()) return std::nullopt;
  if (diagnostics) {
    diagnostics->inlier_loop_indices.clear();
    diagnostics->inlier_loop_indices.reserve(inliers.size());
    for (const auto candidate_index : inliers) {
      diagnostics->inlier_loop_indices.push_back(
          candidate_loop_indices[candidate_index]);
    }
  }

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
  const Eigen::AngleAxisd pose_rotation(proposal.target_T_source.linear());
  std::ostringstream pose_message;
  pose_message << "[descriptor] consensus=" << consensus_size
               << " pose_t=" << proposal.target_T_source.translation().transpose()
               << " pose_rotation_deg=" << pose_rotation.angle() * 180.0 / M_PI
               << " pose_rotation_axis=" << pose_rotation.axis().transpose();
  LogInfo(pose_message.str());
  return proposal;
}

}  // namespace open_lmm
