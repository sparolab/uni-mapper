#include "loop_detector_kdtree.hpp"
#include "descriptor_alignment_proposer.hpp"
#include "kiss_alignment_proposer.hpp"
#include "map_alignment_coordinator.hpp"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/common/rigid_transform.hpp>
#include <open_lmm/common/validation.hpp>
#include <open_lmm/utils/logging.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

namespace open_lmm {

namespace {
DescriptorAlignmentOptions DescriptorOptions(const KdtreeParams& params) {
  return DescriptorAlignmentOptions{
      params.pcm_translation_threshold,
      params.pcm_rotation_threshold_deg * M_PI / 180.0,
      params.pcm_solver,
      params.pcm_threads,
      params.pcm_max_candidates};
}

void LogProposalDifference(
    const std::optional<MapAlignmentProposal>& kiss,
    const std::optional<MapAlignmentProposal>& descriptor) {
  if (!kiss || !descriptor) return;
  const Eigen::Isometry3d delta =
      kiss->target_T_source.inverse() * descriptor->target_T_source;
  const double cosine = std::clamp(
      (delta.linear().trace() - 1.0) * 0.5, -1.0, 1.0);
  std::ostringstream message;
  message << "[alignment] descriptor_vs_kiss translation_diff_m="
          << delta.translation().norm()
          << " rotation_diff_deg=" << std::acos(cosine) * 180.0 / M_PI;
  LogInfo(message.str());
}
}  // namespace

LoopDetectorKdtree::LoopDetectorKdtree(
    const KdtreeParams& params,
    std::shared_ptr<IDescriptorKdtree> model_descriptor)
    : params_(params), model_descriptor_(std::move(model_descriptor)) {
  database_.emplace(DatabaseKdtreeParams{
      static_cast<std::size_t>(model_descriptor_->getDescriptorKey().size()),
      params_.num_candidates, params_.distance_threshold,
      params_.kdtree_rebuild_threshold}, model_descriptor_);
}

LoopPair LoopDetectorKdtree::createLoopPair(
    AgentId agent_id, size_t current_idx,
    const LoopCandidateInfo& candidate_info) {
  LoopPair loop;
  auto [db_id, key, init_rel_pose] = candidate_info;
  loop.to = std::make_pair(db_id, key);
  loop.from = std::make_pair(agent_id, current_idx);
  loop.init_rel_pose = init_rel_pose;
  return loop;
}

std::vector<LoopPair> LoopDetectorKdtree::detectIntraLoops(
    const ScanVec& scans, const AgentContext& agent_ctx) {
  OPEN_LMM_ZONE_N("LoopDetector.IntraQuery");
  std::vector<LoopPair> intra_loop_pairs;
  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Intra Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);
    std::optional<LoopCandidateInfo> intra_loop_candidates =
        database_->query(descriptor);

    if (intra_loop_candidates != std::nullopt) {
      intra_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, intra_loop_candidates.value()));
    }

    database_->insert(agent_ctx.id, idx, descriptor);
  }
  T.finish();
  return intra_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectInterLoops(
    const ScanVec& scans, const DescriptorStore& descriptor_store,
    const AgentContext& agent_ctx) {
  OPEN_LMM_ZONE_N("LoopDetector.InterQuery");
  std::vector<LoopPair> inter_loop_pairs;

  if (agent_ctx.is_anchor()) {
    return inter_loop_pairs;
  }

  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Inter Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);

    if (!descriptor_store.total_db) continue;
    std::optional<LoopCandidateInfo> inter_loop_candidates =
        descriptor_store.total_db->query(descriptor);

    if (inter_loop_candidates != std::nullopt) {
      inter_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, inter_loop_candidates.value()));
    }
  }
  T.finish();
  return inter_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::findLoopPairsFromKdTree(
    const AgentOptimizedDataMap& all_optimized,
    const AgentRawDataMap& all_raw_data,
    const std::vector<Eigen::Isometry3f>& transformed_poses,
    const AgentContext& agent_ctx,
    float distance_threshold) {
  OPEN_LMM_ZONE_N("LoopDetector.PoseKdTreeQuery");
  std::vector<LoopPair> loop_pairs;

  if (transformed_poses.empty()) return loop_pairs;

  for (const auto& [db_id, opt_data] : all_optimized) {
    const auto raw_it = all_raw_data.find(db_id);
    if (raw_it == all_raw_data.end() || opt_data->kdtree_poses.empty() ||
        raw_it->second->odom_poses.empty()) {
      continue;
    }
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = opt_data->kdtree_poses;
    kdtree.setInputCloud(cloud);

    Eigen::Vector3f prev_pose = transformed_poses[0].translation();

    for (size_t idx = 0; idx < transformed_poses.size(); idx++) {
      auto pose = transformed_poses[idx];

      Eigen::Vector3f curr_pose = pose.translation();
      float distance = (curr_pose - prev_pose).norm();
      if (distance < 10.0F) {
        continue;
      } else {
        prev_pose = curr_pose;
      }

      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      pcl::PointXYZ src_point(pose.translation().x(), pose.translation().y(),
                              pose.translation().z());
      const int found = kdtree.nearestKSearch(
          src_point, 1, pointIdxNKNSearch, pointNKNSquaredDistance);
      auto neighbor_result = ValidateNearestNeighborResult(
          found, pointIdxNKNSearch, opt_data->optimized_poses.size(),
          "LoopDetectorKdtree");
      if (!neighbor_result || pointNKNSquaredDistance.empty()) continue;

      if (std::sqrt(pointNKNSquaredDistance[0]) < distance_threshold) {
        const auto neighbor_index = neighbor_result.Value();
        const auto& [target_scan_index, target_global_pose] =
            opt_data->optimized_poses[neighbor_index];
        if (target_scan_index < 0 ||
            static_cast<std::size_t>(target_scan_index) >=
                raw_it->second->odom_poses.size()) {
          continue;
        }
        // Registration expects target_scan_T_source_scan so that it can move
        // the source cloud into the target submap frame. BetweenFactor later
        // stores the inverse (source_T_target).
        const Eigen::Isometry3d init_rel_pose =
            TargetFromSourceScanTransform(target_global_pose,
                                          pose.cast<double>());

        LoopPair inter_loop;
        inter_loop.to = std::make_pair(
            db_id, static_cast<size_t>(target_scan_index));
        inter_loop.from = std::make_pair(agent_ctx.id, idx);
        inter_loop.init_rel_pose = init_rel_pose;
        loop_pairs.push_back(inter_loop);
      }
    }
  }

  return loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectKissMatcherLoops(
    const AlgorithmExecutionContext& context,
    const LoopDetectorProcessInput& input,
    std::vector<Eigen::Vector3f>& out_transformed_map_points,
    std::optional<MapAlignmentProposal>& out_proposal) {
  OPEN_LMM_ZONE_N("LoopDetector.KissMatcher");
  std::vector<LoopPair> additional_loops;
  // anchor: 맵 포인트를 out으로 전달 (caller가 descriptor_store에 set_anchor 호출)
  if (context.agent.is_anchor()) {
    out_transformed_map_points = input.current.map_points;
    return additional_loops;
  }

  const AgentId target_agent = input.all_optimized.empty()
                                ? AgentId{}
                                : input.all_optimized.begin()->first;
  out_proposal = KissAlignmentProposer().Propose(
      input.descriptor_store.merged_map, input.current.map_points,
      target_agent, context.agent.id, params_.kiss_voxel_size,
      params_.kiss_use_quatro);
  if (!out_proposal) {
    return additional_loops;
  }

  const Eigen::Matrix4f relative_map_pose =
      out_proposal->target_T_source.matrix().cast<float>();

  auto transformed_poses = transformEigenPoses(
      input.current.odom_poses, relative_map_pose);

  additional_loops = findLoopPairsFromKdTree(
      input.all_optimized, input.all_raw_data,
      transformed_poses, context.agent, params_.pose_nn_distance_threshold);

  out_transformed_map_points = transformEigenPoints(
      input.current.map_points, relative_map_pose);

  return additional_loops;
}

std::vector<LoopPair> LoopDetectorKdtree::loopsFromGlobalTransform(
    const AlgorithmExecutionContext& context,
    const LoopDetectorProcessInput& input,
    const Eigen::Isometry3d& target_T_source) {
  auto transformed_poses = transformEigenPoses(
      input.current.odom_poses, target_T_source.matrix().cast<float>());
  return findLoopPairsFromKdTree(input.all_optimized, input.all_raw_data,
                                 transformed_poses, context.agent,
                                 params_.pose_nn_distance_threshold);
}

namespace {
std::vector<AlignmentVisualizationPoint> AlignmentPoints(
    const std::vector<Eigen::Vector3f>& points,
    std::size_t max_points = 200000) {
  std::vector<AlignmentVisualizationPoint> output;
  if (points.empty()) return output;
  const std::size_t stride = std::max<std::size_t>(1, points.size() / max_points);
  output.reserve(std::min(points.size(), max_points));
  for (std::size_t i = 0; i < points.size(); i += stride) {
    output.push_back({points[i].x(), points[i].y(), points[i].z()});
    if (output.size() == max_points) break;
  }
  return output;
}

AlignmentVisualizationPoint VisualizationPoint(
    const Eigen::Vector3d& point) {
  return {static_cast<float>(point.x()), static_cast<float>(point.y()),
          static_cast<float>(point.z())};
}

AlignmentVisualizationData DescriptorVisualization(
    const AlgorithmExecutionContext& context,
    const LoopDetectorProcessInput& input, const LoopPairVec& loops,
    const DescriptorAlignmentDiagnostics& diagnostics,
    const Eigen::Isometry3d& target_T_source, const AgentId& target_agent) {
  AlignmentVisualizationData output;
  const auto target = input.all_optimized.find(target_agent);
  if (target != input.all_optimized.end()) {
    output.target_trajectory.reserve(target->second->optimized_poses.size());
    for (const auto& [index, pose] : target->second->optimized_poses) {
      (void)index;
      output.target_trajectory.push_back(VisualizationPoint(pose.translation()));
    }
  }
  output.source_trajectory.reserve(input.current.odom_poses.size());
  for (const auto& pose : input.current.odom_poses) {
    output.source_trajectory.push_back(
        VisualizationPoint((target_T_source * pose).translation()));
  }

  output.descriptor_loops.reserve(loops.size());
  for (std::size_t loop_index = 0; loop_index < loops.size(); ++loop_index) {
    const auto& loop = loops[loop_index];
    if (loop.from.second >= input.current.odom_poses.size()) continue;
    const auto optimized = input.all_optimized.find(loop.to.first);
    if (optimized == input.all_optimized.end()) continue;
    const auto target_pose = std::find_if(
        optimized->second->optimized_poses.begin(),
        optimized->second->optimized_poses.end(), [&loop](const auto& value) {
          return value.first == static_cast<int>(loop.to.second);
        });
    if (target_pose == optimized->second->optimized_poses.end()) continue;
    const bool inlier = std::find(diagnostics.inlier_loop_indices.begin(),
                                  diagnostics.inlier_loop_indices.end(),
                                  loop_index) !=
                        diagnostics.inlier_loop_indices.end();
    output.descriptor_loops.push_back({
        VisualizationPoint(target_pose->second.translation()),
        VisualizationPoint((target_T_source *
                            input.current.odom_poses[loop.from.second])
                               .translation()),
        inlier});
  }
  return output;
}
}  // namespace

Result<LoopDetectorOutput> LoopDetectorKdtree::Process(
    const AlgorithmExecutionContext& context,
    const LoopDetectorProcessInput& input) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before loop detection");
  if (!cancellation) {
    return Result<LoopDetectorOutput>::Failure(cancellation.GetError());
  }
  const auto publish = [&](LoopDetectorOutput output)
      -> Result<LoopDetectorOutput> {
    auto completed =
        CheckAlgorithmCancellation(context, "after loop detection");
    if (!completed) {
      return Result<LoopDetectorOutput>::Failure(completed.GetError());
    }
    return Result<LoopDetectorOutput>::Ok(std::move(output));
  };
  try {
  OPEN_LMM_ZONE_N("LoopDetector.Process");
  OPEN_LMM_PLOT("loop_detector.scan_count", input.current.filtered_scans.size());
  database_->setAgentId(context.agent.id);

  auto intra_loops = detectIntraLoops(input.current.filtered_scans, context.agent);
  if (!context.agent.is_anchor() && input.stored_alignment) {
    const auto& stored = *input.stored_alignment;
    if (!IsFiniteRigidTransform(stored.proposal.target_T_source) ||
        stored.proposal.source_agent != context.agent.id) {
      throw std::runtime_error("stored alignment is invalid for the current agent");
    }
    auto inter_loops = loopsFromGlobalTransform(
        context, input, stored.proposal.target_T_source);
    if (inter_loops.empty()) {
      throw std::runtime_error(
          "stored alignment produced no valid inter-agent loops");
    }
    auto transformed_map_points = transformEigenPoints(
        input.current.map_points,
        stored.proposal.target_T_source.matrix().cast<float>());
    return publish(LoopDetectorOutput{
        .intra_loops = std::move(intra_loops),
        .inter_loops = std::move(inter_loops),
        .agent_descriptors = std::make_shared<const DatabaseKdtree>(
            std::move(database_.value())),
        .transformed_map_points = std::move(transformed_map_points),
        .accepted_global_T_agent = stored.proposal.target_T_source,
        .accepted_alignment_method = stored.proposal.method,
        .accepted_alignment_approval = stored.approval,
        .accepted_target_agent = stored.proposal.target_agent,
        .accepted_at_unix_ms = stored.accepted_at_unix_ms,
        .accepted_alignment_metrics = stored.proposal.metrics,
    });
  }
  std::vector<Eigen::Vector3f> transformed_map_points;
  std::optional<MapAlignmentProposal> kiss_proposal;
  std::vector<LoopPair> kiss_loops;
  std::vector<LoopPair> descriptor_loops;
  std::optional<MapAlignmentProposal> descriptor_proposal;
  std::vector<LoopPair> inter_loops;
  std::optional<Eigen::Isometry3d> accepted_global_T_agent;
  std::optional<AlignmentMethod> accepted_alignment_method;
  std::optional<AlignmentApproval> accepted_alignment_approval;
  AgentId accepted_target_agent = context.agent.id;
  AlignmentMetrics accepted_alignment_metrics;

  const bool feedback_available =
      context.feedback && context.feedback->IsEnabled();
  if (!context.agent.is_anchor() && !feedback_available &&
      (params_.feedback_mode == "interactive" ||
       params_.feedback_mode == "always_manual")) {
    throw std::runtime_error(
        "alignment.feedback_mode requires an enabled GUI feedback service");
  }
  const bool interactive = !context.agent.is_anchor() &&
      params_.feedback_mode != "automatic" && feedback_available;
  if (!interactive) {
    kiss_loops = detectKissMatcherLoops(context, input, transformed_map_points,
                                        kiss_proposal);
  }

  if (context.agent.is_anchor()) {
    accepted_global_T_agent = Eigen::Isometry3d::Identity();
    accepted_alignment_method = AlignmentMethod::kKissMatcher;
    accepted_alignment_approval = AlignmentApproval::kAutomatic;
  }

  if (!interactive) {
    descriptor_loops = detectInterLoops(input.current.filtered_scans,
                                        input.descriptor_store,
                                        context.agent);
    descriptor_proposal = DescriptorAlignmentProposer(
        DescriptorOptions(params_)).Propose(
        input.all_optimized.empty() ? AgentId{}
                                    : input.all_optimized.begin()->first,
        context.agent.id, input.current.odom_poses,
        input.all_optimized, descriptor_loops);
    LogProposalDifference(kiss_proposal, descriptor_proposal);
    std::vector<LoopPair> descriptor_pose_loops;
    if (descriptor_proposal) {
      descriptor_pose_loops = loopsFromGlobalTransform(
          context, input, descriptor_proposal->target_T_source);
    }
    if (params_.headless_policy == "fail" && !context.agent.is_anchor()) {
      throw std::runtime_error(
          "headless alignment is disabled by alignment.headless_policy=fail");
    }
    if (params_.headless_policy == "kiss_only") {
      if (!context.agent.is_anchor() && !kiss_proposal) {
        throw std::runtime_error("KISS Matcher did not produce an alignment");
      }
      inter_loops = kiss_loops;
    } else if (params_.headless_policy == "kiss_then_descriptor") {
      inter_loops = kiss_proposal ? kiss_loops : descriptor_pose_loops;
    } else {
      inter_loops = std::move(descriptor_pose_loops);
      inter_loops.insert(inter_loops.end(), kiss_loops.begin(), kiss_loops.end());
    }
    if (kiss_proposal) {
      accepted_global_T_agent = kiss_proposal->target_T_source;
      accepted_alignment_method = AlignmentMethod::kKissMatcher;
      accepted_alignment_approval = AlignmentApproval::kAutomatic;
      accepted_target_agent = kiss_proposal->target_agent;
      accepted_alignment_metrics = kiss_proposal->metrics;
    } else if (!context.agent.is_anchor() &&
               params_.headless_policy != "kiss_only") {
      if (descriptor_proposal) {
        accepted_global_T_agent = descriptor_proposal->target_T_source;
        accepted_alignment_method = AlignmentMethod::kDescriptor;
        accepted_alignment_approval = AlignmentApproval::kAutomatic;
        accepted_target_agent = descriptor_proposal->target_agent;
        accepted_alignment_metrics = descriptor_proposal->metrics;
        transformed_map_points = transformEigenPoints(
            input.current.map_points,
            descriptor_proposal->target_T_source.matrix().cast<float>());
      }
    }
  } else {
    MapAlignmentCoordinator coordinator;
    MapAlignmentCoordinatorInput coordinator_input;
    auto alignment_visualization =
        std::make_shared<AlignmentVisualizationData>();
    coordinator_input.feedback_mode = params_.feedback_mode;
    coordinator_input.feedback_timeout =
        std::chrono::seconds(params_.feedback_timeout_sec);
    coordinator_input.feedback = context.feedback;
    coordinator_input.cancellation = context.cancellation;
    coordinator_input.target_points =
        AlignmentPoints(input.descriptor_store.merged_map);
    coordinator_input.source_points = AlignmentPoints(input.current.map_points);
    coordinator_input.visualization = alignment_visualization;
    coordinator_input.target_agent = input.all_optimized.empty()
                                         ? AgentId{}
                                         : input.all_optimized.begin()->first;
    coordinator_input.source_agent = context.agent.id;
    coordinator_input.kiss_proposer = [&] {
      *alignment_visualization = {};
      kiss_loops = detectKissMatcherLoops(context, input, transformed_map_points,
                                          kiss_proposal);
      return kiss_proposal;
    };
    coordinator_input.descriptor_proposer = [&] {
      descriptor_loops = detectInterLoops(input.current.filtered_scans,
                                          input.descriptor_store,
                                          context.agent);
      DescriptorAlignmentDiagnostics descriptor_diagnostics;
      descriptor_proposal = DescriptorAlignmentProposer(
          DescriptorOptions(params_)).Propose(
          coordinator_input.target_agent, context.agent.id,
          input.current.odom_poses, input.all_optimized, descriptor_loops,
          &descriptor_diagnostics);
      if (descriptor_proposal) {
        *alignment_visualization = DescriptorVisualization(
            context, input, descriptor_loops, descriptor_diagnostics,
            descriptor_proposal->target_T_source,
            coordinator_input.target_agent);
      } else {
        *alignment_visualization = {};
      }
      LogProposalDifference(kiss_proposal, descriptor_proposal);
      return descriptor_proposal;
    };
    std::vector<LoopPair> validated_loops;
    coordinator_input.proposal_validator = [&](const auto& proposal) {
      if (proposal.method == AlignmentMethod::kKissMatcher) {
        validated_loops = kiss_loops;
      } else {
        validated_loops =
            loopsFromGlobalTransform(context, input, proposal.target_T_source);
      }
      if (validated_loops.empty()) {
        return Result<void>::Failure(Error::RegistrationFailed(
            "accepted map alignment produced no valid inter-agent loops"));
      }
      return Result<void>::Ok();
    };

    auto alignment = coordinator.Align(coordinator_input);
    if (!alignment) {
      return Result<LoopDetectorOutput>::Failure(
          WithAlgorithmContext(alignment.GetError(), context));
    }
    const auto& proposal = alignment.Value();
    inter_loops = std::move(validated_loops);
    transformed_map_points = transformEigenPoints(
        input.current.map_points,
        proposal.target_T_source.matrix().cast<float>());
    accepted_global_T_agent = proposal.target_T_source;
    accepted_alignment_method = proposal.method;
    accepted_alignment_approval = AlignmentApproval::kUser;
    accepted_target_agent = proposal.target_agent;
    accepted_alignment_metrics = proposal.metrics;
  }
  OPEN_LMM_PLOT("loop_detector.intra_loops", intra_loops.size());
  OPEN_LMM_PLOT("loop_detector.inter_loops", inter_loops.size());

  return publish(LoopDetectorOutput{
      .intra_loops             = std::move(intra_loops),
      .inter_loops             = std::move(inter_loops),
      .agent_descriptors      = std::make_shared<const DatabaseKdtree>(
          std::move(database_.value())),
      .transformed_map_points  = std::move(transformed_map_points),
      .accepted_global_T_agent = std::move(accepted_global_T_agent),
      .accepted_alignment_method = accepted_alignment_method,
      .accepted_alignment_approval = accepted_alignment_approval,
      .accepted_target_agent = accepted_target_agent,
      .accepted_at_unix_ms = accepted_alignment_method
          ? static_cast<uint64_t>(std::chrono::duration_cast<
                std::chrono::milliseconds>(
                std::chrono::system_clock::now().time_since_epoch()).count())
          : 0,
      .accepted_alignment_metrics = accepted_alignment_metrics,
  });
  } catch (const CancellationException& error) {
    return Result<LoopDetectorOutput>::Failure(WithAlgorithmContext(
        Error::Cancelled(error.what()), context));
  } catch (const std::exception& error) {
    return Result<LoopDetectorOutput>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(std::string("loop detector exception: ") +
                               error.what()),
        context));
  } catch (...) {
    return Result<LoopDetectorOutput>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown loop detector exception"), context));
  }
}

Result<std::shared_ptr<IDescriptorKdtree>> LoopDetectorKdtree::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IDescriptorKdtree>(
      so_name, "descriptor", config_json);
}

}  // namespace open_lmm
