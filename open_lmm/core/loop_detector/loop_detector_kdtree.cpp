#include "loop_detector_kdtree.hpp"
#include "descriptor_alignment_proposer.hpp"
#include "kiss_alignment_proposer.hpp"
#include "map_alignment_coordinator.hpp"

#include <open_lmm/core/alignment/alignment_decision_policy.hpp>
#include <open_lmm/core/alignment/alignment_proposer.hpp>
#include <open_lmm/core/alignment/loop_constraint_builder.hpp>

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
#include <stdexcept>

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

AlignmentFeedbackMode FeedbackMode(const KdtreeParams& params,
                                   bool feedback_available) {
  if (params.feedback_mode == "always_manual") {
    return AlignmentFeedbackMode::kAlwaysManual;
  }
  if (params.feedback_mode == "interactive" ||
      (params.feedback_mode == "adaptive" && feedback_available)) {
    return AlignmentFeedbackMode::kInteractive;
  }
  return AlignmentFeedbackMode::kAutomatic;
}

HeadlessAlignmentPolicy HeadlessPolicy(const KdtreeParams& params) {
  if (params.headless_policy == "kiss_only") {
    return HeadlessAlignmentPolicy::kKissOnly;
  }
  if (params.headless_policy == "kiss_then_descriptor") {
    return HeadlessAlignmentPolicy::kKissThenDescriptor;
  }
  if (params.headless_policy == "fail") {
    return HeadlessAlignmentPolicy::kFail;
  }
  // legacy_combined used to merge constraints derived from two different
  // transforms. It is retained only as a deprecated configuration alias for
  // the single-accepted-transform kiss_then_descriptor policy.
  return HeadlessAlignmentPolicy::kKissThenDescriptor;
}

uint64_t NowUnixMilliseconds() {
  return static_cast<uint64_t>(std::chrono::duration_cast<
      std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count());
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
    std::shared_ptr<const DescriptorEngine> descriptor_engine)
    : params_(params), descriptor_engine_(std::move(descriptor_engine)) {
  if (!descriptor_engine_) {
    throw std::invalid_argument("descriptor engine is null");
  }
  const auto& metadata = descriptor_engine_->IndexMetadata();
  if (metadata.index_dimension == 0) {
    throw std::invalid_argument("descriptor engine index dimension is zero");
  }
  database_.emplace(DatabaseKdtreeParams{
      metadata.index_dimension,
      params_.num_candidates, params_.distance_threshold,
      params_.kdtree_rebuild_threshold}, descriptor_engine_);
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

Result<std::vector<LoopPair>> LoopDetectorKdtree::detectIntraLoops(
    const AlgorithmExecutionContext& context, const ScanVec& scans,
    const AgentContext& agent_ctx) {
  OPEN_LMM_ZONE_N("LoopDetector.IntraQuery");
  std::vector<LoopPair> intra_loop_pairs;
  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Intra Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    if (!scan) {
      return Result<std::vector<LoopPair>>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor input scan is null"), context));
    }
    auto descriptor = descriptor_engine_->Make(
        context, DescriptorPointView{std::span<const pcl::PointXYZI>(
                     scan->points.data(), scan->points.size())});
    if (!descriptor) {
      return Result<std::vector<LoopPair>>::Failure(descriptor.GetError());
    }
    auto queried = database_->queryArtifact(context, descriptor.Value());
    if (!queried) {
      return Result<std::vector<LoopPair>>::Failure(queried.GetError());
    }
    auto intra_loop_candidates = std::move(queried).Value();

    if (intra_loop_candidates != std::nullopt) {
      intra_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, intra_loop_candidates.value()));
    }

    database_->insertArtifact(agent_ctx.id, idx,
                              std::move(descriptor).Value());
  }
  T.finish();
  return Result<std::vector<LoopPair>>::Ok(std::move(intra_loop_pairs));
}

Result<std::vector<LoopPair>> LoopDetectorKdtree::detectInterLoops(
    const AlgorithmExecutionContext& context, const ScanVec& scans,
    const DescriptorStore& descriptor_store,
    const AgentContext& agent_ctx) {
  OPEN_LMM_ZONE_N("LoopDetector.InterQuery");
  std::vector<LoopPair> inter_loop_pairs;

  if (agent_ctx.is_anchor()) {
    return Result<std::vector<LoopPair>>::Ok(std::move(inter_loop_pairs));
  }

  const auto* artifact_index = descriptor_store.total_db
      ? dynamic_cast<const DatabaseKdtree*>(descriptor_store.total_db.get())
      : nullptr;
  if (descriptor_store.total_db && !artifact_index) {
    return Result<std::vector<LoopPair>>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("descriptor index implementation mismatch"),
        context));
  }

  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Inter Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    if (!scan || !artifact_index) continue;
    auto descriptor = descriptor_engine_->Make(
        context, DescriptorPointView{std::span<const pcl::PointXYZI>(
                     scan->points.data(), scan->points.size())});
    if (!descriptor) {
      return Result<std::vector<LoopPair>>::Failure(descriptor.GetError());
    }
    auto queried = artifact_index->queryArtifact(context, descriptor.Value());
    if (!queried) {
      return Result<std::vector<LoopPair>>::Failure(queried.GetError());
    }
    auto inter_loop_candidates = std::move(queried).Value();

    if (inter_loop_candidates != std::nullopt) {
      inter_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, inter_loop_candidates.value()));
    }
  }
  T.finish();
  return Result<std::vector<LoopPair>>::Ok(std::move(inter_loop_pairs));
}

std::optional<MapAlignmentProposal> LoopDetectorKdtree::proposeKissAlignment(
    const AlgorithmExecutionContext& context,
    const LoopDetectorProcessInput& input) {
  OPEN_LMM_ZONE_N("LoopDetector.KissMatcher");
  if (context.agent.is_anchor()) return std::nullopt;

  const AgentId target_agent = input.all_optimized.empty()
                                ? AgentId{}
                                : input.all_optimized.begin()->first;
  return KissAlignmentProposer().Propose(
      input.descriptor_store.merged_map, input.current.map_points,
      target_agent, context.agent.id, params_.kiss_voxel_size,
      params_.kiss_use_quatro);
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
    OPEN_LMM_PLOT("loop_detector.scan_count",
                  input.current.filtered_scans.size());
    database_->setAgentId(context.agent.id);

    auto intra_result = detectIntraLoops(
        context, input.current.filtered_scans, context.agent);
    if (!intra_result) {
      return Result<LoopDetectorOutput>::Failure(intra_result.GetError());
    }
    auto intra_loops = std::move(intra_result).Value();
    const bool feedback_available =
        context.feedback && context.feedback->IsEnabled();
    const AlignmentFeedbackMode feedback_mode =
        FeedbackMode(params_, feedback_available);
    AlignmentPolicyInput policy_input{
        .source_agent = context.agent.id,
        .source_is_anchor = context.agent.is_anchor(),
        .feedback_mode = feedback_mode,
        .headless_policy = HeadlessPolicy(params_),
        .interactive_service_available = feedback_available,
        .stored_alignment = input.stored_alignment
                                ? std::optional<StoredAlignment>(
                                      *input.stored_alignment)
                                : std::nullopt,
    };
    AlignmentDecisionPolicy policy;
    std::optional<MapAlignmentProposal> kiss_proposal;
    std::optional<MapAlignmentProposal> descriptor_proposal;
    LoopPairVec descriptor_loops;
    std::optional<ValidatedLoopConstraints> built_constraints;

    const AgentId target_agent = input.all_optimized.empty()
                                      ? AgentId{}
                                      : input.all_optimized.begin()->first;
    AlignmentProposer kiss([&](const AlignmentProposalRequest&) {
      return Result<std::optional<MapAlignmentProposal>>::Ok(
          proposeKissAlignment(context, input));
    });
    AlignmentProposer descriptor([&](const AlignmentProposalRequest&) {
      auto detected = detectInterLoops(
          context, input.current.filtered_scans, input.descriptor_store,
          context.agent);
      if (!detected) {
        return Result<std::optional<MapAlignmentProposal>>::Failure(
            detected.GetError());
      }
      descriptor_loops = std::move(detected).Value();
      return Result<std::optional<MapAlignmentProposal>>::Ok(
          DescriptorAlignmentProposer(DescriptorOptions(params_)).Propose(
              target_agent, context.agent.id, input.current.odom_poses,
              input.all_optimized, descriptor_loops));
    });

    const auto build_constraints = [&](const StoredAlignment& accepted,
                                       AlignmentAcceptanceSource source)
        -> Result<ValidatedLoopConstraints> {
      return LoopConstraintBuilder().Build(
          context,
          {accepted, source, input.current, input.all_raw_data,
           input.all_optimized, params_.pose_nn_distance_threshold,
           params_.inter_loop_keyframe_spacing_m});
    };

    if (!context.agent.is_anchor() && !input.stored_alignment &&
        feedback_mode == AlignmentFeedbackMode::kAutomatic) {
      const HeadlessAlignmentPolicy headless = HeadlessPolicy(params_);
      const auto run_kiss = [&]() -> Result<void> {
        auto result = kiss.Propose(context, {target_agent, context.agent.id});
        if (!result) return Result<void>::Failure(result.GetError());
        kiss_proposal = std::move(result).Value();
        return Result<void>::Ok();
      };
      const auto run_descriptor = [&]() -> Result<void> {
        auto result =
            descriptor.Propose(context, {target_agent, context.agent.id});
        if (!result) return Result<void>::Failure(result.GetError());
        descriptor_proposal = std::move(result).Value();
        return Result<void>::Ok();
      };
      if (headless != HeadlessAlignmentPolicy::kFail) {
        auto proposed = run_kiss();
        if (!proposed) {
          return Result<LoopDetectorOutput>::Failure(proposed.GetError());
        }
      }
      if (headless == HeadlessAlignmentPolicy::kKissThenDescriptor &&
          !kiss_proposal) {
        auto proposed = run_descriptor();
        if (!proposed) {
          return Result<LoopDetectorOutput>::Failure(proposed.GetError());
        }
      }
      LogProposalDifference(kiss_proposal, descriptor_proposal);
      policy_input.kiss_proposal = kiss_proposal;
      policy_input.descriptor_proposal = descriptor_proposal;
    }
    if (context.agent.is_anchor() || input.stored_alignment ||
        feedback_mode == AlignmentFeedbackMode::kAutomatic) {
      policy_input.accepted_at_unix_ms = NowUnixMilliseconds();
    }
    Result<AlignmentPolicyOutcome> decision = policy.Decide(policy_input);
    if (decision && decision.Value().action ==
                        AlignmentPolicyAction::kRequestInteractive) {
      MapAlignmentCoordinatorInput coordinator_input;
      auto alignment_visualization =
          std::make_shared<AlignmentVisualizationData>();
      coordinator_input.intent = decision.Value().manual_only
          ? InteractiveAlignmentIntent::kManualOnly
          : InteractiveAlignmentIntent::kChooseMethod;
      coordinator_input.feedback_timeout =
          std::chrono::seconds(params_.feedback_timeout_sec);
      coordinator_input.feedback = context.feedback;
      coordinator_input.cancellation = context.cancellation;
      coordinator_input.target_points =
          AlignmentPoints(input.descriptor_store.merged_map);
      coordinator_input.source_points =
          AlignmentPoints(input.current.map_points);
      coordinator_input.visualization = alignment_visualization;
      coordinator_input.target_agent = target_agent;
      coordinator_input.source_agent = context.agent.id;
      coordinator_input.kiss_proposer = [&] {
        *alignment_visualization = {};
        auto result = kiss.Propose(context, {target_agent, context.agent.id});
        if (result) kiss_proposal = result.Value();
        return result;
      };
      AlignmentProposer interactive_descriptor(
          [&](const AlignmentProposalRequest&)
              -> Result<std::optional<MapAlignmentProposal>> {
        DescriptorAlignmentDiagnostics diagnostics;
        auto detected = detectInterLoops(
            context, input.current.filtered_scans, input.descriptor_store,
            context.agent);
        if (!detected) {
          return Result<std::optional<MapAlignmentProposal>>::Failure(
              detected.GetError());
        }
        descriptor_loops = std::move(detected).Value();
        descriptor_proposal = DescriptorAlignmentProposer(
            DescriptorOptions(params_)).Propose(
            target_agent, context.agent.id, input.current.odom_poses,
            input.all_optimized, descriptor_loops, &diagnostics);
        if (descriptor_proposal) {
          *alignment_visualization = DescriptorVisualization(
              context, input, descriptor_loops, diagnostics,
              descriptor_proposal->target_T_source, target_agent);
        } else {
          *alignment_visualization = {};
        }
        LogProposalDifference(kiss_proposal, descriptor_proposal);
        return Result<std::optional<MapAlignmentProposal>>::Ok(
            descriptor_proposal);
      });
      coordinator_input.descriptor_proposer = [&] {
        return interactive_descriptor.Propose(
            context, {target_agent, context.agent.id});
      };
      coordinator_input.proposal_validator = [&](const auto& proposal) {
        StoredAlignment accepted{proposal, AlignmentApproval::kUser, 0};
        auto built = build_constraints(
            accepted, AlignmentAcceptanceSource::kInteractive);
        if (!built) return Result<void>::Failure(built.GetError());
        built_constraints = std::move(built).Value();
        return Result<void>::Ok();
      };
      auto alignment = MapAlignmentCoordinator().Align(coordinator_input);
      if (!alignment) {
        return Result<LoopDetectorOutput>::Failure(
            WithAlgorithmContext(alignment.GetError(), context));
      }
      policy_input.accepted_at_unix_ms = NowUnixMilliseconds();
      policy_input.interactive = {InteractiveAlignmentState::kAccepted,
                                  alignment.Value()};
      decision = policy.Decide(policy_input);
    }
    if (!decision) {
      return Result<LoopDetectorOutput>::Failure(
          WithAlgorithmContext(decision.GetError(), context));
    }
    if (!decision.Value().accepted) {
      return Result<LoopDetectorOutput>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("alignment policy accepted no proposal"),
          context));
    }
    const StoredAlignment accepted = *decision.Value().accepted;
    LoopPairVec inter_loops;
    if (!context.agent.is_anchor()) {
      if (!built_constraints) {
        const auto source = input.stored_alignment
                                ? AlignmentAcceptanceSource::kStored
                                : feedback_mode == AlignmentFeedbackMode::kAutomatic
                                      ? AlignmentAcceptanceSource::kAutomatic
                                      : AlignmentAcceptanceSource::kInteractive;
        auto built = build_constraints(accepted, source);
        if (!built) {
          return Result<LoopDetectorOutput>::Failure(built.GetError());
        }
        built_constraints = std::move(built).Value();
      }
      inter_loops = std::move(built_constraints->loops);
    }
    auto transformed_map_points = context.agent.is_anchor()
        ? input.current.map_points
        : transformEigenPoints(
              input.current.map_points,
              accepted.proposal.target_T_source.matrix().cast<float>());

    OPEN_LMM_PLOT("loop_detector.intra_loops", intra_loops.size());
    OPEN_LMM_PLOT("loop_detector.inter_loops", inter_loops.size());
    return publish(LoopDetectorOutput{
        .intra_loops = std::move(intra_loops),
        .inter_loops = std::move(inter_loops),
        .agent_descriptors = std::make_shared<const DatabaseKdtree>(
            std::move(database_.value())),
        .transformed_map_points = std::move(transformed_map_points),
        .accepted_global_T_agent = accepted.proposal.target_T_source,
        .accepted_alignment_method = accepted.proposal.method,
        .accepted_alignment_approval = accepted.approval,
        .accepted_target_agent = accepted.proposal.target_agent,
        .accepted_at_unix_ms = accepted.accepted_at_unix_ms,
        .accepted_alignment_metrics = accepted.proposal.metrics,
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
