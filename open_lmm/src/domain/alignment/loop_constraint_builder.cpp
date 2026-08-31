#include <open_lmm/core/alignment/loop_constraint_builder.hpp>

#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include <pcl/kdtree/kdtree_flann.h>

#include <open_lmm/common/rigid_transform.hpp>
#include <domain/support/algorithm_invariants.hpp>

namespace open_lmm {
namespace {

Result<void> ValidateAcceptedAlignment(
    const AlgorithmExecutionContext& context,
    const LoopConstraintBuildInput& input) {
  const auto& proposal = input.accepted.proposal;
  if (!proposal.target_agent.IsValid() || !proposal.source_agent.IsValid() ||
      proposal.target_agent == proposal.source_agent) {
    return Result<void>::Failure(Error::InvalidArgument(
        "accepted alignment must join two valid, different agents"));
  }
  if (proposal.source_agent != input.source.agent_id ||
      (context.agent.id.IsValid() && context.agent.id != input.source.agent_id)) {
    return Result<void>::Failure(Error::InvalidArgument(
        "accepted alignment source does not match the execution agent"));
  }
  if (proposal.method == AlignmentMethod::kPending) {
    return Result<void>::Failure(
        Error::InvalidArgument("pending alignment cannot produce constraints"));
  }
  auto transform = ValidateRigidTransform(
      proposal.target_T_source, "accepted alignment transform");
  if (!transform) return transform;
  if (!std::isfinite(input.pose_nn_distance_threshold) ||
      input.pose_nn_distance_threshold <= 0.0 ||
      !std::isfinite(input.minimum_source_separation) ||
      input.minimum_source_separation < 0.0) {
    return Result<void>::Failure(Error::InvalidArgument(
        "loop constraint distance thresholds are invalid"));
  }
  auto source = ValidateAgentRawData(input.source,
                                     "loop constraint source raw data");
  if (!source) return source;
  if (!input.optimized_agents.contains(proposal.target_agent)) {
    return Result<void>::Failure(Error::InvalidArgument(
        "accepted alignment target has no optimized trajectory"));
  }
  return Result<void>::Ok();
}

}  // namespace

Result<ValidatedLoopConstraints> LoopConstraintBuilder::Build(
    const AlgorithmExecutionContext& context,
    const LoopConstraintBuildInput& input) const {
  AlgorithmExecutionTimer timer(context);
  auto cancelled =
      CheckAlgorithmCancellation(context, "before loop constraint build");
  if (!cancelled) {
    return Result<ValidatedLoopConstraints>::Failure(cancelled.GetError());
  }
  auto accepted = ValidateAcceptedAlignment(context, input);
  if (!accepted) {
    return Result<ValidatedLoopConstraints>::Failure(
        WithAlgorithmContext(accepted.GetError(), context));
  }

  LoopConstraintBuildDiagnostics diagnostics;
  diagnostics.threshold_m = input.pose_nn_distance_threshold;
  diagnostics.nearest_distance_m =
      std::numeric_limits<double>::infinity();
  const auto publish_diagnostics = [&] {
    if (input.diagnostics) *input.diagnostics = diagnostics;
  };
  publish_diagnostics();

  try {
    PoseVec transformed;
    transformed.reserve(input.source.odom_poses.size());
    for (const auto& pose : input.source.odom_poses) {
      transformed.push_back(input.accepted.proposal.target_T_source * pose);
    }

    std::vector<std::size_t> sampled_source_frames;
    Eigen::Vector3d previous = transformed.front().translation();
    for (std::size_t source_frame = 0; source_frame < transformed.size();
         ++source_frame) {
      const Eigen::Vector3d current = transformed[source_frame].translation();
      if ((current - previous).norm() < input.minimum_source_separation) {
        continue;
      }
      previous = current;
      sampled_source_frames.push_back(source_frame);
    }
    diagnostics.sampled_source_frames = sampled_source_frames.size();

    LoopPairVec loops;
    for (const auto& [target_id, optimized_handle] : input.optimized_agents) {
      cancelled = CheckAlgorithmCancellation(
          context, "while building loop constraints");
      if (!cancelled) {
        return Result<ValidatedLoopConstraints>::Failure(cancelled.GetError());
      }
      if (target_id == input.source.agent_id || !optimized_handle) continue;
      const auto raw = input.all_raw_data.find(target_id);
      if (raw == input.all_raw_data.end() || !raw->second) continue;
      auto valid_target = ValidateOptimizedData(
          *optimized_handle, *raw->second,
          "loop constraint target agent '" + target_id.Value() + "'");
      if (!valid_target) {
        return Result<ValidatedLoopConstraints>::Failure(
            WithAlgorithmContext(valid_target.GetError(), context));
      }
      pcl::PointCloud<pcl::PointXYZ>::Ptr target_points(
          new pcl::PointCloud<pcl::PointXYZ>());
      if (!optimized_handle->kdtree_poses.empty()) {
        *target_points = optimized_handle->kdtree_poses;
      } else {
        target_points->reserve(optimized_handle->optimized_poses.size());
        for (const auto& [frame, pose] : optimized_handle->optimized_poses) {
          (void)frame;
          target_points->push_back(
              pcl::PointXYZ(static_cast<float>(pose.translation().x()),
                            static_cast<float>(pose.translation().y()),
                            static_cast<float>(pose.translation().z())));
        }
      }
      diagnostics.target_frames += target_points->size();
      if (target_points->empty()) continue;
      pcl::KdTreeFLANN<pcl::PointXYZ> nearest;
      nearest.setInputCloud(target_points);

      for (const std::size_t source_frame : sampled_source_frames) {
        const Eigen::Vector3d current = transformed[source_frame].translation();

        std::vector<int> indices(1);
        std::vector<float> squared_distances(1);
        const int found = nearest.nearestKSearch(
            pcl::PointXYZ(static_cast<float>(current.x()),
                          static_cast<float>(current.y()),
                          static_cast<float>(current.z())),
            1, indices, squared_distances);
        if (found == 1 && !squared_distances.empty()) {
          diagnostics.nearest_distance_m = std::min(
              diagnostics.nearest_distance_m,
              std::sqrt(static_cast<double>(squared_distances.front())));
        }
        if (found != 1 || indices.front() < 0 ||
            static_cast<std::size_t>(indices.front()) >=
                optimized_handle->optimized_poses.size() ||
            squared_distances.empty() ||
            std::sqrt(squared_distances.front()) >=
                input.pose_nn_distance_threshold) {
          continue;
        }
        ++diagnostics.within_radius;
        const auto& best = optimized_handle->optimized_poses[
            static_cast<std::size_t>(indices.front())];
        auto relative = TargetFromSourceScanTransform(
            best.second, transformed[source_frame]);
        auto valid_relative = ValidateRigidTransform(
            relative, "generated loop constraint transform");
        if (!valid_relative) {
          return Result<ValidatedLoopConstraints>::Failure(
              WithAlgorithmContext(valid_relative.GetError(), context));
        }
        loops.push_back({
            .to = {target_id, static_cast<std::size_t>(best.first)},
            .from = {input.source.agent_id, source_frame},
            .init_rel_pose = std::move(relative),
        });
      }
    }
    diagnostics.search_completed = true;
    publish_diagnostics();
    if (loops.empty()) {
      return Result<ValidatedLoopConstraints>::Failure(WithAlgorithmContext(
          Error::RegistrationFailed(
              "accepted alignment produced no valid inter-agent loops"),
          context));
    }
    cancelled =
        CheckAlgorithmCancellation(context, "after loop constraint build");
    if (!cancelled) {
      return Result<ValidatedLoopConstraints>::Failure(cancelled.GetError());
    }
    return Result<ValidatedLoopConstraints>::Ok(
        {input.accepted, input.acceptance_source, std::move(loops)});
  } catch (const std::exception& error) {
    return Result<ValidatedLoopConstraints>::Failure(WithAlgorithmContext(
        Error::RegistrationFailed(
            std::string("loop constraint builder exception: ") + error.what())
            .MarkFatalRuntime(),
        context));
  } catch (...) {
    return Result<ValidatedLoopConstraints>::Failure(WithAlgorithmContext(
        Error::RegistrationFailed("unknown loop constraint builder exception")
            .MarkFatalRuntime(),
        context));
  }
}

}  // namespace open_lmm
