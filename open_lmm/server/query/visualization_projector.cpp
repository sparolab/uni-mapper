#include "visualization_projector.hpp"

#include <algorithm>
#include <filesystem>
#include <utility>

#include <pcl/common/point_tests.h>
#include <pcl/io/pcd_io.h>

namespace open_lmm {

void VisualizationProjector::Clear(uint64_t runtime_revision) {
  auto state = std::make_shared<State>();
  state->revision = runtime_revision;
  std::lock_guard lock(mutex_);
  state_ = std::move(state);
}

void VisualizationProjector::Publish(
    std::shared_ptr<const RuntimeState> runtime, bool include_maps) {
  if (!runtime || !runtime->config || !runtime->payload ||
      !runtime->payload->database) {
    Clear(runtime ? runtime->revision : 0);
    return;
  }

  auto state = std::make_shared<State>();
  state->revision = runtime->revision;
  const auto& database = *runtime->payload->database;
  for (const auto& [agent, optimized] : database.optimized_data) {
    if (!optimized) continue;
    auto& snapshot = state->agents[agent];
    snapshot.agent = agent;
    snapshot.revision = runtime->revision;
    snapshot.poses.reserve(optimized->optimized_poses.size());
    for (const auto& [index, pose] : optimized->optimized_poses) {
      snapshot.poses.push_back({index, pose.cast<float>()});
    }
    for (std::size_t index = 1; index < snapshot.poses.size(); ++index) {
      snapshot.edges.push_back(
          {agent, static_cast<std::size_t>(snapshot.poses[index - 1].index),
           agent, static_cast<std::size_t>(snapshot.poses[index].index),
           VisualizationEdgeType::kTrajectory});
    }

    const auto context = std::find_if(
        runtime->payload->contexts.begin(), runtime->payload->contexts.end(),
        [&agent](const AgentPipelineCtx& item) {
          return item.agent.id == agent;
        });
    if (context != runtime->payload->contexts.end() && context->loop_output) {
      const auto append_loops = [&snapshot](const LoopPairVec& loops,
                                            VisualizationEdgeType type) {
        for (const auto& loop : loops) {
          snapshot.edges.push_back({loop.from.first, loop.from.second,
                                    loop.to.first, loop.to.second, type});
        }
      };
      append_loops(context->loop_output->intra_loops,
                   VisualizationEdgeType::kIntraLoop);
      append_loops(context->loop_output->inter_loops,
                   VisualizationEdgeType::kInterLoop);
    }

    if (include_maps) {
      const auto path = runtime->config->root.output_directory /
                        ("global_map_" + agent.Value() + ".pcd");
      std::error_code error;
      if (std::filesystem::is_regular_file(path, error) && !error) {
        state->map_paths.emplace(agent, path);
      }
    }
  }

  std::lock_guard lock(mutex_);
  state_ = std::move(state);
}

Result<VisualizationSnapshot> VisualizationProjector::Project(
    const AgentId& agent) const {
  std::shared_ptr<const State> state;
  {
    std::lock_guard lock(mutex_);
    state = state_;
  }
  const auto found = state->agents.find(agent);
  if (found == state->agents.end()) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("optimized poses are not available for agent"));
  }

  VisualizationSnapshot snapshot = found->second;
  const auto map = state->map_paths.find(agent);
  if (map == state->map_paths.end()) {
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile(map->second.string(), cloud) < 0) {
    return Result<VisualizationSnapshot>::Failure(
        Error::IoError("failed to read visualization map " +
                       map->second.string()));
  }
  snapshot.points.reserve(cloud.size());
  for (const auto& point : cloud) {
    if (!pcl::isFinite(point)) continue;
    snapshot.points.push_back({point.x, point.y, point.z, point.intensity});
    const Eigen::Vector3f position(point.x, point.y, point.z);
    if (!snapshot.has_bounds) {
      snapshot.min_bound = snapshot.max_bound = position;
      snapshot.has_bounds = true;
    } else {
      snapshot.min_bound = snapshot.min_bound.cwiseMin(position);
      snapshot.max_bound = snapshot.max_bound.cwiseMax(position);
    }
  }
  snapshot.map_available = true;
  return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
}

}  // namespace open_lmm
