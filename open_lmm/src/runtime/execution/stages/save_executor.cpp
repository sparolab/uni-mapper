#include "save_executor.hpp"

#include <foundation/diagnostics/profiling.hpp>
#include <foundation/logging/logging.hpp>

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <Eigen/Geometry>

#include <fstream>
#include <iomanip>
#include <sstream>
#include <string>

namespace fs = std::filesystem;
namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

Result<std::string> FingerprintFile(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return Result<std::string>::Failure(
        Error::IoError("failed to fingerprint output: " + path.string()));
  }
  uint64_t hash = kFnvOffset;
  char buffer[8192];
  while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
    for (std::streamsize index = 0; index < input.gcount(); ++index) {
      hash ^= static_cast<unsigned char>(buffer[index]);
      hash *= kFnvPrime;
    }
  }
  if (input.bad()) {
    return Result<std::string>::Failure(
        Error::IoError("failed while fingerprinting output: " +
                       path.string()));
  }
  std::ostringstream encoded;
  encoded << std::hex << std::setfill('0') << std::setw(16) << hash;
  return Result<std::string>::Ok(encoded.str());
}

Result<void> CheckCancelled(const SaveExecutionRequest& request,
                            std::string_view boundary) {
  if (request.cancellation &&
      request.cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(Error::Cancelled(boundary));
  }
  return Result<void>::Ok();
}

Result<void> RemoveStaleTemporary(const fs::path& temporary) {
  std::error_code error;
  const bool exists = fs::exists(temporary, error);
  if (error) {
    return Result<void>::Failure(Error::IoError(
        "failed to inspect temporary output " + temporary.string() + ": " +
        error.message()));
  }
  if (!exists) return Result<void>::Ok();
  if (!fs::remove(temporary, error) || error) {
    return Result<void>::Failure(Error::IoError(
        "failed to remove stale temporary output " + temporary.string() +
        ": " + (error ? error.message() : "not removed")));
  }
  return Result<void>::Ok();
}

}  // namespace

Result<SaveExecutionSummary> SaveExecutor::Prepare(
    const SaveExecutionRequest& request, PendingOutputSet& pending,
    ArtifactRepository& artifacts) const {
  OPEN_LMM_ZONE_N("SaveExecutor.Prepare");
  if (request.mode == SaveExecutionMode::kStage) {
    const auto save_nodes = StageNodes(StageId::kSave);
    if (save_nodes != std::vector<NodeId>{NodeId::kPoseSave,
                                          NodeId::kFallbackMapSave}) {
      return Result<SaveExecutionSummary>::Failure(
          Error::InvalidArgument("unsupported Save execution spec"));
    }
  }

  SaveExecutionSummary summary;
  const bool prepare_poses = request.mode == SaveExecutionMode::kStage ||
                             request.mode == SaveExecutionMode::kPoseSave;
  const bool prepare_fallback =
      request.mode == SaveExecutionMode::kStage ||
      request.mode == SaveExecutionMode::kFallbackMapSave;
  if (prepare_fallback && request.map_update_enabled) {
    summary.fallback_map_skipped = true;
    if (!prepare_poses) {
      return Result<SaveExecutionSummary>::Ok(std::move(summary));
    }
  }

  if (!request.state.payload || !request.state.payload->database ||
      request.state.payload->database->optimized_data.empty()) {
    return Result<SaveExecutionSummary>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before Save"));
  }
  if (request.output_directory.empty()) {
    return Result<SaveExecutionSummary>::Failure(
        Error::InvalidArgument("Save output directory is empty"));
  }
  auto cancelled = CheckCancelled(request, "before Save preparation");
  if (!cancelled) {
    return Result<SaveExecutionSummary>::Failure(cancelled.GetError());
  }

  if (prepare_poses) {
    auto pose_agents =
        artifacts.ExecutionAgents(NodeId::kPoseSave, std::nullopt);
    if (!pose_agents) {
      return Result<SaveExecutionSummary>::Failure(pose_agents.GetError());
    }
    summary.pose_agents = pose_agents.Value();
    artifacts.BeginNode(NodeId::kPoseSave, summary.pose_agents);
    auto poses = PrepareOptimizedPoses(request, summary.pose_agents, pending,
                                       artifacts);
    if (!poses) return Result<SaveExecutionSummary>::Failure(poses.GetError());
  }

  if (!prepare_fallback || request.map_update_enabled) {
    return Result<SaveExecutionSummary>::Ok(std::move(summary));
  }

  auto map_agents =
      artifacts.ExecutionAgents(NodeId::kFallbackMapSave, std::nullopt);
  if (!map_agents) {
    return Result<SaveExecutionSummary>::Failure(map_agents.GetError());
  }
  summary.fallback_map_agents = map_agents.Value();
  artifacts.BeginNode(NodeId::kFallbackMapSave,
                      summary.fallback_map_agents);
  auto maps = PrepareFallbackMaps(request, summary.fallback_map_agents,
                                  pending, artifacts);
  if (!maps) return Result<SaveExecutionSummary>::Failure(maps.GetError());
  return Result<SaveExecutionSummary>::Ok(std::move(summary));
}

Result<void> SaveExecutor::PrepareOptimizedPoses(
    const SaveExecutionRequest& request,
    const std::vector<AgentId>& affected_agents, PendingOutputSet& pending,
    ArtifactRepository& artifacts) const {
  OPEN_LMM_ZONE_N("SaveExecutor.Poses");
  const auto& database = *request.state.payload->database;
  for (const AgentId& agent : affected_agents) {
    const auto found = database.optimized_data.find(agent);
    if (found == database.optimized_data.end() || !found->second ||
        found->second->optimized_poses.empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Ready OptimizedPoses artifact has no pose payload for agent " +
          agent.Value()));
    }
  }

  for (const AgentId& agent : affected_agents) {
    auto cancelled = CheckCancelled(request, "during pose preparation");
    if (!cancelled) return cancelled;
    const fs::path destination =
        request.output_directory /
        ("optimized_poses_" + agent.Value() + ".txt");
    fs::path temporary = destination;
    temporary += ".tmp";
    auto removed = RemoveStaleTemporary(temporary);
    if (!removed) return removed;
    pending.Add(temporary, destination);

    std::ofstream output(temporary);
    if (!output) {
      return Result<void>::Failure(Error::IoError(
          "failed to open temporary pose output: " + temporary.string()));
    }
    for (const auto& [frame, pose] :
         database.optimized_data.at(agent)->optimized_poses) {
      cancelled = CheckCancelled(request, "during pose write");
      if (!cancelled) return cancelled;
      const Eigen::Vector3d translation = pose.translation();
      const Eigen::Quaterniond quaternion(pose.linear());
      output << frame << ',' << translation.x() << ',' << translation.y()
             << ',' << translation.z() << ',' << quaternion.x() << ','
             << quaternion.y() << ',' << quaternion.z() << ','
             << quaternion.w() << '\n';
    }
    output.flush();
    if (!output) {
      return Result<void>::Failure(Error::IoError(
          "failed to write pose output: " + temporary.string()));
    }
    auto fingerprint = FingerprintFile(temporary);
    if (!fingerprint) return Result<void>::Failure(fingerprint.GetError());
    artifacts.RecordExternalFile(ArtifactType::kPoseFile, agent,
                                 destination.string(), fingerprint.Value());
  }
  return CheckCancelled(request, "before pose file-set commit");
}

Result<void> SaveExecutor::PrepareFallbackMaps(
    const SaveExecutionRequest& request,
    const std::vector<AgentId>& affected_agents, PendingOutputSet& pending,
    ArtifactRepository& artifacts) const {
  OPEN_LMM_ZONE_N("SaveExecutor.FallbackMaps");
  const auto& database = *request.state.payload->database;
  for (const AgentId& agent : affected_agents) {
    const auto optimized = database.optimized_data.find(agent);
    const auto raw = database.raw_data.find(agent);
    if (optimized == database.optimized_data.end() || !optimized->second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Ready OptimizedPoses artifact has no map payload for agent " +
          agent.Value()));
    }
    if (raw == database.raw_data.end() || !raw->second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Optimized agent has no corresponding raw data"));
    }

    auto map = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    for (const auto& [frame, pose] : optimized->second->optimized_poses) {
      auto cancelled = CheckCancelled(request, "during map assembly");
      if (!cancelled) return cancelled;
      if (frame < 0 || static_cast<std::size_t>(frame) >=
                           raw->second->filtered_scans.size() ||
          !raw->second->filtered_scans[static_cast<std::size_t>(frame)]) {
        return Result<void>::Failure(Error::InvalidArgument(
            "Optimized pose scan index is out of range"));
      }
      pcl::PointCloud<pcl::PointXYZI> transformed;
      pcl::transformPointCloud(
          *raw->second->filtered_scans[static_cast<std::size_t>(frame)],
          transformed, pose.matrix());
      *map += transformed;
    }
    if (map->empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Optimized map is empty for agent " + agent.Value()));
    }

    const fs::path destination =
        request.output_directory / ("global_map_" + agent.Value() + ".pcd");
    fs::path temporary = destination;
    temporary += ".tmp";
    auto removed = RemoveStaleTemporary(temporary);
    if (!removed) return removed;
    pending.Add(temporary, destination);
    if (pcl::io::savePCDFileBinaryCompressed(temporary, *map) != 0) {
      std::error_code ignored;
      fs::remove(temporary, ignored);
      return Result<void>::Failure(Error::IoError(
          "failed to save temporary map output: " + temporary.string()));
    }
    auto cancelled = CheckCancelled(request, "before map file-set commit");
    if (!cancelled) return cancelled;
    auto fingerprint = FingerprintFile(temporary);
    if (!fingerprint) return Result<void>::Failure(fingerprint.GetError());
    for (ArtifactType type : {ArtifactType::kGlobalMap,
                              ArtifactType::kPcdFile}) {
      artifacts.RecordExternalFile(type, agent, destination.string(),
                                   fingerprint.Value());
    }
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
