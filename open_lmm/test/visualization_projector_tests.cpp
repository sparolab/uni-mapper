#include <open_lmm/server/query/visualization_projector.hpp>

#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

void Check(bool value, const char* message) {
  if (value) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

std::shared_ptr<RuntimeState> MakeState(const fs::path& output,
                                        uint64_t revision = 7) {
  const AgentId agent = Id("agent");
  auto raw = std::make_shared<AgentRawData>();
  raw->agent_id = agent;
  raw->odom_poses = {Eigen::Isometry3d::Identity(),
                     Eigen::Isometry3d::Identity()};
  raw->odom_poses[1].translation().x() = 1.0;
  for (int frame = 0; frame != 2; ++frame) {
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->push_back(pcl::PointXYZI{0.0F, 0.0F, 0.0F,
                                  static_cast<float>(frame)});
    scan->push_back(pcl::PointXYZI{0.1F, 0.0F, 0.0F,
                                  static_cast<float>(frame)});
    raw->filtered_scans.push_back(std::move(scan));
  }

  auto database = std::make_shared<SharedDatabase>();
  database->raw_data.emplace(agent, raw);
  auto payload = std::make_shared<RuntimePayload>();
  payload->database = database;
  AgentPipelineCtx context;
  context.agent.id = agent;
  context.raw_data = raw;
  payload->contexts.push_back(std::move(context));
  auto config = std::make_shared<RuntimeConfig>();
  config->root.output_directory = output;
  auto state = std::make_shared<RuntimeState>();
  state->revision = revision;
  state->config = config;
  state->ordered_agents = {agent};
  state->payload = payload;
  return state;
}

void TestDataLoadIsLazyAndBounded() {
  VisualizationProjector projector;
  auto state = MakeState({});
  projector.Publish(state, VisualizationPhase::kDataLoad, false);
  auto metadata = projector.Project({Id("agent"), false, 0.4F, 2});
  Check(metadata && metadata.Value().phase == VisualizationPhase::kDataLoad &&
            metadata.Value().pose_kind == VisualizationPoseKind::kOdometry &&
            metadata.Value().poses.size() == 2 &&
            metadata.Value().points.empty() &&
            metadata.Value().points_available &&
            !metadata.Value().points_complete,
        "DataLoad metadata exposes odometry without materializing points");

  auto full = projector.Project({Id("agent"), true, 0.4F, 2});
  Check(full && full.Value().points_complete &&
            full.Value().points.size() == 2 &&
            full.Value().displayed_point_count == 2 &&
            full.Value().source_point_count == 4 &&
            full.Value().point_kind ==
                VisualizationPointKind::kFilteredScanPreview,
        "DataLoad preview is bounded and accounts for source points");
}

void TestCancelledProjectionCannotPopulateCache() {
  VisualizationProjector projector;
  projector.Publish(MakeState({}), VisualizationPhase::kDataLoad, false);
  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  {
    CancellationContextScope scope(cancellation);
    auto cancelled = projector.Project({Id("agent"), true, 0.4F, 2});
    Check(!cancelled &&
              cancelled.GetError().code == Error::Code::kCancelled,
          "superseded projection stops before publishing a cache result");
  }
  auto retry = projector.Project({Id("agent"), true, 0.4F, 2});
  Check(retry && retry.Value().points.size() == 2,
        "a cancelled projection leaves the query available for a clean retry");
}

void TestQueryShapeCacheIsBounded() {
  VisualizationProjector projector;
  projector.Publish(MakeState({}), VisualizationPhase::kDataLoad, false);
  for (std::size_t shape = 1; shape <= 40; ++shape) {
    auto result = projector.Project(
        {Id("agent"), true, static_cast<float>(shape) / 1000.0F,
         shape + 1});
    Check(result.IsOk(), "variant visualization query succeeds");
  }
  Check(projector.PointCacheEntryCount() <= 16,
        "variant query shapes retain at most sixteen cache entries");
  Check(projector.PointCacheBytes() <= 128U * 1024U * 1024U,
        "variant query shapes stay within the cache byte budget");
  auto newest = projector.Project({Id("agent"), true, 0.04F, 41});
  Check(newest && newest.Value().points_complete,
        "most recently used query remains available after cache eviction");
}

void TestLoopCandidatesAndCandidateFrameSurvive() {
  const AgentId agent = Id("agent");
  auto state = MakeState({});
  auto payload = std::make_shared<RuntimePayload>(*state->payload);
  auto contexts = payload->contexts;
  auto loops = std::make_shared<LoopDetectorOutput>();
  loops->accepted_global_T_agent = Eigen::Isometry3d::Identity();
  auto accepted = *loops->accepted_global_T_agent;
  accepted.translation().x() = 10.0;
  loops->accepted_global_T_agent = accepted;
  loops->intra_loops.push_back(
      {{agent, 0}, {agent, 1}, Eigen::Isometry3d::Identity()});
  loops->inter_loops.push_back(
      {{agent, 1}, {agent, 0}, Eigen::Isometry3d::Identity()});
  contexts.front().loop_output = loops;
  payload->contexts = std::move(contexts);
  state->payload = payload;

  VisualizationProjector projector;
  projector.Publish(state, VisualizationPhase::kLoopDetection, false);
  auto result = projector.Project({agent, false});
  Check(result && result.Value().phase ==
                      VisualizationPhase::kLoopDetection &&
            result.Value().poses.front().transform.translation().x() == 10.0F,
        "LoopDetect snapshot uses the accepted candidate frame");
  const auto& edges = result.Value().edges;
  Check(std::count_if(edges.begin(), edges.end(), [](const auto& edge) {
          return edge.type == VisualizationEdgeType::kIntraLoop;
        }) == 1 &&
            std::count_if(edges.begin(), edges.end(), [](const auto& edge) {
              return edge.type == VisualizationEdgeType::kInterLoop;
            }) == 1,
        "LoopDetect snapshot retains intra and inter candidates");
}

void TestDataLoadCandidatesAccumulateAndRejectStaleCallbacks() {
  VisualizationProjector projector;
  auto state = MakeState({}, 7);
  projector.Publish(state, VisualizationPhase::kOptimization, false);
  const auto first_raw = state->payload->database->raw_data.at(Id("agent"));
  projector.PublishDataLoadCandidate(7, Id("agent"), first_raw);

  auto second_raw = std::make_shared<AgentRawData>(*first_raw);
  second_raw->agent_id = Id("second");
  projector.PublishDataLoadCandidate(7, Id("second"), second_raw);
  auto first = projector.Project({Id("agent"), false});
  auto second = projector.Project({Id("second"), false});
  Check(first && second &&
            first.Value().phase == VisualizationPhase::kDataLoad &&
            second.Value().phase == VisualizationPhase::kDataLoad,
        "DataLoad read model accumulates each completed candidate agent");

  projector.RollbackDataLoadCandidate(7);
  auto rolled_back = projector.Project({Id("agent"), false});
  Check(rolled_back &&
            rolled_back.Value().phase == VisualizationPhase::kOptimization &&
            !projector.Project({Id("second"), false}),
        "failed DataLoad restores the previous committed read model");
  projector.PublishDataLoadCandidate(7, Id("second"), second_raw);

  projector.Publish(MakeState({}, 8), VisualizationPhase::kOptimization,
                    false);
  projector.PublishDataLoadCandidate(7, Id("second"), second_raw);
  auto committed = projector.Project({Id("agent"), false});
  Check(committed && committed.Value().revision == 8 &&
            committed.Value().phase == VisualizationPhase::kOptimization &&
            !projector.Project({Id("second"), false}),
        "late candidate callbacks cannot replace a newer committed snapshot");
}

void TestFinalMapIntensityAndCache() {
  const fs::path root =
      fs::temp_directory_path() / "open_lmm_visualization_projector";
  fs::remove_all(root);
  fs::create_directories(root);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  cloud.push_back(pcl::PointXYZI{1.0F, 2.0F, 3.0F, 0.75F});
  cloud.push_back(pcl::PointXYZI{1.1F, 2.0F, 3.0F, 0.25F});
  const fs::path map_path = root / "global_map_agent.pcd";
  Check(pcl::io::savePCDFileBinary(map_path.string(), cloud) == 0,
        "write final map fixture");

  VisualizationProjector projector;
  projector.Publish(MakeState(root, 9), VisualizationPhase::kMapUpdate, true);
  auto metadata = projector.Project({Id("agent"), false});
  Check(metadata && metadata.Value().points.empty() &&
            metadata.Value().point_kind ==
                VisualizationPointKind::kFinalStaticMap &&
            metadata.Value().points_available && !metadata.Value().map_available,
        "MapUpdate metadata advertises lazy points without claiming a loaded map");
  auto first = projector.Project({Id("agent"), true, 10.0F, 1});
  Check(first.IsOk(), "MapUpdate loads the committed PCD");
  Check(first.Value().map_available, "MapUpdate marks the loaded final map");
  Check(first.Value().points.size() == 1 &&
            first.Value().displayed_point_count == 1 &&
            first.Value().source_point_count == 2,
        "MapUpdate obeys the bounded visualization query");
  Check(first.Value().points.front().intensity == 0.75F ||
            first.Value().points.front().intensity == 0.25F,
        "MapUpdate preserves committed intensity semantics");
  fs::remove(map_path);
  auto cached = projector.Project({Id("agent"), true, 10.0F, 1});
  Check(cached && cached.Value().points.size() == 1,
        "MapUpdate point payload is cached for the committed revision");
  fs::remove_all(root);
}

void TestFinalMapBoundIsDeterministic() {
  const fs::path root =
      fs::temp_directory_path() / "open_lmm_visualization_final_bound";
  fs::remove_all(root);
  fs::create_directories(root);
  pcl::PointCloud<pcl::PointXYZI> cloud;
  for (int index = 0; index != 200; ++index) {
    cloud.push_back(pcl::PointXYZI{static_cast<float>(index) * 0.05F,
                                  static_cast<float>(index % 7) * 0.2F,
                                  0.0F, static_cast<float>(index)});
  }
  const fs::path map_path = root / "global_map_agent.pcd";
  Check(pcl::io::savePCDFileBinary(map_path.string(), cloud) == 0,
        "write bounded final map fixture");

  VisualizationProjector first_projector;
  first_projector.Publish(MakeState(root, 11),
                          VisualizationPhase::kMapUpdate, true);
  const VisualizationQuery query{Id("agent"), true, 0.1F, 17};
  auto first = first_projector.Project(query);
  Check(first && first.Value().source_point_count == 200 &&
            first.Value().displayed_point_count == 17 &&
            first.Value().points.size() == 17,
        "final map never exceeds the requested point bound");

  VisualizationProjector second_projector;
  second_projector.Publish(MakeState(root, 11),
                           VisualizationPhase::kMapUpdate, true);
  auto second = second_projector.Project(query);
  Check(second && second.Value().points.size() == first.Value().points.size(),
        "second bounded projection succeeds");
  for (std::size_t index = 0; index < first.Value().points.size(); ++index) {
    const auto& left = first.Value().points[index];
    const auto& right = second.Value().points[index];
    Check(left.x == right.x && left.y == right.y && left.z == right.z &&
              left.intensity == right.intensity,
          "bounded final map selection is deterministic");
  }
  fs::remove_all(root);
}

void TestFinalMapsRemainAvailableForEveryAgent() {
  const fs::path root =
      fs::temp_directory_path() / "open_lmm_visualization_multi_agent";
  fs::remove_all(root);
  fs::create_directories(root);

  auto state = MakeState(root, 10);
  auto payload = std::make_shared<RuntimePayload>(*state->payload);
  auto database = std::make_shared<SharedDatabase>(*payload->database);
  auto second_raw = std::make_shared<AgentRawData>(
      *database->raw_data.at(Id("agent")));
  second_raw->agent_id = Id("second");
  database->raw_data.emplace(Id("second"), second_raw);
  payload->database = database;
  AgentPipelineCtx second_context;
  second_context.agent.id = Id("second");
  second_context.raw_data = second_raw;
  payload->contexts.push_back(std::move(second_context));
  state->payload = payload;
  state->ordered_agents.push_back(Id("second"));

  pcl::PointCloud<pcl::PointXYZI> first_cloud;
  first_cloud.push_back(pcl::PointXYZI{1.0F, 0.0F, 0.0F, 0.0F});
  pcl::PointCloud<pcl::PointXYZI> second_cloud;
  second_cloud.push_back(pcl::PointXYZI{2.0F, 0.0F, 0.0F, 1.0F});
  Check(pcl::io::savePCDFileBinary(
            (root / "global_map_agent.pcd").string(), first_cloud) == 0 &&
            pcl::io::savePCDFileBinary(
                (root / "global_map_second.pcd").string(), second_cloud) == 0,
        "write multi-agent final map fixtures");

  VisualizationProjector projector;
  projector.Publish(state, VisualizationPhase::kMapUpdate, true);
  auto first = projector.Project({Id("agent"), true});
  auto second = projector.Project({Id("second"), true});
  Check(first && second && first.Value().points.size() == 1 &&
            second.Value().points.size() == 1 &&
            first.Value().points.front().x == 1.0F &&
            second.Value().points.front().x == 2.0F,
        "MapUpdate publishes every agent map independently");
  fs::remove_all(root);
}

}  // namespace

int main() {
  TestDataLoadIsLazyAndBounded();
  TestCancelledProjectionCannotPopulateCache();
  TestQueryShapeCacheIsBounded();
  TestLoopCandidatesAndCandidateFrameSurvive();
  TestDataLoadCandidatesAccumulateAndRejectStaleCallbacks();
  TestFinalMapIntensityAndCache();
  TestFinalMapBoundIsDeterministic();
  TestFinalMapsRemainAvailableForEveryAgent();
  return 0;
}
