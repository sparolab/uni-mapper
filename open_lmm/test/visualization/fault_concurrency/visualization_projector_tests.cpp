#include <visualization/projection/visualization_projector.hpp>
#include <runtime/state/runtime_state.hpp>

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
                                        uint64_t revision = 7,
                                        float preview_voxel_size_m = 0.2F) {
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
  config->root.save_voxel_size = preview_voxel_size_m;
  auto state = std::make_shared<RuntimeState>();
  state->revision = revision;
  state->config = config;
  state->ordered_agents = {agent};
  state->payload = payload;
  return state;
}

VisualizationSource MakeSource(const std::shared_ptr<RuntimeState>& state) {
  VisualizationSource source;
  source.revision = state->revision;
  source.output_directory = state->config->root.output_directory;
  source.preview_voxel_size_m =
      static_cast<float>(state->config->root.save_voxel_size);
  for (const auto& entry : state->payload->database->raw_data) {
    const auto& agent = entry.first;
    const auto& raw = entry.second;
    const auto optimized = state->payload->database->optimized_data.find(agent);
    const auto context = std::find_if(
        state->payload->contexts.begin(), state->payload->contexts.end(),
        [agent](const AgentPipelineCtx& item) {
          return item.agent.id == agent;
        });
    source.agents.push_back(
        {agent, raw,
         optimized == state->payload->database->optimized_data.end()
             ? nullptr
             : optimized->second,
         context == state->payload->contexts.end() ? nullptr
                                                   : context->loop_output});
  }
  return source;
}

void TestDataLoadIsLazyAndVoxelComplete() {
  VisualizationProjector projector;
  auto state = MakeState({});
  projector.Publish(MakeSource(state), VisualizationPhase::kDataLoad, false);
  auto metadata = projector.Project({Id("agent"), false, 0.4F, 2});
  Check(metadata && metadata.Value().phase == VisualizationPhase::kDataLoad &&
            metadata.Value().pose_kind == VisualizationPoseKind::kOdometry &&
            metadata.Value().poses.size() == 2 &&
            metadata.Value().points.empty() &&
            metadata.Value().points_available &&
            !metadata.Value().points_complete,
        "DataLoad metadata exposes odometry without materializing points");

  auto full = projector.Project({Id("agent"), true, 0.4F, 1});
  Check(full && full.Value().points_complete &&
            full.Value().points.size() == 2 &&
            full.Value().displayed_point_count == 2 &&
            full.Value().source_point_count == 4 &&
            full.Value().point_kind ==
                VisualizationPointKind::kFilteredScanPreview,
        "DataLoad preview returns every voxel and accounts for source points");
  auto warm = projector.Project({Id("agent"), true, 0.4F, 1});
  const auto diagnostics = projector.Diagnostics();
  Check(warm && diagnostics.entries == 1 && diagnostics.bytes > 0 &&
            diagnostics.hits == 1 && diagnostics.misses == 1 &&
            diagnostics.insertions == 1 && diagnostics.evictions == 0 &&
            diagnostics.clears == 0,
        "projector diagnostics distinguishes cold insertion from warm hit");
}

void TestCancelledProjectionCannotPopulateCache() {
  VisualizationProjector projector;
  projector.Publish(MakeSource(MakeState({})), VisualizationPhase::kDataLoad,
                    false);
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
  projector.Publish(MakeSource(MakeState({})), VisualizationPhase::kDataLoad,
                    false);
  for (std::size_t shape = 1; shape <= 40; ++shape) {
    auto result = projector.Project(
        {Id("agent"), true, static_cast<float>(shape) / 1000.0F,
         shape + 1});
    Check(result.IsOk(), "variant visualization query succeeds");
  }
  Check(projector.PointCacheEntryCount() <= 16,
        "variant query shapes retain at most sixteen cache entries");
  Check(projector.PointCacheBytes() > 0,
        "variant query shapes retain point-cache telemetry");
  const auto bounded = projector.Diagnostics();
  Check(bounded.entries == 16 && bounded.misses == 40 &&
            bounded.insertions == 40 && bounded.evictions == 24,
        "projector diagnostics accounts for bounded LRU evictions");
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
  projector.Publish(MakeSource(state), VisualizationPhase::kLoopDetection,
                    false);
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
  projector.Publish(MakeSource(state), VisualizationPhase::kOptimization,
                    false);
  const auto first_raw = state->payload->database->raw_data.at(Id("agent"));
  projector.PublishDataLoadCandidate(7, Id("agent"), first_raw);
  auto first_points = projector.Project({Id("agent"), true});
  Check(first_points && projector.PointCacheEntryCount() == 1,
        "first DataLoad candidate populates its point cache");

  auto second_raw = std::make_shared<AgentRawData>(*first_raw);
  second_raw->agent_id = Id("second");
  projector.PublishDataLoadCandidate(7, Id("second"), second_raw);
  Check(projector.PointCacheEntryCount() == 1,
        "publishing another agent preserves existing point caches");
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

  projector.Publish(MakeSource(MakeState({}, 8)),
                    VisualizationPhase::kOptimization, false);
  projector.PublishDataLoadCandidate(7, Id("second"), second_raw);
  auto committed = projector.Project({Id("agent"), false});
  Check(committed && committed.Value().revision == 8 &&
            committed.Value().phase == VisualizationPhase::kOptimization &&
            !projector.Project({Id("second"), false}),
        "late candidate callbacks cannot replace a newer committed snapshot");
}

void TestDataLoadCandidateReusesIncrementalPreviewAcrossCommit() {
  VisualizationProjector projector;
  auto state = MakeState({}, 7);
  projector.Publish(MakeSource(state), VisualizationPhase::kOptimization,
                    false);
  const auto raw = state->payload->database->raw_data.at(Id("agent"));
  auto preview = std::make_shared<VisualizationPointPreview>();
  preview->voxel_millimeters = 200;
  preview->points.push_back({99.0F, 1.0F, 2.0F, 0.75F});
  preview->points.push_back({100.0F, 2.0F, 3.0F, 0.25F});
  preview->min_bound = Eigen::Vector3f(99.0F, 1.0F, 2.0F);
  preview->max_bound = Eigen::Vector3f(100.0F, 2.0F, 3.0F);
  preview->has_bounds = true;
  preview->source_point_count = 123;

  projector.PublishDataLoadCandidate(7, Id("agent"), raw, preview);
  auto candidate = projector.Project({Id("agent"), true});
  Check(candidate && candidate.Value().points.size() == 2 &&
            candidate.Value().points.front().x == 99.0F &&
            candidate.Value().source_point_count == 123,
        "DataLoad candidate reuses every incrementally built voxel");

  projector.Publish(MakeSource(state), VisualizationPhase::kDataLoad, false);
  auto committed = projector.Project({Id("agent"), true});
  Check(committed && committed.Value().points.size() == 2 &&
            committed.Value().points.front().x == 99.0F &&
            committed.Value().source_point_count == 123,
        "DataLoad commit retains the matching preview without rebuilding");
}

void TestAlignmentCandidatePublishesIntermediateOptimizedPosesAndRollsBack() {
  VisualizationProjector projector;
  auto state = MakeState({}, 7);
  auto payload = std::make_shared<RuntimePayload>(*state->payload);
  auto database = std::make_shared<SharedDatabase>(*payload->database);
  auto contexts = payload->contexts;
  for (const char* name : {"second", "third"}) {
    const AgentId agent = Id(name);
    auto raw = std::make_shared<AgentRawData>(
        *database->raw_data.at(Id("agent")));
    raw->agent_id = agent;
    database->raw_data.emplace(agent, raw);
    AgentPipelineCtx context;
    context.agent.id = agent;
    context.raw_data = raw;
    contexts.push_back(std::move(context));
    state->ordered_agents.push_back(agent);
  }
  const auto optimized = [](double x) {
    auto data = std::make_shared<AgentOptimizedData>();
    for (int frame = 0; frame != 2; ++frame) {
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.translation().x() = x + frame;
      data->optimized_poses.emplace_back(frame, pose);
    }
    return std::shared_ptr<const AgentOptimizedData>(std::move(data));
  };
  database->optimized_data[Id("agent")] = optimized(10.0);
  database->optimized_data[Id("second")] = optimized(20.0);
  payload->database = database;
  payload->contexts = contexts;
  state->payload = payload;

  auto committed_source = MakeSource(state);
  for (auto& agent : committed_source.agents) agent.optimized_data.reset();
  projector.Publish(std::move(committed_source),
                    VisualizationPhase::kDataLoad, false);
  projector.PublishAlignmentCandidate(MakeSource(state));

  auto first = projector.Project({Id("agent"), false});
  auto second = projector.Project({Id("second"), false});
  auto third = projector.Project({Id("third"), false});
  Check(first && second && third &&
            first.Value().pose_kind == VisualizationPoseKind::kOptimized &&
            second.Value().pose_kind == VisualizationPoseKind::kOptimized &&
            third.Value().pose_kind == VisualizationPoseKind::kOdometry &&
            first.Value().poses.front().transform.translation().x() == 10.0F &&
            second.Value().poses.front().transform.translation().x() == 20.0F,
        "Alignment candidate exposes optimized predecessors and raw follower");

  database->optimized_data[Id("agent")] = optimized(11.0);
  projector.PublishAlignmentCandidate(MakeSource(state));
  auto revised = projector.Project({Id("agent"), false});
  Check(revised &&
            revised.Value().poses.front().transform.translation().x() == 11.0F,
        "same-revision Alignment candidates replace older optimizer poses");

  projector.RollbackAlignmentCandidate(7);
  auto rolled_back = projector.Project({Id("agent"), false});
  Check(rolled_back &&
            rolled_back.Value().phase == VisualizationPhase::kDataLoad &&
            rolled_back.Value().pose_kind == VisualizationPoseKind::kOdometry &&
            rolled_back.Value().poses.front().transform.translation().x() == 0.0F,
        "failed Alignment restores the committed visualization read model");

  projector.PublishAlignmentCandidate(MakeSource(state));
  auto newer = MakeState({}, 8);
  projector.Publish(MakeSource(newer), VisualizationPhase::kOptimization,
                    false);
  projector.PublishAlignmentCandidate(MakeSource(state));
  auto retained = projector.Project({Id("agent"), false});
  Check(retained && retained.Value().revision == 8,
        "late Alignment candidate cannot replace a newer committed revision");
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
  projector.Publish(MakeSource(MakeState(root, 9, 0.01F)),
                    VisualizationPhase::kMapUpdate, true);
  auto metadata = projector.Project({Id("agent"), false});
  Check(metadata && metadata.Value().points.empty() &&
            metadata.Value().point_kind ==
                VisualizationPointKind::kFinalStaticMap &&
            metadata.Value().points_available && !metadata.Value().map_available,
        "MapUpdate metadata advertises lazy points without claiming a loaded map");
  auto first = projector.Project({Id("agent"), true});
  Check(first.IsOk(), "MapUpdate loads the committed PCD");
  Check(first.Value().map_available, "MapUpdate marks the loaded final map");
  Check(first.Value().points.size() == 2 &&
            first.Value().displayed_point_count == 2 &&
            first.Value().source_point_count == 2,
        "MapUpdate returns every voxel regardless of the legacy point limit");
  Check(std::any_of(first.Value().points.begin(), first.Value().points.end(),
                    [](const VisualizationPoint& point) {
                      return point.intensity == 0.75F;
                    }) &&
            std::any_of(first.Value().points.begin(),
                        first.Value().points.end(),
                        [](const VisualizationPoint& point) {
                          return point.intensity == 0.25F;
                        }),
        "MapUpdate preserves committed intensity semantics");
  fs::remove(map_path);
  auto cached = projector.Project({Id("agent"), true});
  Check(cached && cached.Value().points.size() == 2,
        "MapUpdate point payload is cached for the committed revision");
  fs::remove_all(root);
}

void TestFinalMapVoxelizationIsCompleteAndDeterministic() {
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
        "write complete final map fixture");

  VisualizationProjector first_projector;
  first_projector.Publish(MakeSource(MakeState(root, 11)),
                          VisualizationPhase::kMapUpdate, true);
  const VisualizationQuery query{Id("agent"), true, 0.01F, 17};
  auto first = first_projector.Project(query);
  Check(first && first.Value().source_point_count == 200 &&
            first.Value().displayed_point_count == 200 &&
            first.Value().points.size() == 200,
        "final map returns every voxel despite the legacy point limit");

  VisualizationProjector second_projector;
  second_projector.Publish(MakeSource(MakeState(root, 11)),
                           VisualizationPhase::kMapUpdate, true);
  auto second = second_projector.Project(query);
  Check(second && second.Value().points.size() == first.Value().points.size(),
        "second complete projection succeeds");
  for (std::size_t index = 0; index < first.Value().points.size(); ++index) {
    const auto& left = first.Value().points[index];
    const auto& right = second.Value().points[index];
    Check(left.x == right.x && left.y == right.y && left.z == right.z &&
              left.intensity == right.intensity,
          "complete final map voxelization is deterministic");
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
  projector.Publish(MakeSource(state), VisualizationPhase::kMapUpdate, true);
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

#ifndef OPEN_LMM_VISUALIZATION_SUITE
#define OPEN_LMM_VISUALIZATION_SUITE 0
#endif

int main() {
#if OPEN_LMM_VISUALIZATION_SUITE == 1
  TestDataLoadIsLazyAndVoxelComplete();
  TestQueryShapeCacheIsBounded();
  TestLoopCandidatesAndCandidateFrameSurvive();
  TestDataLoadCandidateReusesIncrementalPreviewAcrossCommit();
  TestFinalMapIntensityAndCache();
  TestFinalMapVoxelizationIsCompleteAndDeterministic();
  TestFinalMapsRemainAvailableForEveryAgent();
#elif OPEN_LMM_VISUALIZATION_SUITE == 2
  TestCancelledProjectionCannotPopulateCache();
  TestDataLoadCandidatesAccumulateAndRejectStaleCallbacks();
  TestAlignmentCandidatePublishesIntermediateOptimizedPosesAndRollsBack();
#else
#error "OPEN_LMM_VISUALIZATION_SUITE must select a layer suite"
#endif
  return 0;
}
