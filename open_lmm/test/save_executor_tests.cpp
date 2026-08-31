#include <runtime/execution/stages/save_executor.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <string>

namespace {

namespace fs = std::filesystem;
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

std::shared_ptr<const RuntimeState> State(const std::vector<AgentId>& agents) {
  auto database = std::make_shared<SharedDatabase>();
  auto payload = std::make_shared<RuntimePayload>();
  payload->database = database;
  for (const AgentId& agent : agents) {
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->emplace_back(static_cast<float>(agent.Value().front()), 2.0F, 3.0F,
                       4.0F);
    auto raw = std::make_shared<AgentRawData>();
    raw->agent_id = agent;
    raw->odom_poses = {Eigen::Isometry3d::Identity()};
    raw->filtered_scans = {scan};
    database->raw_data.emplace(agent, raw);
    payload->resident_memory_reservations.emplace(
        agent, std::make_shared<MemoryReservation>());

    auto optimized = std::make_shared<AgentOptimizedData>();
    optimized->agent_id = agent;
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = static_cast<double>(agent.Value().front());
    optimized->optimized_poses.emplace_back(0, pose);
    database->optimized_data.emplace(agent, optimized);
  }
  auto state = std::make_shared<RuntimeState>();
  state->revision = 5;
  state->ordered_agents = agents;
  state->payload = payload;
  return state;
}

void ReadyArtifacts(ArtifactRepository& artifacts,
                    const std::vector<AgentId>& agents) {
  artifacts.RegisterAgents(agents);
  std::vector<ArtifactMetadata> ready;
  uint64_t revision = 1;
  for (const AgentId& agent : agents) {
    for (ArtifactType type : {ArtifactType::kRawData,
                              ArtifactType::kOptimizedPoses}) {
      ready.push_back({{type, agent}, ArtifactState::kReady, revision++,
                       "fixture"});
    }
  }
  artifacts.Restore(ready);
}

fs::path TemporaryDirectory(const char* label) {
  const fs::path path = fs::temp_directory_path() /
      (std::string("open_lmm_save_executor_") + label + "_" +
       std::to_string(std::random_device{}()));
  fs::create_directories(path);
  return path;
}

bool HasArtifact(const ArtifactRepository& artifacts, ArtifactType type,
                 const AgentId& agent) {
  for (const auto& item : artifacts.Snapshot()) {
    if (item.key.type == type && item.key.agent == agent &&
        item.state == ArtifactState::kReady &&
        !item.external_path.empty() && !item.fingerprint.empty()) {
      return true;
    }
  }
  return false;
}

bool SameArtifacts(const std::vector<ArtifactMetadata>& lhs,
                   const std::vector<ArtifactMetadata>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    const auto& a = lhs[index];
    const auto& b = rhs[index];
    if (a.key != b.key || a.state != b.state || a.revision != b.revision ||
        a.producer != b.producer || a.detail != b.detail ||
        a.external_path != b.external_path ||
        a.fingerprint != b.fingerprint) {
      return false;
    }
  }
  return true;
}

void TestStagePreparesPoseAndSkipsFallback() {
  const std::vector<AgentId> agents{Id("A"), Id("B")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, agents);
  const fs::path output = TemporaryDirectory("skip");
  {
    PendingOutputSet pending;
    SaveExecutor executor;
    auto prepared = executor.Prepare(
        {*state, SaveExecutionMode::kStage, output, true,
         std::make_shared<CancellationToken>()},
        pending, artifacts);
    Check(prepared.IsOk(), "pose-only Save preparation must succeed");
    Check(prepared.Value().pose_agents == agents &&
              prepared.Value().fallback_map_agents.empty() &&
              prepared.Value().fallback_map_skipped,
          "PoseSave must run once for all ready agents and skip fallback maps");
    Check(pending.Files().size() == agents.size(),
          "all ready pose files must share one PendingOutputSet");
    for (const AgentId& agent : agents) {
      Check(HasArtifact(artifacts, ArtifactType::kPoseFile, agent),
            "pose artifact candidate must be recorded");
    }
    // Destruction without coordinator commit rolls every temporary back.
  }
  for (const AgentId& agent : agents) {
    Check(!fs::exists(output / ("optimized_poses_" + agent.Value() + ".txt")) &&
              !fs::exists(output /
                          ("optimized_poses_" + agent.Value() + ".txt.tmp")),
          "uncommitted Save preparation must leave no output");
  }
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

void TestPoseNodeOnly() {
  const std::vector<AgentId> agents{Id("A"), Id("B")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, agents);
  const fs::path output = TemporaryDirectory("pose_node");
  PendingOutputSet pending;
  SaveExecutor executor;
  auto prepared = executor.Prepare(
      {*state, SaveExecutionMode::kPoseSave, output, false,
       std::make_shared<CancellationToken>()},
      pending, artifacts);
  Check(prepared.IsOk() && prepared.Value().pose_agents == agents &&
            prepared.Value().fallback_map_agents.empty() &&
            !prepared.Value().fallback_map_skipped &&
            pending.Files().size() == agents.size(),
        "PoseSave node command must prepare only pose outputs");
  for (const AgentId& agent : agents) {
    Check(HasArtifact(artifacts, ArtifactType::kPoseFile, agent) &&
              !HasArtifact(artifacts, ArtifactType::kGlobalMap, agent),
          "PoseSave node command must not mutate fallback artifacts");
  }
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

void TestFallbackNodeSkipHasNoChanges() {
  const std::vector<AgentId> agents{Id("A"), Id("B")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, agents);
  const auto before = artifacts.Snapshot();
  const fs::path output = TemporaryDirectory("fallback_skip");
  PendingOutputSet pending;
  SaveExecutor executor;
  auto prepared = executor.Prepare(
      {*state, SaveExecutionMode::kFallbackMapSave, output, true,
       std::make_shared<CancellationToken>()},
      pending, artifacts);
  Check(prepared.IsOk() && prepared.Value().pose_agents.empty() &&
            prepared.Value().fallback_map_agents.empty() &&
            prepared.Value().fallback_map_skipped,
        "FallbackMapSave node must explicitly report MapUpdate skip");
  Check(pending.Files().empty() && SameArtifacts(artifacts.Snapshot(), before),
        "skipped FallbackMapSave node must not change files or artifacts");
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

void TestFallbackNodeAndAtomicCommit() {
  const std::vector<AgentId> agents{Id("A"), Id("B")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, agents);
  const fs::path output = TemporaryDirectory("fallback");
  PendingOutputSet pending;
  SaveExecutor executor;
  auto prepared = executor.Prepare(
      {*state, SaveExecutionMode::kFallbackMapSave, output, false,
       std::make_shared<CancellationToken>()},
      pending, artifacts);
  Check(prepared.IsOk() && prepared.Value().pose_agents.empty() &&
            prepared.Value().fallback_map_agents == agents &&
            !prepared.Value().fallback_map_skipped,
        "FallbackMapSave node must prepare maps for all ready agents");
  Check(pending.Files().size() == agents.size(),
        "FallbackMapSave node must prepare only map files");
  auto committed = pending.Commit();
  Check(committed.IsOk(), "complete Save file set must commit atomically");
  for (const AgentId& agent : agents) {
    Check(!fs::exists(output /
                      ("optimized_poses_" + agent.Value() + ".txt")) &&
              fs::is_regular_file(output /
                                  ("global_map_" + agent.Value() + ".pcd")),
          "committed FallbackMapSave node must publish only map outputs");
    Check(HasArtifact(artifacts, ArtifactType::kGlobalMap, agent) &&
              HasArtifact(artifacts, ArtifactType::kPcdFile, agent),
          "fallback map artifact candidates must be recorded");
  }
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

void TestOnlyReadyAgentsArePrepared() {
  const std::vector<AgentId> agents{Id("A"), Id("B")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, {agents.front()});
  artifacts.RegisterAgents(agents);
  const fs::path output = TemporaryDirectory("ready");
  PendingOutputSet pending;
  SaveExecutor executor;
  auto prepared = executor.Prepare(
      {*state, SaveExecutionMode::kStage, output, false,
       std::make_shared<CancellationToken>()},
      pending, artifacts);
  Check(prepared.IsOk() &&
            prepared.Value().pose_agents ==
                std::vector<AgentId>{agents.front()} &&
            prepared.Value().fallback_map_agents ==
                std::vector<AgentId>{agents.front()} &&
            pending.Files().size() == 2,
        "session Save must select only agents with ready requirements");
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

void TestRecoveryRequiredPropagatesFromFileSet() {
  const std::vector<AgentId> agents{Id("A")};
  const auto state = State(agents);
  ArtifactRepository artifacts;
  ReadyArtifacts(artifacts, agents);
  const fs::path output = TemporaryDirectory("recovery");
  const fs::path original_map = output / "global_map_A.pcd";
  fs::create_directories(original_map);
  std::ofstream(original_map / "original") << "preserve";

  PendingOutputSet pending;
  SaveExecutor executor;
  auto prepared = executor.Prepare(
      {*state, SaveExecutionMode::kStage, output, false,
       std::make_shared<CancellationToken>()},
      pending, artifacts);
  Check(prepared.IsOk(), "recovery fixture preparation must succeed");
  auto committed = pending.Commit();
  Check(committed.IsOk() && committed.Value().recovery_required.has_value() &&
            fs::is_regular_file(original_map),
        "Save commit must remain successful after post-install cleanup failure");
  Check(fs::exists(output / "global_map_A.pcd.open_lmm_backup/original"),
        "recovery-required Save must preserve the original backup");
  bool manifest = false;
  for (const auto& item : fs::directory_iterator(output)) {
    if (item.path().filename().string().starts_with(
            ".open_lmm_recovery_")) {
      manifest = true;
      break;
    }
  }
  Check(manifest, "post-commit cleanup failure must retain a manifest");
  std::error_code ignored;
  fs::remove_all(output, ignored);
}

}  // namespace

int main() {
  Check(ExecutionSpecFor(NodeId::kPoseSave).scope ==
            ExecutionScope::kRuntime,
        "PoseSave execution scope must remain session-wide");
  TestStagePreparesPoseAndSkipsFallback();
  TestPoseNodeOnly();
  TestFallbackNodeSkipHasNoChanges();
  TestFallbackNodeAndAtomicCommit();
  TestOnlyReadyAgentsArePrepared();
  TestRecoveryRequiredPropagatesFromFileSet();
  std::cout << "SaveExecutor tests passed\n";
  return EXIT_SUCCESS;
}
