#include <config/bootstrap/bootstrap_config.hpp>
#include <runtime/composition/runtime_bootstrapper.hpp>
#include <runtime/execution/stages/stage_coordinator.hpp>

#include "support/runtime/runtime_config_fixture.hpp"
#include "support/check.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

ExecutionContext Context(uint64_t revision) {
  return {std::make_shared<CancellationToken>(),
          std::make_shared<AlignmentFeedbackBroker>(), revision, {}};
}

std::string Read(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  Check(input.is_open(), "expected output exists and is readable");
  return {std::istreambuf_iterator<char>(input), {}};
}

void TestStateAndFileBarrier() {
  const auto root = fs::temp_directory_path() /
                    "open_lmm_stage_coordinator_contract";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  test::WriteRuntimeConfigFixture(root / "config", root / "data",
                                  root / "output");
  auto config = LoadBootstrapConfig(root / "config");
  Check(config.IsOk(), "coordinator bootstrap fixture validates");
  auto bootstrapped = RuntimeBootstrapper().Bootstrap(
      {std::move(config).Value(), std::nullopt, {}});
  Check(bootstrapped.IsOk(), "coordinator runtime bootstrap succeeds");
  auto runtime = std::move(bootstrapped).Value();
  const auto initial = runtime.initial_state;
  RuntimeStateStore store;
  store.Initialize(initial, runtime.recovery_required);
  OutputRepository outputs;
  auto governor = std::make_shared<ResourceGovernor>(
      runtime.suggested_resource_budget);
  StageCoordinator coordinator(store, outputs, governor, runtime.algorithms);

  auto first_context = Context(initial->revision);
  Check(coordinator.ExecuteStage(initial, StageId::kDataLoad, first_context).IsOk(),
        "DataLoad candidate commits through the coordinator barrier");
  const auto loaded = store.Snapshot();
  Check(loaded && loaded->revision == initial->revision + 1 &&
            loaded->payload && loaded->payload->database &&
            loaded->payload->database->raw_data.contains(
                AgentId::Parse("agent1").Value()),
        "committed authority contains the validated DataLoad payload");

  const auto stale = coordinator.ExecuteStage(
      initial, StageId::kDataLoad, first_context);
  Check(!stale && store.Snapshot().get() == loaded.get(),
        "stale candidate cannot replace the newer committed authority");

  auto cancelled_context = Context(loaded->revision);
  bool preview_seen = false;
  const auto raw = loaded->payload->database->raw_data.at(
      AgentId::Parse("agent1").Value());
  StageCoordinator cancelling_coordinator(
      store, outputs, governor, runtime.algorithms,
      [&](uint64_t revision, const AgentId&, const AgentRawDataHandle&,
          const VisualizationPointPreviewHandle&) {
        Check(revision == loaded->revision &&
                  store.Snapshot().get() == loaded.get(),
              "DataLoad preview does not publish the candidate");
        preview_seen = true;
        cancelled_context.cancellation->Request();
      });
  const auto cancelled_load = cancelling_coordinator.ExecuteStage(
      loaded, StageId::kDataLoad, cancelled_context);
  Check(preview_seen && !cancelled_load &&
            cancelled_load.GetError().code == Error::Code::kCancelled &&
            store.Snapshot().get() == loaded.get() &&
            loaded->payload->database->raw_data.at(
                AgentId::Parse("agent1").Value()).get() == raw.get(),
        "cancellation during candidate work preserves committed raw ownership");

  // The fixture disables MapUpdate, so this reaches transaction finalization
  // directly, without an algorithm consuming the cancellation first.
  Check(!loaded->config->root.enable_map_updater,
        "fixture exercises the skipped MapUpdate commit path");
  const auto cancelled_commit = coordinator.ExecuteStage(
      loaded, StageId::kMapUpdate, cancelled_context);
  Check(!cancelled_commit &&
            cancelled_commit.GetError().code == Error::Code::kCancelled &&
            store.Snapshot().get() == loaded.get(),
        "candidate finalization observes cancellation before state publication");

  auto aligned_context = Context(loaded->revision);
  Check(coordinator.ExecuteStage(
            loaded, StageId::kAlignment, aligned_context).IsOk(),
        "alignment candidate commits before save preparation");
  const auto aligned = store.Snapshot();
  const auto output = aligned->config->root.output_directory;
  const auto poses = output / "optimized_poses_agent1.txt";
  const auto map = output / "global_map_agent1.pcd";
  const fs::path pose_tmp = poses.string() + ".tmp";
  const fs::path map_tmp = map.string() + ".tmp";
  const fs::path pose_backup = poses.string() + ".open_lmm_backup";
  const fs::path map_backup = map.string() + ".open_lmm_backup";
  std::ofstream(poses) << "previous poses";
  std::ofstream(map) << "previous map";
  const auto check_preserved = [&] {
    Check(store.Snapshot().get() == aligned.get() &&
              !store.AuthoritySnapshot().recovery_required &&
              Read(poses) == "previous poses" && Read(map) == "previous map",
          "failed save preserves committed authority and both previous files");
    Check(!fs::exists(pose_tmp) && !fs::exists(pose_backup),
          "failed save removes its staged pose output without installing it");
  };

  auto cancelled_save_context = Context(aligned->revision);
  cancelled_save_context.cancellation->Request();
  const auto cancelled_save = coordinator.ExecuteStage(
      aligned, StageId::kSave, cancelled_save_context);
  Check(!cancelled_save &&
            cancelled_save.GetError().code == Error::Code::kCancelled,
        "pre-commit save cancellation reports cancellation");
  check_preserved();

  // A nonempty temporary directory fails deterministically even as root,
  // after the pose temporary has already been prepared.
  fs::create_directories(map_tmp / "blocker");
  const auto preparation_failure = coordinator.ExecuteStage(
      aligned, StageId::kSave, Context(aligned->revision));
  Check(!preparation_failure &&
            preparation_failure.GetError().code == Error::Code::kIoError,
        "map preparation failure rejects the complete save candidate");
  check_preserved();
  fs::remove_all(map_tmp);

  std::ofstream(map_backup) << "retained backup";
  const auto commit_failure = coordinator.ExecuteStage(
      aligned, StageId::kSave, Context(aligned->revision));
  Check(!commit_failure &&
            commit_failure.GetError().code == Error::Code::kIoError &&
            Read(map_backup) == "retained backup" && !fs::exists(map_tmp),
        "stale backup rejects the file barrier and cleans staged outputs");
  check_preserved();
  fs::remove(map_backup);

  auto save_context = Context(aligned->revision);
  Check(coordinator.ExecuteStage(
            aligned, StageId::kSave, save_context).IsOk(),
        "Save file set and runtime state cross one coordinator barrier");
  const auto saved = store.Snapshot();
  bool pose_published = false;
  for (const auto& entry : fs::recursive_directory_iterator(root / "output")) {
    if (entry.path().filename() == "optimized_poses_agent1.txt") {
      pose_published = true;
      break;
    }
  }
  Check(saved && saved->revision == aligned->revision + 1 &&
            pose_published,
        "successful file publication advances committed state exactly once");

  const auto saved_poses = Read(poses);
  const auto saved_map = Read(map);
  Check(saved_poses != "previous poses" && saved_map != "previous map",
        "successful save replaces both previous outputs");
  const auto stale_save = coordinator.ExecuteStage(
      aligned, StageId::kSave, Context(aligned->revision));
  Check(!stale_save &&
            stale_save.GetError().Message().find("revision conflict") !=
                std::string::npos && store.Snapshot().get() == saved.get() &&
            Read(poses) == saved_poses && Read(map) == saved_map &&
            !fs::exists(pose_tmp) && !fs::exists(map_tmp) &&
            !fs::exists(pose_backup) && !fs::exists(map_backup),
        "stale save rejects publication and discards both prepared files");

  // Reuse the storage contract's cleanup-fault fixture: replacing a nonempty
  // directory succeeds, but removing its backup fails after all files install.
  fs::remove(map);
  fs::create_directories(map / "retained-child");
  const auto recovery_save = coordinator.ExecuteStage(
      saved, StageId::kSave, Context(saved->revision));
  const auto authority = store.AuthoritySnapshot();
  Check(recovery_save.IsOk() && authority.state &&
            authority.state->revision == saved->revision + 1 &&
            authority.recovery_required &&
            authority.recovery_required->severity == Error::Severity::kFatalRuntime &&
            authority.recovery_required->context.runtime_revision ==
                authority.state->revision &&
            Read(poses) == saved_poses && Read(map) == saved_map &&
            fs::is_directory(map_backup / "retained-child"),
        "post-commit cleanup failure publishes new state, files and recovery health");
  bool manifest_found = false;
  for (const auto& entry : fs::directory_iterator(output)) {
    if (entry.path().filename().string().starts_with(".open_lmm_recovery_")) {
      manifest_found = true;
    }
  }
  Check(manifest_found, "cleanup failure retains a recovery manifest");
  const auto blocked = coordinator.ExecuteStage(
      authority.state, StageId::kSave, Context(authority.state->revision));
  const auto after_blocked = store.AuthoritySnapshot();
  Check(!blocked && blocked.GetError().severity == Error::Severity::kFatalRuntime &&
            after_blocked.state.get() == authority.state.get() &&
            after_blocked.recovery_required.get() == authority.recovery_required.get() &&
            Read(poses) == saved_poses && Read(map) == saved_map &&
            !fs::exists(pose_tmp) && !fs::exists(map_tmp) &&
            fs::is_directory(map_backup / "retained-child"),
        "recovery health stays authoritative and blocks subsequent file publication");
  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestStateAndFileBarrier();
  std::cout << "stage coordinator contract tests passed\n";
  return 0;
}
