#include <config/bootstrap/bootstrap_config.hpp>
#include <runtime/composition/runtime_bootstrapper.hpp>
#include <runtime/execution/stages/stage_coordinator.hpp>

#include "support/runtime/runtime_config_fixture.hpp"
#include "support/check.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

ExecutionContext Context(uint64_t revision) {
  return {std::make_shared<CancellationToken>(),
          std::make_shared<AlignmentFeedbackBroker>(), revision, {}};
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

  auto aligned_context = Context(loaded->revision);
  Check(coordinator.ExecuteStage(
            loaded, StageId::kAlignment, aligned_context).IsOk(),
        "alignment candidate commits before save preparation");
  const auto aligned = store.Snapshot();
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
  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestStateAndFileBarrier();
  std::cout << "stage coordinator contract tests passed\n";
  return 0;
}
