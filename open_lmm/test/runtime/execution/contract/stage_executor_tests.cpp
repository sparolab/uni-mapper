#include <config/bootstrap/bootstrap_config.hpp>
#include <runtime/execution/stage_executor.hpp>

#include "support/runtime/runtime_config_fixture.hpp"
#include "support/check.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

void TestCommandAndQueryFacade() {
  const auto root = fs::temp_directory_path() / "open_lmm_stage_executor_contract";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  test::WriteRuntimeConfigFixture(root / "config", root / "data",
                                  root / "output");
  auto bootstrap = LoadBootstrapConfig(root / "config");
  Check(bootstrap.IsOk(), "stage executor bootstrap fixture validates");
  StageExecutor executor(std::move(bootstrap).Value());
  Check(executor.ValidateReady().IsOk(),
        "stage executor validates its private runtime owners");
  const auto initial = executor.Snapshot();
  Check(initial.revision == 1 && initial.ordered_agents.size() == 1,
        "query façade exposes the bootstrapped committed revision");

  ExecutionContext context;
  context.cancellation = std::make_shared<CancellationToken>();
  context.alignment_feedback = std::make_shared<AlignmentFeedbackBroker>();
  context.base_revision = initial.revision;
  Check(!executor.Execute({}, context),
        "malformed command is rejected before candidate mutation");

  const auto receipt = executor.Execute(
      ExecutionCommand::Stage(StageId::kDataLoad), context);
  const auto committed = executor.Snapshot();
  Check(receipt && receipt.Value().base_revision == initial.revision &&
            receipt.Value().committed_revision == initial.revision + 1 &&
            committed.revision == receipt.Value().committed_revision,
        "command receipt and query authority publish the same revision");
  const auto visualization = executor.Visualization(
      {committed.ordered_agents.front(), false});
  Check(visualization && visualization.Value().revision == committed.revision,
        "query façade projects only the committed runtime revision");

  auto recovery = std::make_shared<Error>(Error::IoError("executor recovery"));
  recovery->MarkFatalRuntime().WithRuntimeRevision(committed.revision);
  executor.RecordRecoveryRequired(recovery);
  context.base_revision = committed.revision;
  const auto blocked = executor.Execute(
      ExecutionCommand::Stage(StageId::kDataLoad), context);
  Check(!blocked && executor.Snapshot().recovery_required == recovery &&
            executor.Snapshot().revision == committed.revision,
        "recovery authority gates mutation without changing committed state");
  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestCommandAndQueryFacade();
  std::cout << "stage executor contract tests passed\n";
  return 0;
}
