#include <open_lmm/server/runtime_client.hpp>

#include "support/runtime/runtime_config_fixture.hpp"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <type_traits>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

void TestForwardingMoveAndClose() {
  static_assert(std::is_move_constructible_v<RuntimeClient>);
  static_assert(std::is_nothrow_move_assignable_v<RuntimeClient>);
  static_assert(!std::is_copy_constructible_v<RuntimeClient>);

  const auto root = fs::temp_directory_path() /
                    "open_lmm_runtime_client_contract";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  test::WriteRuntimeConfigFixture(root / "config", root / "data",
                                  root / "output");

  RuntimeClient source(1);
  Check(!source.IsOpen() && source.Open({root / "config", "client"}).IsOk(),
        "public client forwards Open to the one runtime service");
  RuntimeClient moved(std::move(source));
  Check(moved.IsOpen(), "move construction transfers the PImpl runtime");
  const auto snapshot = moved.Snapshot();
  const auto descriptors = moved.NodeDescriptors();
  Check(snapshot && snapshot.Value().pipeline.agents.size() == 1 &&
            descriptors && !descriptors.Value().empty(),
        "public query types forward through the client façade");

  const auto job = moved.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job && moved.Wait(job.Value()).IsOk(),
        "public command and wait forward through one job namespace");

  RuntimeClient destination(1);
  destination = std::move(moved);
  Check(destination.IsOpen() && destination.Close().IsOk() &&
            !destination.IsOpen() && destination.Close().IsOk(),
        "move assignment transfers ownership and Close is idempotent");
  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestForwardingMoveAndClose();
  std::cout << "runtime client contract tests passed\n";
  return 0;
}
