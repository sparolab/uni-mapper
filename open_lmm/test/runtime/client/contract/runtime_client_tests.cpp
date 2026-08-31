#include <open_lmm/server/runtime_client.hpp>

#include <runtime/client/runtime_retirement_coordinator.hpp>
#include "support/runtime/runtime_config_fixture.hpp"

#include <atomic>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <mutex>
#include <system_error>
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
  const auto logs = moved.RecentLogs(1);
  Check(snapshot && snapshot.Value().pipeline.agents.size() == 1 &&
            descriptors && !descriptors.Value().empty() && logs,
        "public query types forward through the client façade");
  Check(!moved.RecentLogs(0) && !moved.RecentLogs(513),
        "public runtime logs remain bounded to the process ring capacity");

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

void TestRetirementCoordinatorDrainsBoundedQueue() {
  struct Owner {
    RuntimeRetirementNode node;
    std::atomic<int>* destroyed = nullptr;
  };
  auto destroy = [](void* raw) noexcept {
    auto* owner = static_cast<Owner*>(raw);
    ++*owner->destroyed;
    delete owner;
  };

  RuntimeRetirementCoordinator coordinator;
  std::atomic<int> destroyed{0};
  for (int i = 0; i < 4; ++i) {
    auto* owner = new Owner;
    owner->node.owner = owner;
    owner->node.destroy = destroy;
    owner->destroyed = &destroyed;
    coordinator.Retire(owner->node);
  }
  coordinator.WaitIdle();
  const auto diagnostics = coordinator.Diagnostics();
  Check(diagnostics.pending == 0 && diagnostics.completed == 4 &&
            diagnostics.peak_pending >= 1 && destroyed.load() == 4,
        "retirement coordinator drains every intrusive owner exactly once");

  bool launch_failed = false;
  try {
    RuntimeRetirementCoordinator failing(
        [](ThreadTask) -> std::thread {
          throw std::system_error(
              std::make_error_code(std::errc::resource_unavailable_try_again));
        });
  } catch (const std::system_error&) {
    launch_failed = true;
  }
  Check(launch_failed,
        "retirement coordinator preserves worker construction failure");
}

void TestCallbackDestructionUsesCoordinator() {
  const auto root = fs::temp_directory_path() /
                    "open_lmm_runtime_client_callback_retirement";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  test::WriteRuntimeConfigFixture(root / "config", root / "data",
                                  root / "output");

  auto& coordinator = GlobalRuntimeRetirementCoordinator();
  coordinator.WaitIdle();
  const auto completed_before = coordinator.Diagnostics().completed;

  std::mutex mutex;
  std::condition_variable changed;
  bool main_released_owner = false;
  bool callback_released_last_owner = false;
  auto client = std::make_shared<RuntimeClient>(1);
  auto callback_owner =
      std::make_shared<std::shared_ptr<RuntimeClient>>(client);
  Check(client->Open({root / "config", "callback-retirement"}).IsOk(),
        "callback retirement fixture opens");
  auto subscription = client->SubscribeEvents(
      [callback_owner, &mutex, &changed, &main_released_owner,
       &callback_released_last_owner](const ExecutionEvent& event) {
        if (event.type == EventType::kJobStarted) {
          std::unique_lock lock(mutex);
          changed.wait(lock, [&] { return main_released_owner; });
          return;
        }
        if (event.type != EventType::kJobCompleted &&
            event.type != EventType::kJobCancelled) {
          return;
        }
        callback_owner->reset();
        {
          std::lock_guard lock(mutex);
          callback_released_last_owner = true;
        }
        changed.notify_all();
      });
  Check(subscription.IsOk(), "callback retirement subscription succeeds");
  const auto job = client->Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job.IsOk(), "callback retirement job submits");
  client.reset();
  {
    std::lock_guard lock(mutex);
    main_released_owner = true;
  }
  changed.notify_all();
  {
    std::unique_lock lock(mutex);
    changed.wait(lock, [&] { return callback_released_last_owner; });
  }
  coordinator.WaitIdle();
  const auto after = coordinator.Diagnostics();
  Check(after.pending == 0 && after.completed == completed_before + 1,
        "last callback owner is retired by the bounded coordinator");

  fs::remove_all(root, ignored);
}

void TestCallbackMoveAssignmentRetiresOldImpl() {
  const auto root = fs::temp_directory_path() /
                    "open_lmm_runtime_client_callback_move";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  test::WriteRuntimeConfigFixture(root / "config", root / "data",
                                  root / "output");

  auto& coordinator = GlobalRuntimeRetirementCoordinator();
  coordinator.WaitIdle();
  const auto completed_before = coordinator.Diagnostics().completed;

  RuntimeClient active(1);
  RuntimeClient replacement(1);
  Check(active.Open({root / "config", "callback-move"}).IsOk(),
        "callback move fixture opens");
  std::mutex mutex;
  std::condition_variable changed;
  bool assigned = false;
  auto subscription = active.SubscribeEvents(
      [&active, &replacement, &mutex, &changed,
       &assigned](const ExecutionEvent& event) {
        if (event.type != EventType::kJobCompleted &&
            event.type != EventType::kJobCancelled) {
          return;
        }
        active = std::move(replacement);
        {
          std::lock_guard lock(mutex);
          assigned = true;
        }
        changed.notify_all();
      });
  Check(subscription.IsOk(), "callback move subscription succeeds");
  const auto job = active.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job.IsOk(), "callback move job submits");
  {
    std::unique_lock lock(mutex);
    changed.wait(lock, [&] { return assigned; });
  }
  coordinator.WaitIdle();
  const auto after = coordinator.Diagnostics();
  Check(!active.IsOpen() && after.pending == 0 &&
            after.completed == completed_before + 1,
        "callback move assignment retires only the replaced implementation");

  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestForwardingMoveAndClose();
  TestRetirementCoordinatorDrainsBoundedQueue();
  TestCallbackDestructionUsesCoordinator();
  TestCallbackMoveAssignmentRetiresOldImpl();
  std::cout << "runtime client contract tests passed\n";
  return 0;
}
