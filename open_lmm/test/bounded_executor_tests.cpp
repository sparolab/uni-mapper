#include <open_lmm/server/resource_governor.hpp>
#include <open_lmm/utils/bounded_executor.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

namespace {
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

template <typename Predicate>
void WaitUntil(Predicate predicate, const char* message) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  Check(predicate(), message);
}

void TestConcurrencyBoundAndCompletionOrder() {
  BoundedExecutor executor(2, 2);
  std::atomic<int> active{0};
  std::atomic<int> maximum_active{0};
  std::atomic<bool> release{false};
  std::mutex order_mutex;
  std::vector<int> completion_order;
  std::vector<BoundedTaskHandle> handles;
  for (int index = 0; index < 4; ++index) {
    auto submitted = executor.Submit([&, index] {
      const int current = ++active;
      int observed = maximum_active.load();
      while (observed < current &&
             !maximum_active.compare_exchange_weak(observed, current)) {
      }
      while (!release.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      --active;
      std::lock_guard lock(order_mutex);
      completion_order.push_back(3 - index);
      return Result<void>::Ok();
    });
    Check(submitted.IsOk(), "bounded task submitted");
    handles.push_back(std::move(submitted).Value());
  }
  WaitUntil([&] { return maximum_active.load() == 2; },
            "bounded executor reaches configured concurrency");
  release = true;
  for (const auto& handle : handles) {
    Check(handle.Wait().IsOk(), "bounded task completes");
  }
  Check(maximum_active.load() == 2 && completion_order.size() == 4,
        "bounded executor never exceeds worker count");
  executor.WaitIdle();
  const auto snapshot = executor.Snapshot();
  Check(snapshot.queued_tasks == 0 && snapshot.active_tasks == 0 &&
            snapshot.completed_tasks == 4,
        "executor idle telemetry is consistent");
}

void TestQueuedCancellationAndExceptionConversion() {
  BoundedExecutor executor(1, 2);
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
  auto running = executor.Submit([&] {
    entered = true;
    while (!release.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    return Result<void>::Ok();
  });
  Check(running.IsOk(), "running fixture submitted");
  WaitUntil([&] { return entered.load(); }, "running fixture entered");

  auto token = std::make_shared<CancellationToken>();
  auto queued_one = executor.Submit([] { return Result<void>::Ok(); }, token);
  auto queued_two = executor.Submit([] { return Result<void>::Ok(); }, token);
  Check(queued_one && queued_two, "queued cancellation fixtures submitted");
  token->Request();
  Check(executor.CancelQueued(token) == 2,
        "queued tasks are removed by cancellation token");
  Check(!queued_one.Value().Wait() && !queued_two.Value().Wait(),
        "cancelled queued handles complete with errors");
  release = true;
  Check(running.Value().Wait().IsOk(), "active task joins after release");

  auto throwing = executor.Submit([]() -> Result<void> {
    throw std::runtime_error("fixture");
  });
  Check(throwing && !throwing.Value().Wait() &&
            throwing.Value().Wait().GetError().Message().find("fixture") !=
                std::string::npos,
        "task exception is converted to Result failure");
}

void TestBackpressuredSubmissionCancellation() {
  BoundedExecutor executor(1, 1);
  std::atomic<bool> entered{false};
  std::atomic<bool> release{false};
  auto running = executor.Submit([&] {
    entered = true;
    while (!release.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    return Result<void>::Ok();
  });
  Check(running.IsOk(), "backpressure running task submitted");
  WaitUntil([&] { return entered.load(); }, "backpressure worker entered");
  auto queued = executor.Submit([] { return Result<void>::Ok(); });
  Check(queued.IsOk(), "backpressure queue filled");

  auto cancellation = std::make_shared<CancellationToken>();
  std::optional<Result<BoundedTaskHandle>> blocked;
  std::thread submitter([&] {
    blocked = executor.Submit([] { return Result<void>::Ok(); }, cancellation);
  });
  WaitUntil([&] { return executor.Snapshot().waiting_submitters == 1; },
            "submitter reports backpressure wait");
  cancellation->Request();
  submitter.join();
  Check(blocked && !*blocked,
        "backpressured submission observes cancellation without queue space");
  release = true;
  Check(running.Value().Wait().IsOk() && queued.Value().Wait().IsOk(),
        "existing work completes after blocked submission cancellation");
}

void TestResourceBudget() {
  bool rejected_zero = false;
  try {
    ResourceGovernor invalid(ResourceBudget{1, 0, 1, 1});
  } catch (const std::invalid_argument&) {
    rejected_zero = true;
  }
  Check(rejected_zero, "zero resource budget is rejected");

  ResourceGovernor governor(ResourceBudget{2, 3, 2, 100});
  Check(governor.TryAcquireSession() && governor.TryAcquireSession() &&
            !governor.TryAcquireSession(),
        "session admission respects budget");
  governor.ReleaseSession();
  governor.ReleaseSession();
  Check(governor.TryReserveMemory(60) &&
            !governor.TryReserveMemory(50) &&
            governor.ReservedMemoryBytes() == 60 &&
            governor.MemoryAdmissionFailures() == 1,
        "soft memory reservation applies backpressure");
  governor.ReleaseMemory(60);
  Check(governor.ReservedMemoryBytes() == 0 &&
            governor.AgentExecutor().Snapshot().worker_count == 2,
        "CPU thread budget caps agent executor workers");

  {
    auto reservation = governor.ReserveMemory(
        40, MemoryClass::kResidentPayload);
    Check(reservation && governor.ReservedMemoryBytes() == 40 &&
              governor.ReservedMemoryBytes(MemoryClass::kResidentPayload) == 40,
          "resident reservation remains charged while its owner lives");
    auto owned = std::make_shared<MemoryReservation>(
        std::move(reservation).Value());
    Check(owned->Resize(70).IsOk() &&
              governor.ReservedMemoryBytes() == 70 &&
              governor.ReservedMemoryBytes(MemoryClass::kResidentPayload) == 70,
          "resident reservation can be corrected to measured size");
    Check(!owned->Resize(110) && owned->Bytes() == 70 &&
              governor.ReservedMemoryBytes() == 70,
          "failed resize preserves the previous reservation");
    auto shared_owner = owned;
    owned.reset();
    Check(governor.ReservedMemoryBytes() == 70,
          "shared immutable payload does not release reservation early");
    shared_owner.reset();
  }
  Check(governor.ReservedMemoryBytes() == 0,
        "last resident payload owner releases its reservation");
  Check(governor.ReservedMemoryBytes(MemoryClass::kResidentPayload) == 0 &&
            governor.MemoryAdmissionFailures() == 2,
        "memory class accounting and admission failure metrics are balanced");

  auto first_phase = governor.AcquireHeavyMemoryPhase({});
  Check(first_phase.IsOk(), "first heavy memory phase is admitted");
  auto cancellation = std::make_shared<CancellationToken>();
  std::atomic<bool> waiting{false};
  std::optional<Result<void>> second_phase;
  std::thread waiter([&] {
    waiting = true;
    second_phase = governor.AcquireHeavyMemoryPhase(cancellation);
  });
  WaitUntil([&] { return waiting.load(); },
            "second heavy memory phase starts waiting");
  cancellation->Request();
  waiter.join();
  Check(second_phase && !*second_phase,
        "heavy memory phase wait observes cancellation");
  governor.ReleaseHeavyMemoryPhase();
  Check(governor.AcquireHeavyMemoryPhase({}).IsOk(),
        "heavy memory phase can be reacquired after release");
  governor.ReleaseHeavyMemoryPhase();
}

}  // namespace

int main() {
  TestConcurrencyBoundAndCompletionOrder();
  TestQueuedCancellationAndExceptionConversion();
  TestBackpressuredSubmissionCancellation();
  TestResourceBudget();
  std::cout << "bounded executor and resource budget tests passed\n";
  return 0;
}
