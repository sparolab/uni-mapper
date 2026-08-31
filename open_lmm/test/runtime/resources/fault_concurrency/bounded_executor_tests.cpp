#include <runtime/resources/resource_governor.hpp>
#include <foundation/concurrency/bounded_executor.hpp>
#include "support/synchronization.hpp"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

namespace {
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

class ThreadLaunchProbe {
 public:
  explicit ThreadLaunchProbe(std::size_t fail_on) : fail_on_(fail_on) {}

  ThreadLauncher Launcher() {
    return [this](ThreadTask task) {
      const auto call = ++calls_;
      if (call == fail_on_) {
        throw std::system_error(
            std::make_error_code(std::errc::resource_unavailable_try_again),
            "injected bounded executor launch failure");
      }
      return std::thread([this, task = std::move(task)]() mutable {
        ++started_;
        task();
        ++completed_;
      });
    };
  }

  [[nodiscard]] std::size_t Started() const { return started_.load(); }
  [[nodiscard]] std::size_t Completed() const { return completed_.load(); }

 private:
  const std::size_t fail_on_;
  std::atomic<std::size_t> calls_{0};
  std::atomic<std::size_t> started_{0};
  std::atomic<std::size_t> completed_{0};
};

void TestPartialConstructionJoinsEveryWorker() {
  for (const std::size_t fail_on :
       {std::size_t{1}, std::size_t{2}, std::size_t{3}}) {
    ThreadLaunchProbe probe(fail_on);
    bool threw = false;
    try {
      BoundedExecutor executor(3, 2, {}, probe.Launcher());
    } catch (const std::system_error&) {
      threw = true;
    }
    Check(threw, "injected executor construction failure is propagated");
    Check(probe.Started() == fail_on - 1 &&
              probe.Completed() == probe.Started(),
          "partially constructed executor wakes and joins every worker");

    BoundedExecutor retry(2, 1, {}, probe.Launcher());
    const auto task = retry.Submit([] { return Result<void>::Ok(); });
    Check(task && task.Value().Wait(),
          "a fresh executor works after injected construction failure");
  }
}

void TestConcurrencyBoundAndCompletionOrder() {
  BoundedExecutor executor(2, 2);
  std::atomic<int> active{0};
  std::atomic<int> maximum_active{0};
  test::CountdownEvent workers_entered(2);
  test::ManualResetEvent release;
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
      workers_entered.Arrive();
      release.Wait("bounded worker release");
      --active;
      std::lock_guard lock(order_mutex);
      completion_order.push_back(3 - index);
      return Result<void>::Ok();
    });
    Check(submitted.IsOk(), "bounded task submitted");
    handles.push_back(std::move(submitted).Value());
  }
  workers_entered.Wait("configured executor concurrency");
  release.Signal();
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
  test::ManualResetEvent entered;
  test::ManualResetEvent release;
  auto running = executor.Submit([&] {
    entered.Signal();
    release.Wait("queued cancellation running task release");
    return Result<void>::Ok();
  });
  Check(running.IsOk(), "running fixture submitted");
  entered.Wait("running fixture entered");

  auto token = std::make_shared<CancellationToken>();
  auto queued_one = executor.Submit([] { return Result<void>::Ok(); }, token);
  auto queued_two = executor.Submit([] { return Result<void>::Ok(); }, token);
  Check(queued_one && queued_two, "queued cancellation fixtures submitted");
  token->Request();
  Check(executor.CancelQueued(token) == 2,
        "queued tasks are removed by cancellation token");
  Check(!queued_one.Value().Wait() && !queued_two.Value().Wait(),
        "cancelled queued handles complete with errors");
  release.Signal();
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
  test::ManualResetEvent submitter_waiting;
  BoundedExecutor executor(1, 1, [&] { submitter_waiting.Signal(); });
  test::ManualResetEvent entered;
  test::ManualResetEvent release;
  auto running = executor.Submit([&] {
    entered.Signal();
    release.Wait("backpressure running task release");
    return Result<void>::Ok();
  });
  Check(running.IsOk(), "backpressure running task submitted");
  entered.Wait("backpressure worker entered");
  auto queued = executor.Submit([] { return Result<void>::Ok(); });
  Check(queued.IsOk(), "backpressure queue filled");

  auto cancellation = std::make_shared<CancellationToken>();
  std::optional<Result<BoundedTaskHandle>> blocked;
  std::thread submitter([&] {
    blocked = executor.Submit([] { return Result<void>::Ok(); }, cancellation);
  });
  submitter_waiting.Wait("submitter backpressure wait");
  Check(executor.Snapshot().waiting_submitters == 1,
        "submitter reports backpressure wait");
  cancellation->Request();
  submitter.join();
  Check(blocked && !*blocked,
        "backpressured submission observes cancellation without queue space");
  release.Signal();
  Check(running.Value().Wait().IsOk() && queued.Value().Wait().IsOk(),
        "existing work completes after blocked submission cancellation");
}

void TestResourceBudget() {
  bool rejected_zero = false;
  try {
    ResourceGovernor invalid(ResourceBudget{0, 1, 1});
  } catch (const std::invalid_argument&) {
    rejected_zero = true;
  }
  Check(rejected_zero, "zero resource budget is rejected");

  ResourceGovernor governor(ResourceBudget{3, 2, 100});
  Check(governor.TryReserveMemory(60) &&
            !governor.TryReserveMemory(50) &&
            governor.ReservedMemoryBytes() == 60 &&
            governor.MemoryAdmissionFailures() == 1,
        "soft memory reservation applies backpressure");
  governor.ReleaseMemory(60);
  Check(governor.ReservedMemoryBytes() == 0 &&
            governor.AgentExecutor().Snapshot().worker_count == 2,
        "CPU thread budget caps one-runtime agent executor workers");

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
    auto rejected_resize = owned->Resize(110);
    Check(!rejected_resize && owned->Bytes() == 70 &&
              governor.ReservedMemoryBytes() == 70 &&
              rejected_resize.GetError().Message().find(
                  "memory admission rejected: class=resident_payload") !=
                  std::string::npos &&
              rejected_resize.GetError().Message().find(
                  "current_bytes=70 requested_delta_bytes=40") !=
                  std::string::npos &&
              rejected_resize.GetError().Message().find(
                  "requested_total_bytes=110 limit_bytes=100") !=
                  std::string::npos,
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
  const auto active_diagnostics = governor.Diagnostics();
  Check(active_diagnostics.budget.max_agent_tasks == 3 &&
            active_diagnostics.budget.max_cpu_threads == 2 &&
            active_diagnostics.budget.soft_memory_bytes == 100 &&
            active_diagnostics.reserved_total_bytes == 0 &&
            active_diagnostics.reserved_by_class ==
                std::array<uint64_t, 3>{0, 0, 0} &&
            active_diagnostics.peak_reserved_total_bytes == 70 &&
            active_diagnostics.peak_reserved_by_class ==
                std::array<uint64_t, 3>{70, 0, 0} &&
            active_diagnostics.admission_failures == 2 &&
            active_diagnostics.heavy_phase_active &&
            active_diagnostics.executor.worker_count == 2,
        "resource diagnostics snapshot preserves owner accounting classes");
  auto cancellation = std::make_shared<CancellationToken>();
  test::ManualResetEvent waiting;
  std::optional<Result<void>> second_phase;
  std::thread waiter([&] {
    waiting.Signal();
    second_phase = governor.AcquireHeavyMemoryPhase(cancellation);
  });
  waiting.Wait("second heavy memory phase starts waiting");
  cancellation->Request();
  waiter.join();
  Check(second_phase && !*second_phase,
        "heavy memory phase wait observes cancellation");
  governor.ReleaseHeavyMemoryPhase();
  Check(!governor.Diagnostics().heavy_phase_active,
        "resource diagnostics observes released heavy phase");
  Check(governor.AcquireHeavyMemoryPhase({}).IsOk(),
        "heavy memory phase can be reacquired after release");
  governor.ReleaseHeavyMemoryPhase();
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3 || std::string_view(argv[1]) != "--suite") {
    std::cerr << "usage: bounded_executor_tests --suite 1|2\n";
    return 2;
  }
  const std::string_view suite(argv[2]);
  if (suite == "1") {
  TestResourceBudget();
  } else if (suite == "2") {
  TestConcurrencyBoundAndCompletionOrder();
  TestQueuedCancellationAndExceptionConversion();
  TestBackpressuredSubmissionCancellation();
  TestPartialConstructionJoinsEveryWorker();
  } else {
    std::cerr << "unknown resource suite: " << suite << '\n';
    return 2;
  }
  std::cout << "bounded executor and resource budget tests passed\n";
  return 0;
}
