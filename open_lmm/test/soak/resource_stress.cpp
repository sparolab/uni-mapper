#include "support/soak/owner_stress_support.hpp"
#include "support/synchronization.hpp"

#include <runtime/resources/resource_governor.hpp>

#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace soak = open_lmm::test::soak;
namespace test = open_lmm::test;
using Json = nlohmann::json;

namespace {
using namespace open_lmm;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

Json ExecutorJson(const BoundedExecutorSnapshot& snapshot) {
  return {{"worker_count", snapshot.worker_count},
          {"queue_capacity", snapshot.queue_capacity},
          {"queued_tasks", snapshot.queued_tasks},
          {"active_tasks", snapshot.active_tasks},
          {"waiting_submitters", snapshot.waiting_submitters},
          {"completed_tasks", snapshot.completed_tasks},
          {"cancelled_queued_tasks", snapshot.cancelled_queued_tasks}};
}

void Append(Json& report, uint64_t iteration,
            const soak::ProcessMetrics& process,
            const ResourceGovernor& governor) {
  report["samples"].push_back(
      {{"iteration", iteration},
       {"checkpoint", "resource_owner_idle"},
       {"process", soak::ProcessMetricsJson(process)},
       {"runtime_revision", nullptr},
       {"recent_event_count", nullptr},
       {"reserved_memory_bytes", governor.ReservedMemoryBytes()},
       {"executor", ExecutorJson(governor.AgentExecutor().Snapshot())},
       {"owner", soak::EmptyOwnerMetrics()}});
}

soak::ProcessSeries Run(const soak::RunOptions& options, Json& report) {
  ResourceGovernor governor(ResourceBudget{3, 1, 100});
  soak::ProcessSeries series;
  uint64_t expected_completed = 0;
  uint64_t expected_cancelled = 0;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    {
      auto reservation =
          governor.ReserveMemory(60, MemoryClass::kTransientTask);
      Require(reservation && governor.ReservedMemoryBytes() == 60 &&
                  !governor.ReserveMemory(50, MemoryClass::kHeavyMap),
              "memory admission did not preserve its limit");
      auto owned = std::move(reservation).Value();
      Require(owned.Resize(80).IsOk() &&
                  governor.ReservedMemoryBytes() == 80,
              "reservation resize did not update owner accounting");
    }
    Require(governor.ReservedMemoryBytes() == 0,
            "reservation owner did not release memory");

    auto& executor = governor.AgentExecutor();
    test::ManualResetEvent entered;
    test::ManualResetEvent release;
    auto running = executor.Submit([&] {
      entered.Signal();
      release.Wait("release resource stress worker");
      return Result<void>::Ok();
    });
    Require(running.IsOk(), "running resource task was rejected");
    entered.Wait("resource stress worker entered");
    auto cancellation = std::make_shared<CancellationToken>();
    auto queued = executor.Submit([] { return Result<void>::Ok(); },
                                  cancellation);
    Require(queued.IsOk(), "resource stress queue did not reach capacity");
    cancellation->Request();
    Require(executor.CancelQueued(cancellation) == 1 &&
                !queued.Value().Wait(),
            "queued cancellation did not drain the pending task");
    ++expected_cancelled;
    release.Signal();
    Require(running.Value().Wait().IsOk(),
            "active resource task failed after release");
    ++expected_completed;
    executor.WaitIdle();

    Require(governor.AcquireHeavyMemoryPhase({}).IsOk(),
            "heavy phase was not admitted at idle");
    governor.ReleaseHeavyMemoryPhase();
    const auto snapshot = executor.Snapshot();
    Require(snapshot.queued_tasks == 0 && snapshot.active_tasks == 0 &&
                snapshot.waiting_submitters == 0 &&
                snapshot.completed_tasks == expected_completed &&
                snapshot.cancelled_queued_tasks == expected_cancelled &&
                governor.ReservedMemoryBytes() == 0,
            "resource owner telemetry did not return to exact idle state");
    const auto process = soak::SampleProcessMetrics();
    Append(report, iteration, process, governor);
    soak::AddProcessPoint(series, iteration, process);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = soak::ParseRunOptions(argc, argv);
    Json report = soak::InitialOwnerReport(
        options, "resource-admission", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "resource_governor"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "resource stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << options.iterations << '\n';
    return passed ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid soak request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak infrastructure failure: " << error.what() << '\n';
    return 2;
  }
}
