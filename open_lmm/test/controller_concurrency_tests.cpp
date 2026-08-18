#include <open_lmm/server/pipeline_controller.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace {
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

class ConcurrencyRunner final : public StageRunner {
 public:
  void SetCancellationToken(
      std::shared_ptr<CancellationToken> value) override {
    cancellation = std::move(value);
  }

  void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker> value) override {
    feedback = std::move(value);
  }

  Result<void> RunStage(StageId stage) override {
    if (stage != StageId::kAlignment || !request_feedback) {
      return Result<void>::Ok();
    }
    std::unique_lock lock(runner_mutex);
    feedback_window.store(true, std::memory_order_release);
    while (!release_feedback.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    AlignmentFeedbackSnapshot snapshot;
    snapshot.proposal.target_agent = 'A';
    snapshot.proposal.source_agent = 'B';
    snapshot.proposal.method = AlignmentMethod::kKissMatcher;
    auto response = feedback->Request(std::move(snapshot), cancellation);
    if (!response || response.Value().decision == AlignmentDecision::kCancel) {
      return Result<void>::Failure(Error::Cancelled("feedback cancelled"));
    }
    return Result<void>::Ok();
  }

  Result<void> RunNode(NodeId, std::optional<char>) override {
    return Result<void>::Ok();
  }

  Result<void> RunOptimizeThrough(char) override {
    return Result<void>::Ok();
  }

  std::vector<char> AgentIds() const override {
    std::lock_guard lock(runner_mutex);
    if (agent_ids_hook) agent_ids_hook();
    return {'A', 'B'};
  }

  mutable std::mutex runner_mutex;
  std::function<void()> agent_ids_hook;
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  bool request_feedback = false;
  std::atomic<bool> feedback_window{false};
  std::atomic<bool> release_feedback{false};
};

void SnapshotUsesImmutableRunnerState() {
  auto runner = std::make_shared<ConcurrencyRunner>();
  PipelineController controller(runner);
  runner->agent_ids_hook = [&controller] { (void)controller.Snapshot(); };

  auto future = std::async(std::launch::async,
                           [&controller] { return controller.Snapshot(); });
  Check(future.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot called StageRunner under the controller lock");
  Check(future.get().agents == std::vector<char>({'A', 'B'}),
        "cached agent snapshot mismatch");
}

void FeedbackAndSnapshotHaveNoLockInversion() {
  auto runner = std::make_shared<ConcurrencyRunner>();
  runner->request_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "alignment submission failed");

  const auto lock_deadline = std::chrono::steady_clock::now() +
                             std::chrono::milliseconds(500);
  while (!runner->feedback_window.load(std::memory_order_acquire) &&
         std::chrono::steady_clock::now() < lock_deadline) {
    std::this_thread::yield();
  }
  Check(runner->feedback_window.load(std::memory_order_acquire),
        "feedback lock window was not reached");

  auto future = std::async(std::launch::async,
                           [&controller] { return controller.Snapshot(); });
  Check(future.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot blocked on runner state");
  (void)future.get();

  runner->release_feedback.store(true, std::memory_order_release);
  std::optional<AlignmentFeedbackSnapshot> request;
  const auto request_deadline = std::chrono::steady_clock::now() +
                                std::chrono::milliseconds(500);
  while (!(request = controller.GetAlignmentFeedbackSnapshot()) &&
         std::chrono::steady_clock::now() < request_deadline) {
    std::this_thread::yield();
  }
  Check(request.has_value(), "feedback request was not published");
  Check(controller.RespondToAlignment(
            job.Value(), {request->proposal.request_id,
                          AlignmentDecision::kAccept, std::nullopt}).IsOk(),
        "feedback response failed");
  Check(controller.Wait(job.Value()).IsOk(), "alignment job did not finish");
}

}  // namespace

int main() {
  SnapshotUsesImmutableRunnerState();
  FeedbackAndSnapshotHaveNoLockInversion();
  std::cout << "controller concurrency tests passed\n";
  return 0;
}
