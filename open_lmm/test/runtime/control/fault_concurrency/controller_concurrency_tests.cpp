#include <runtime/control/pipeline_controller.hpp>
#include "support/runtime/recording_runtime_port.hpp"
#include "support/synchronization.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
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

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

AlignmentFeedbackSnapshot FeedbackSnapshot() {
  AlignmentFeedbackSnapshot snapshot;
  snapshot.proposal.target_agent = Id("A");
  snapshot.proposal.source_agent = Id("B");
  snapshot.proposal.method = AlignmentMethod::kKissMatcher;
  return snapshot;
}

class ConcurrencyRunner final : public test::RuntimePortFixture {
 public:
  ConcurrencyRunner() : RuntimePortFixture({Id("A"), Id("B")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand& command,
                              const ExecutionContext& context) override {
    cancellation = context.cancellation;
    feedback = context.alignment_feedback;
    if (command.kind == ExecutionCommandKind::kStage) {
      return RunStage(*command.stage);
    }
    return Result<void>::Ok();
  }

  Result<void> RunStage(StageId stage) {
    if (stage != StageId::kAlignment || !request_feedback) {
      return Result<void>::Ok();
    }
    std::unique_lock lock(runner_mutex);
    feedback_gate.ArriveAndWait("controller feedback release");
    AlignmentFeedbackSnapshot snapshot;
    snapshot.proposal.target_agent = Id("A");
    snapshot.proposal.source_agent = Id("B");
    snapshot.proposal.method = AlignmentMethod::kKissMatcher;
    auto response = feedback->Request(std::move(snapshot), cancellation);
    if (!response || response.Value().decision == AlignmentDecision::kCancel) {
      return Result<void>::Failure(Error::Cancelled("feedback cancelled"));
    }
    return Result<void>::Ok();
  }

  CommittedRuntimeSnapshot Snapshot() const override {
    std::lock_guard lock(runner_mutex);
    if (agent_ids_hook) agent_ids_hook();
    return RuntimePortFixture::Snapshot();
  }

  Result<void> AfterCommitFixture(
      const ExecutionCommand&, const ExecutionContext&,
      const ExecutionReceipt&) override {
    if (!block_after_commit) return Result<void>::Ok();
    committed_before_return.Signal();
    release_receipt.Wait("committed receipt release");
    return Result<void>::Ok();
  }

  mutable std::mutex runner_mutex;
  std::function<void()> agent_ids_hook;
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  bool request_feedback = false;
  test::PhaseGate feedback_gate;
  bool block_after_commit = false;
  test::ManualResetEvent committed_before_return;
  test::ManualResetEvent release_receipt;
};

void LateCancelAfterCommitKeepsCommittedSuccess() {
  auto runner = std::make_shared<ConcurrencyRunner>();
  runner->block_after_commit = true;
  PipelineController controller(runner);
  auto job = controller.SubmitStage(StageId::kDataLoad);
  Check(job.IsOk(), "late-cancel fixture submission failed");

  runner->committed_before_return.Wait("committed receipt withheld");
  Check(runner->Snapshot().revision == 2,
        "fixture did not commit before withholding its receipt");
  Check(controller.Cancel(job.Value()).IsOk(),
        "late cancellation request was rejected");
  runner->release_receipt.Signal();

  Check(controller.Wait(job.Value()).IsOk(),
        "valid committed receipt must win over late cancellation");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.job && snapshot.job->state == JobState::kSucceeded &&
            snapshot.runtime_revision == runner->Snapshot().revision &&
            snapshot.runtime_revision == 2,
        "controller did not reconcile the committed late-cancel authority");
  runner->block_after_commit = false;
  auto next = controller.SubmitStage(StageId::kDataLoad);
  Check(next && controller.Wait(next.Value()) &&
            controller.Snapshot().runtime_revision ==
                runner->Snapshot().revision,
        "a late cancellation poisoned the next command base revision");
}

void SnapshotUsesImmutableRunnerState() {
  auto runner = std::make_shared<ConcurrencyRunner>();
  PipelineController controller(runner);
  runner->agent_ids_hook = [&controller] { (void)controller.Snapshot(); };

  auto future = std::async(std::launch::async,
                           [&controller] { return controller.Snapshot(); });
  Check(future.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot called the query port under the controller lock");
  Check(future.get().agents == std::vector<AgentId>({Id("A"), Id("B")}),
        "cached agent snapshot mismatch");
}

void FeedbackAndSnapshotHaveNoLockInversion() {
  auto runner = std::make_shared<ConcurrencyRunner>();
  runner->request_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  test::ManualResetEvent feedback_published;
  auto subscription = controller.SubscribeEvents(
      [&](const ExecutionEvent& event) {
        if (event.type == EventType::kAlignmentFeedbackRequested) {
          feedback_published.Signal();
        }
      });
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "alignment submission failed");

  runner->feedback_gate.WaitUntilEntered("feedback lock window");

  auto future = std::async(std::launch::async,
                           [&controller] { return controller.Snapshot(); });
  Check(future.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot blocked on runner state");
  (void)future.get();

  runner->feedback_gate.Release();
  feedback_published.Wait("feedback request publication");
  const auto request = controller.GetAlignmentFeedbackSnapshot();
  Check(request.has_value(), "feedback request was not published");
  Check(controller.RespondToAlignment(
            job.Value(), {request->proposal.request_id,
                          AlignmentDecision::kAccept, std::nullopt,
                          request->session_revision}).IsOk(),
        "feedback response failed");
  Check(controller.Wait(job.Value()).IsOk(), "alignment job did not finish");
}

void FeedbackPublishesBeforeSynchronousNotification() {
  AlignmentFeedbackBroker broker;
  uint64_t observed_request_id = 0;
  broker.SetNotification([&](const AlignmentFeedbackSnapshot& notified) {
    const auto published = broker.Snapshot();
    Check(published && published->proposal.request_id ==
                           notified.proposal.request_id,
          "feedback was not published before synchronous notification");
    observed_request_id = notified.proposal.request_id;
    Check(broker.Respond({notified.proposal.request_id,
                          AlignmentDecision::kAccept, std::nullopt,
                          notified.session_revision}).IsOk(),
          "synchronous feedback response was rejected");
  });
  const auto response = broker.Request(FeedbackSnapshot(), nullptr);
  Check(response && response.Value().decision == AlignmentDecision::kAccept &&
            observed_request_id != 0 && !broker.Snapshot(),
        "synchronous feedback did not complete and clear the request");
}

void ThrowingFeedbackNotificationRecoversForNextRequest() {
  AlignmentFeedbackBroker broker;
  broker.SetNotification([](const AlignmentFeedbackSnapshot&) {
    throw std::runtime_error("notification fixture");
  });
  const auto failed = broker.Request(FeedbackSnapshot(), nullptr);
  Check(!failed &&
            failed.GetError().Message().find("notification fixture") !=
                std::string::npos &&
            !broker.Snapshot(),
        "throwing notification did not clear the active request");

  uint64_t retry_request_id = 0;
  broker.SetNotification([&](const AlignmentFeedbackSnapshot& notified) {
    retry_request_id = notified.proposal.request_id;
    Check(broker.Respond({notified.proposal.request_id,
                          AlignmentDecision::kAccept, std::nullopt,
                          notified.session_revision}).IsOk(),
          "response after notification failure was rejected");
  });
  const auto retried = broker.Request(FeedbackSnapshot(), nullptr);
  Check(retried && retry_request_id > 1 && !broker.Snapshot(),
        "feedback request was not reusable after callback failure");
}

void FeedbackUpdateNotifiesOutsideBrokerLock() {
  AlignmentFeedbackBroker broker;
  std::atomic<int> notifications{0};
  broker.SetNotification([&](const AlignmentFeedbackSnapshot& notified) {
    ++notifications;
    const auto published = broker.Snapshot();
    Check(published && published->session_revision ==
                           notified.session_revision,
          "feedback update callback cannot re-enter Snapshot");
    if (notified.session_revision > 1) {
      Check(broker.Respond({notified.proposal.request_id,
                            AlignmentDecision::kAccept, std::nullopt,
                            notified.session_revision}).IsOk(),
            "feedback update callback cannot synchronously respond");
    }
  });
  auto begun = broker.Begin(FeedbackSnapshot());
  Check(begun.IsOk(), "feedback session did not begin");
  auto updated = FeedbackSnapshot();
  updated.attempt_status.state = AlignmentAttemptState::kSucceeded;
  Check(broker.Update(begun.Value(), std::move(updated)).IsOk(),
        "feedback session did not update");
  const auto response = broker.WaitDecision(begun.Value(), nullptr);
  Check(response && response.Value().decision == AlignmentDecision::kAccept &&
            notifications.load() == 2,
        "feedback update notification did not complete outside the lock");
  Check(broker.End(begun.Value()).IsOk(),
        "feedback session did not end cleanly");
}

void TerminalFeedbackNotifiesOutsideBrokerLock() {
  AlignmentFeedbackBroker broker;
  std::atomic<bool> saw_terminal{false};
  broker.SetNotification([&](const AlignmentFeedbackSnapshot& notified) {
    const auto published = broker.Snapshot();
    Check(published && published->session_revision ==
                           notified.session_revision,
          "terminal callback cannot re-enter Snapshot");
    if (notified.review_state == AlignmentReviewState::kFailed) {
      saw_terminal = true;
      Check(!broker.Respond({notified.proposal.request_id,
                             AlignmentDecision::kAccept, std::nullopt,
                             notified.session_revision}),
            "terminal callback cannot mutate its read-only review");
    }
  });
  const auto begun = broker.Begin(FeedbackSnapshot());
  Check(begun.IsOk(), "terminal notification session did not begin");
  Check(broker.End(begun.Value(), Error::PluginLoadFailed("fixture failure"))
            .IsOk(),
        "terminal notification did not complete outside the lock");
  Check(saw_terminal.load(), "terminal notification was not observed");
}

}  // namespace

int main() {
  LateCancelAfterCommitKeepsCommittedSuccess();
  SnapshotUsesImmutableRunnerState();
  FeedbackAndSnapshotHaveNoLockInversion();
  FeedbackPublishesBeforeSynchronousNotification();
  ThrowingFeedbackNotificationRecoversForNextRequest();
  FeedbackUpdateNotifiesOutsideBrokerLock();
  TerminalFeedbackNotifiesOutsideBrokerLock();
  std::cout << "controller concurrency tests passed\n";
  return 0;
}
