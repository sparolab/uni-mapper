#pragma once

#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>

#include <chrono>
#include <condition_variable>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

namespace open_lmm {

using AlignmentReviewSessionId = uint64_t;

class AlignmentFeedbackBroker {
 public:
  using Notification = std::function<void(const AlignmentFeedbackSnapshot&)>;

  Result<AlignmentReviewSessionId> Begin(AlignmentFeedbackSnapshot snapshot) {
    Notification notification;
    AlignmentFeedbackSnapshot published;
    AlignmentReviewSessionId session_id = 0;
    {
      std::lock_guard lock(mutex_);
      if (active_) {
        return Result<AlignmentReviewSessionId>::Failure(
            Error::InvalidArgument("another alignment review is active"));
      }
      session_id = next_session_id_++;
      snapshot.proposal.request_id = session_id;
      snapshot.session_revision = 1;
      snapshot.review_state = AlignmentReviewState::kActive;
      snapshot.terminal_message.clear();
      terminal_.reset();
      active_ = std::move(snapshot);
      response_.reset();
      accepting_response_ = true;
      published = *active_;
      notification = notification_;
    }
    auto notified = Notify(notification, published);
    if (!notified) {
      ClearSession(session_id);
      return Result<AlignmentReviewSessionId>::Failure(notified.GetError());
    }
    return Result<AlignmentReviewSessionId>::Ok(session_id);
  }

  Result<void> Update(AlignmentReviewSessionId session_id,
                      AlignmentFeedbackSnapshot snapshot) {
    Notification notification;
    AlignmentFeedbackSnapshot published;
    {
      std::lock_guard lock(mutex_);
      if (!active_ || active_->proposal.request_id != session_id) {
        return Result<void>::Failure(
            Error::InvalidArgument("stale or unknown alignment review"));
      }
      snapshot.proposal.request_id = session_id;
      snapshot.session_revision = active_->session_revision + 1;
      snapshot.review_state = AlignmentReviewState::kActive;
      snapshot.terminal_message.clear();
      active_ = std::move(snapshot);
      accepting_response_ = !response_ &&
          active_->attempt_status.state != AlignmentAttemptState::kRunning;
      published = *active_;
      notification = notification_;
    }
    auto notified = Notify(notification, published);
    if (!notified) {
      ClearSession(session_id);
      return notified;
    }
    return Result<void>::Ok();
  }

  Result<AlignmentResponse> WaitDecision(
      AlignmentReviewSessionId session_id,
      const std::shared_ptr<CancellationToken>& cancellation,
      std::chrono::milliseconds timeout = std::chrono::milliseconds::zero()) {
    std::unique_lock lock(mutex_);
    const auto deadline = timeout.count() > 0
                              ? std::chrono::steady_clock::now() + timeout
                              : std::chrono::steady_clock::time_point::max();
    while (!response_) {
      if (!active_ || active_->proposal.request_id != session_id) {
        return Result<AlignmentResponse>::Failure(
            Error::Cancelled("alignment review ended while waiting"));
      }
      if (cancellation && cancellation->IsCancellationRequested()) {
        accepting_response_ = false;
        return Result<AlignmentResponse>::Failure(
            Error::Cancelled("alignment feedback cancelled"));
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        accepting_response_ = false;
        return Result<AlignmentResponse>::Failure(
            Error::InvalidArgument("alignment feedback timed out"));
      }
      condition_.wait_for(lock, std::chrono::milliseconds(50));
    }
    AlignmentResponse response = std::move(*response_);
    response_.reset();
    accepting_response_ = false;
    if (response.request_id != session_id) {
      return Result<AlignmentResponse>::Failure(
          Error::InvalidArgument("alignment response session mismatch"));
    }
    return Result<AlignmentResponse>::Ok(std::move(response));
  }

  Result<void> End(AlignmentReviewSessionId session_id,
                   std::optional<Error> terminal_error = std::nullopt) {
    Notification notification;
    std::optional<AlignmentFeedbackSnapshot> published;
    {
      std::lock_guard lock(mutex_);
      if (!active_) {
        if (terminal_ && terminal_->proposal.request_id == session_id) {
          return Result<void>::Ok();
        }
        return Result<void>::Ok();
      }
      if (active_->proposal.request_id != session_id) {
        return Result<void>::Failure(
            Error::InvalidArgument("stale or unknown alignment review"));
      }
      if (terminal_error) {
        active_->review_state =
            terminal_error->code == Error::Code::kCancelled
                ? AlignmentReviewState::kCancelled
                : AlignmentReviewState::kFailed;
        active_->terminal_message = terminal_error->Message();
        ++active_->session_revision;
        terminal_ = *active_;
        published = terminal_;
        notification = notification_;
      }
      response_.reset();
      accepting_response_ = false;
      active_.reset();
      condition_.notify_all();
    }
    if (published) {
      auto notified = Notify(notification, *published);
      if (!notified) return notified;
    }
    return Result<void>::Ok();
  }

  // Compatibility adapter for callers that need a single request/response.
  Result<AlignmentResponse> Request(
      AlignmentFeedbackSnapshot snapshot,
      const std::shared_ptr<CancellationToken>& cancellation,
      std::chrono::milliseconds timeout = std::chrono::milliseconds::zero()) {
    auto begun = Begin(std::move(snapshot));
    if (!begun) {
      return Result<AlignmentResponse>::Failure(begun.GetError());
    }
    const auto session_id = begun.Value();
    auto response = WaitDecision(session_id, cancellation, timeout);
    if (response && response.Value().decision == AlignmentDecision::kCancel) {
      (void)End(session_id,
                Error::Cancelled("alignment feedback cancelled"));
    } else if (response) {
      (void)End(session_id);
    } else {
      (void)End(session_id, response.GetError());
    }
    return response;
  }

  Result<void> Respond(AlignmentResponse response) {
    std::lock_guard lock(mutex_);
    if (!active_ || active_->proposal.request_id != response.request_id) {
      return Result<void>::Failure(
          Error::InvalidArgument("stale or unknown alignment request"));
    }
    if (response.session_revision != active_->session_revision) {
      return Result<void>::Failure(Error::InvalidArgument(
          "stale alignment response revision"));
    }
    if ((!accepting_response_ &&
         response.decision != AlignmentDecision::kCancel) || response_) {
      return Result<void>::Failure(
          Error::InvalidArgument(
              "alignment review is not accepting a response"));
    }
    if (response.decision == AlignmentDecision::kManual) {
      if (!response.manual_target_T_source) {
        return Result<void>::Failure(Error::InvalidArgument(
            "manual alignment requires a finite rigid transform"));
      }
      auto valid = ValidateRigidTransform(*response.manual_target_T_source,
                                          "manual alignment");
      if (!valid) return valid;
    } else if (response.manual_target_T_source) {
      return Result<void>::Failure(Error::InvalidArgument(
          "only a manual alignment response may include a transform"));
    }
    response_ = std::move(response);
    accepting_response_ = false;
    condition_.notify_all();
    return Result<void>::Ok();
  }

  [[nodiscard]] std::optional<AlignmentFeedbackSnapshot> Snapshot() const {
    std::lock_guard lock(mutex_);
    return active_ ? active_ : terminal_;
  }

  void Cancel() {
    std::lock_guard lock(mutex_);
    if (!active_ || response_) return;
    response_ = AlignmentResponse{active_->proposal.request_id,
                                  AlignmentDecision::kCancel, std::nullopt,
                                  active_->session_revision};
    accepting_response_ = false;
    condition_.notify_all();
  }

  void SetNotification(Notification notification) {
    std::lock_guard lock(mutex_);
    notification_ = std::move(notification);
  }

  void SetEnabled(bool enabled) {
    std::lock_guard lock(mutex_);
    enabled_ = enabled;
    if (!enabled && active_ && !response_) {
      response_ = AlignmentResponse{active_->proposal.request_id,
                                    AlignmentDecision::kCancel, std::nullopt,
                                    active_->session_revision};
      accepting_response_ = false;
      condition_.notify_all();
    }
  }

  [[nodiscard]] bool IsEnabled() const {
    std::lock_guard lock(mutex_);
    return enabled_;
  }

 private:
  static Result<void> Notify(const Notification& notification,
                             const AlignmentFeedbackSnapshot& published) {
    try {
      if (notification) notification(published);
      return Result<void>::Ok();
    } catch (const std::exception& error) {
      return Result<void>::Failure(
          Error::InvalidArgument(
              std::string("alignment feedback notification failed: ") +
              error.what())
              .MarkFatalRuntime());
    } catch (...) {
      return Result<void>::Failure(
          Error::InvalidArgument(
              "alignment feedback notification failed: unknown exception")
              .MarkFatalRuntime());
    }
  }

  void ClearSession(AlignmentReviewSessionId session_id) {
    std::lock_guard lock(mutex_);
    if (!active_ || active_->proposal.request_id != session_id) return;
    response_.reset();
    accepting_response_ = false;
    active_.reset();
    condition_.notify_all();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  AlignmentReviewSessionId next_session_id_ = 1;
  std::optional<AlignmentFeedbackSnapshot> active_;
  std::optional<AlignmentFeedbackSnapshot> terminal_;
  std::optional<AlignmentResponse> response_;
  bool accepting_response_ = false;
  Notification notification_;
  bool enabled_ = false;
};

}  // namespace open_lmm
