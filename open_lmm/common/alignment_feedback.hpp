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

class AlignmentFeedbackBroker {
 public:
  using Notification = std::function<void(const AlignmentFeedbackSnapshot&)>;

  Result<AlignmentResponse> Request(
      AlignmentFeedbackSnapshot snapshot,
      const std::shared_ptr<CancellationToken>& cancellation,
      std::chrono::milliseconds timeout = std::chrono::milliseconds::zero()) {
    Notification notification;
    AlignmentFeedbackSnapshot published;
    {
      std::lock_guard lock(mutex_);
      if (active_) {
        return Result<AlignmentResponse>::Failure(
            Error::InvalidArgument("another alignment request is active"));
      }
      snapshot.proposal.request_id = next_request_id_++;
      active_ = std::move(snapshot);
      // Publish the request before notifying observers. Notifications are
      // synchronous and may immediately inspect or answer the request.
      active_published_ = true;
      published = *active_;
      response_.reset();
      notification = notification_;
    }
    try {
      if (notification) notification(published);
    } catch (const std::exception& error) {
      ClearRequest(published.proposal.request_id);
      return Result<AlignmentResponse>::Failure(Error::InvalidArgument(
          std::string("alignment feedback notification failed: ") +
          error.what()));
    } catch (...) {
      ClearRequest(published.proposal.request_id);
      return Result<AlignmentResponse>::Failure(Error::InvalidArgument(
          "alignment feedback notification failed: unknown exception"));
    }

    std::unique_lock lock(mutex_);
    const auto deadline = timeout.count() > 0
                              ? std::chrono::steady_clock::now() + timeout
                              : std::chrono::steady_clock::time_point::max();
    while (!response_) {
      if (cancellation && cancellation->IsCancellationRequested()) {
        active_.reset();
        active_published_ = false;
        condition_.notify_all();
        return Result<AlignmentResponse>::Failure(
            Error::Cancelled("alignment feedback cancelled"));
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        active_.reset();
        active_published_ = false;
        return Result<AlignmentResponse>::Failure(
            Error::InvalidArgument("alignment feedback timed out"));
      }
      condition_.wait_for(lock, std::chrono::milliseconds(50));
    }
    AlignmentResponse response = std::move(*response_);
    response_.reset();
    active_.reset();
    active_published_ = false;
    return Result<AlignmentResponse>::Ok(std::move(response));
  }

  Result<void> Respond(AlignmentResponse response) {
    std::lock_guard lock(mutex_);
    if (!active_ || !active_published_ ||
        active_->proposal.request_id != response.request_id) {
      return Result<void>::Failure(
          Error::InvalidArgument("stale or unknown alignment request"));
    }
    if (response.decision == AlignmentDecision::kManual) {
      if (!response.manual_target_T_source) {
        return Result<void>::Failure(
            Error::InvalidArgument("manual alignment requires a finite rigid transform"));
      }
      auto valid = ValidateRigidTransform(*response.manual_target_T_source,
                                          "manual alignment");
      if (!valid) return valid;
    } else if (response.manual_target_T_source) {
      return Result<void>::Failure(Error::InvalidArgument(
          "only a manual alignment response may include a transform"));
    }
    response_ = std::move(response);
    condition_.notify_all();
    return Result<void>::Ok();
  }

  [[nodiscard]] std::optional<AlignmentFeedbackSnapshot> Snapshot() const {
    std::lock_guard lock(mutex_);
    return active_published_ ? active_ : std::nullopt;
  }

  void Cancel() {
    std::lock_guard lock(mutex_);
    if (!active_) return;
    response_ = AlignmentResponse{active_->proposal.request_id,
                                  AlignmentDecision::kCancel, std::nullopt};
    condition_.notify_all();
  }

  void SetNotification(Notification notification) {
    std::lock_guard lock(mutex_);
    notification_ = std::move(notification);
  }

  void SetEnabled(bool enabled) {
    std::lock_guard lock(mutex_);
    enabled_ = enabled;
  }

  [[nodiscard]] bool IsEnabled() const {
    std::lock_guard lock(mutex_);
    return enabled_;
  }

 private:
  void ClearRequest(uint64_t request_id) {
    std::lock_guard lock(mutex_);
    if (!active_ || active_->proposal.request_id != request_id) return;
    response_.reset();
    active_.reset();
    active_published_ = false;
    condition_.notify_all();
  }

  mutable std::mutex mutex_;
  std::condition_variable condition_;
  uint64_t next_request_id_ = 1;
  std::optional<AlignmentFeedbackSnapshot> active_;
  bool active_published_ = false;
  std::optional<AlignmentResponse> response_;
  Notification notification_;
  bool enabled_ = false;
};

}  // namespace open_lmm
