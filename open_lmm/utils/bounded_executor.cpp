#include "bounded_executor.hpp"

#include <chrono>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

namespace open_lmm {

Result<void> BoundedTaskHandle::Wait() const {
  if (!completion_.valid()) {
    return Result<void>::Failure(
        Error::InvalidArgument("bounded task handle is invalid"));
  }
  return completion_.get();
}

BoundedExecutor::BoundedExecutor(std::size_t worker_count,
                                 std::size_t queue_capacity)
    : queue_capacity_(queue_capacity) {
  if (worker_count == 0 || queue_capacity == 0) {
    throw std::invalid_argument(
        "bounded executor worker and queue counts must be positive");
  }
  workers_.reserve(worker_count);
  for (std::size_t index = 0; index < worker_count; ++index) {
    workers_.emplace_back([this] { workerLoop(); });
  }
}

BoundedExecutor::~BoundedExecutor() {
  std::deque<WorkItem> cancelled;
  {
    std::lock_guard lock(mutex_);
    accepting_ = false;
    stopping_ = true;
    cancelled.swap(queue_);
    cancelled_queued_tasks_ += cancelled.size();
  }
  for (auto& item : cancelled) {
    item.completion->set_value(Result<void>::Failure(
        Error::Cancelled("bounded executor shutdown before task start")));
  }
  work_available_.notify_all();
  queue_space_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) worker.join();
  }
}

Result<BoundedTaskHandle> BoundedExecutor::Submit(
    Task task, std::shared_ptr<CancellationToken> cancellation) {
  if (!task) {
    return Result<BoundedTaskHandle>::Failure(
        Error::InvalidArgument("bounded executor task must be non-empty"));
  }
  auto completion = std::make_shared<std::promise<Result<void>>>();
  auto future = completion->get_future().share();
  uint64_t id = 0;
  {
    std::unique_lock lock(mutex_);
    ++waiting_submitters_;
    while (accepting_ && queue_.size() >= queue_capacity_ &&
           !(cancellation && cancellation->IsCancellationRequested())) {
      queue_space_.wait_for(lock, std::chrono::milliseconds(10));
    }
    --waiting_submitters_;
    if (!accepting_) {
      return Result<BoundedTaskHandle>::Failure(
          Error::InvalidArgument("bounded executor is shutting down"));
    }
    if (cancellation && cancellation->IsCancellationRequested()) {
      return Result<BoundedTaskHandle>::Failure(
          Error::Cancelled("before bounded task submission"));
    }
    id = next_id_++;
    queue_.push_back(
        {id, std::move(task), std::move(cancellation), completion});
  }
  work_available_.notify_one();
  return Result<BoundedTaskHandle>::Ok(
      BoundedTaskHandle(id, std::move(future)));
}

std::size_t BoundedExecutor::CancelQueued(
    const std::shared_ptr<CancellationToken>& cancellation) {
  if (!cancellation) return 0;
  std::vector<WorkItem> cancelled;
  {
    std::lock_guard lock(mutex_);
    for (auto item = queue_.begin(); item != queue_.end();) {
      if (item->cancellation == cancellation) {
        cancelled.push_back(std::move(*item));
        item = queue_.erase(item);
      } else {
        ++item;
      }
    }
    cancelled_queued_tasks_ += cancelled.size();
  }
  for (auto& item : cancelled) {
    item.completion->set_value(Result<void>::Failure(
        Error::Cancelled("bounded task cancelled while queued")));
  }
  if (!cancelled.empty()) queue_space_.notify_all();
  return cancelled.size();
}

void BoundedExecutor::WaitIdle() {
  std::unique_lock lock(mutex_);
  idle_.wait(lock, [&] { return queue_.empty() && active_tasks_ == 0; });
}

BoundedExecutorSnapshot BoundedExecutor::Snapshot() const {
  std::lock_guard lock(mutex_);
  return {workers_.size(), queue_capacity_, queue_.size(), active_tasks_,
          waiting_submitters_, completed_tasks_, cancelled_queued_tasks_};
}

Result<void> BoundedExecutor::Run(const WorkItem& item) noexcept {
  try {
    if (item.cancellation &&
        item.cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(
          Error::Cancelled("bounded task cancelled before execution"));
    }
    return item.task();
  } catch (const CancellationException& error) {
    return Result<void>::Failure(Error::Cancelled(error.what()));
  } catch (const std::exception& error) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string("bounded task exception: ") + error.what()));
  } catch (...) {
    return Result<void>::Failure(
        Error::InvalidArgument("bounded task threw an unknown exception"));
  }
}

void BoundedExecutor::workerLoop() {
  while (true) {
    WorkItem item;
    {
      std::unique_lock lock(mutex_);
      work_available_.wait(lock,
                           [&] { return stopping_ || !queue_.empty(); });
      if (stopping_ && queue_.empty()) return;
      item = std::move(queue_.front());
      queue_.pop_front();
      ++active_tasks_;
    }
    queue_space_.notify_one();
    item.completion->set_value(Run(item));
    {
      std::lock_guard lock(mutex_);
      --active_tasks_;
      ++completed_tasks_;
      if (queue_.empty() && active_tasks_ == 0) idle_.notify_all();
    }
  }
}

}  // namespace open_lmm
