#pragma once

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace open_lmm {

struct BoundedExecutorSnapshot {
  std::size_t worker_count = 0;
  std::size_t queue_capacity = 0;
  std::size_t queued_tasks = 0;
  std::size_t active_tasks = 0;
  std::size_t waiting_submitters = 0;
  std::size_t max_queued_tasks = 0;
  std::size_t max_active_tasks = 0;
  std::size_t max_waiting_submitters = 0;
  uint64_t completed_tasks = 0;
  uint64_t cancelled_queued_tasks = 0;
};

class BoundedTaskHandle {
 public:
  BoundedTaskHandle() = default;
  [[nodiscard]] uint64_t Id() const noexcept { return id_; }
  [[nodiscard]] bool Valid() const noexcept { return completion_.valid(); }
  Result<void> Wait() const;

 private:
  BoundedTaskHandle(uint64_t id,
                    std::shared_future<Result<void>> completion)
      : id_(id), completion_(std::move(completion)) {}

  uint64_t id_ = 0;
  std::shared_future<Result<void>> completion_;
  friend class BoundedExecutor;
};

class BoundedExecutor {
 public:
  using Task = std::function<Result<void>()>;
  using SubmissionWaitNotification = std::function<void()>;

  explicit BoundedExecutor(std::size_t worker_count,
                           std::size_t queue_capacity,
                           SubmissionWaitNotification wait_notification = {});
  ~BoundedExecutor();
  BoundedExecutor(const BoundedExecutor&) = delete;
  BoundedExecutor& operator=(const BoundedExecutor&) = delete;

  Result<BoundedTaskHandle> Submit(
      Task task,
      std::shared_ptr<CancellationToken> cancellation = {});
  std::size_t CancelQueued(
      const std::shared_ptr<CancellationToken>& cancellation);
  void WaitIdle();
  [[nodiscard]] BoundedExecutorSnapshot Snapshot() const;
  void ResetDiagnosticPeaksToCurrent();

 private:
  struct WorkItem {
    uint64_t id = 0;
    Task task;
    std::shared_ptr<CancellationToken> cancellation;
    std::shared_ptr<std::promise<Result<void>>> completion;
  };

  void workerLoop();
  static Result<void> Run(const WorkItem& item) noexcept;

  const std::size_t queue_capacity_;
  SubmissionWaitNotification wait_notification_;
  mutable std::mutex mutex_;
  std::condition_variable work_available_;
  std::condition_variable queue_space_;
  std::condition_variable idle_;
  std::deque<WorkItem> queue_;
  std::vector<std::thread> workers_;
  uint64_t next_id_ = 1;
  uint64_t completed_tasks_ = 0;
  uint64_t cancelled_queued_tasks_ = 0;
  std::size_t active_tasks_ = 0;
  std::size_t waiting_submitters_ = 0;
  std::size_t max_queued_tasks_ = 0;
  std::size_t max_active_tasks_ = 0;
  std::size_t max_waiting_submitters_ = 0;
  bool accepting_ = true;
  bool stopping_ = false;
};

}  // namespace open_lmm
