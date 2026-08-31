#include "runtime_retirement_coordinator.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace open_lmm {

RuntimeRetirementCoordinator::RuntimeRetirementCoordinator(
    ThreadLauncher launcher) {
  if (!launcher) {
    throw std::invalid_argument("runtime retirement thread launcher is empty");
  }
  worker_ = launcher([this] { Run(); });
}

RuntimeRetirementCoordinator::~RuntimeRetirementCoordinator() {
  {
    std::lock_guard lock(mutex_);
    stopping_ = true;
  }
  changed_.notify_one();
  if (worker_.joinable()) worker_.join();
}

void RuntimeRetirementCoordinator::Retire(
    RuntimeRetirementNode& node) noexcept {
  if (!node.owner || !node.destroy) std::terminate();
  {
    std::lock_guard lock(mutex_);
    if (stopping_ || node.queued) std::terminate();
    node.queued = true;
    node.next = nullptr;
    if (tail_) {
      tail_->next = &node;
    } else {
      head_ = &node;
    }
    tail_ = &node;
    ++pending_;
    peak_pending_ = std::max(peak_pending_, pending_);
  }
  changed_.notify_one();
}

void RuntimeRetirementCoordinator::WaitIdle() {
  std::unique_lock lock(mutex_);
  idle_.wait(lock, [this] { return pending_ == 0; });
}

RuntimeRetirementDiagnostics RuntimeRetirementCoordinator::Diagnostics()
    const {
  std::lock_guard lock(mutex_);
  return {pending_, peak_pending_, completed_};
}

void RuntimeRetirementCoordinator::Run() noexcept {
  while (true) {
    RuntimeRetirementNode* node = nullptr;
    {
      std::unique_lock lock(mutex_);
      changed_.wait(lock, [this] { return stopping_ || head_; });
      if (!head_) {
        if (stopping_) return;
        continue;
      }
      node = head_;
      head_ = node->next;
      if (!head_) tail_ = nullptr;
      node->next = nullptr;
      node->queued = false;
    }

    node->destroy(node->owner);

    {
      std::lock_guard lock(mutex_);
      --pending_;
      ++completed_;
      if (pending_ == 0) idle_.notify_all();
    }
  }
}

RuntimeRetirementCoordinator& GlobalRuntimeRetirementCoordinator() {
  static RuntimeRetirementCoordinator coordinator;
  return coordinator;
}

}  // namespace open_lmm
