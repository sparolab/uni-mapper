#pragma once

#include <foundation/concurrency/thread_launcher.hpp>

#include <condition_variable>
#include <cstddef>
#include <mutex>
#include <thread>

namespace open_lmm {

// Intrusive node embedded in the retired owner.  The coordinator removes the
// node from its queue before invoking destroy, so destroy may free the node.
struct RuntimeRetirementNode {
  RuntimeRetirementNode* next = nullptr;
  void* owner = nullptr;
  void (*destroy)(void*) noexcept = nullptr;
  bool queued = false;
};

struct RuntimeRetirementDiagnostics {
  std::size_t pending = 0;
  std::size_t peak_pending = 0;
  std::size_t completed = 0;
};

class RuntimeRetirementCoordinator {
 public:
  explicit RuntimeRetirementCoordinator(
      ThreadLauncher launcher = DefaultThreadLauncher());
  ~RuntimeRetirementCoordinator();

  RuntimeRetirementCoordinator(const RuntimeRetirementCoordinator&) = delete;
  RuntimeRetirementCoordinator& operator=(
      const RuntimeRetirementCoordinator&) = delete;

  void Retire(RuntimeRetirementNode& node) noexcept;
  void WaitIdle();
  [[nodiscard]] RuntimeRetirementDiagnostics Diagnostics() const;

 private:
  void Run() noexcept;

  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::condition_variable idle_;
  RuntimeRetirementNode* head_ = nullptr;
  RuntimeRetirementNode* tail_ = nullptr;
  std::size_t pending_ = 0;
  std::size_t peak_pending_ = 0;
  std::size_t completed_ = 0;
  bool stopping_ = false;
  std::thread worker_;
};

RuntimeRetirementCoordinator& GlobalRuntimeRetirementCoordinator();

}  // namespace open_lmm
