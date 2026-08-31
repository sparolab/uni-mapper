#pragma once

#include <chrono>
#include <condition_variable>
#include <mutex>
#include <stdexcept>
#include <string>

namespace open_lmm::test {

inline constexpr auto kWatchdogTimeout = std::chrono::seconds(5);

class ManualResetEvent {
 public:
  void Signal() {
    {
      std::lock_guard lock(mutex_);
      signaled_ = true;
    }
    condition_.notify_all();
  }

  bool WaitFor(std::chrono::steady_clock::duration timeout =
                   kWatchdogTimeout) const {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return signaled_; });
  }

  void Wait(const char* description,
            std::chrono::steady_clock::duration timeout =
                kWatchdogTimeout) const {
    if (!WaitFor(timeout)) {
      throw std::runtime_error(std::string("test watchdog expired: ") +
                               description);
    }
  }

 private:
  mutable std::mutex mutex_;
  mutable std::condition_variable condition_;
  bool signaled_ = false;
};

class PhaseGate {
 public:
  void ArriveAndWait(const char* description) {
    entered_.Signal();
    release_.Wait(description);
  }

  void WaitUntilEntered(const char* description) const {
    entered_.Wait(description);
  }

  void Release() { release_.Signal(); }

 private:
  ManualResetEvent entered_;
  ManualResetEvent release_;
};

}  // namespace open_lmm::test
