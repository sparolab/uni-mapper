#pragma once

#include <chrono>
#include <condition_variable>
#include <cstddef>
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

  void Reset() {
    std::lock_guard lock(mutex_);
    signaled_ = false;
  }

  bool IsSignaled() const {
    std::lock_guard lock(mutex_);
    return signaled_;
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

class CountdownEvent {
 public:
  explicit CountdownEvent(std::size_t count) : remaining_(count) {
    if (count == 0) throw std::invalid_argument("countdown must be non-zero");
  }

  void Arrive() {
    {
      std::lock_guard lock(mutex_);
      if (remaining_ != 0) --remaining_;
    }
    condition_.notify_all();
  }

  bool WaitFor(std::chrono::steady_clock::duration timeout =
                   kWatchdogTimeout) const {
    std::unique_lock lock(mutex_);
    return condition_.wait_for(lock, timeout, [&] { return remaining_ == 0; });
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
  std::size_t remaining_;
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
