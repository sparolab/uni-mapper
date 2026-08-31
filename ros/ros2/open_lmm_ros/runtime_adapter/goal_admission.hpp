#pragma once

#include <atomic>

namespace open_lmm {

class GoalAdmissionGate {
 public:
  bool TryReserve() {
    bool expected = false;
    return reserved_.compare_exchange_strong(
        expected, true, std::memory_order_acq_rel,
        std::memory_order_acquire);
  }
  void Release() { reserved_.store(false, std::memory_order_release); }
  [[nodiscard]] bool IsReserved() const {
    return reserved_.load(std::memory_order_acquire);
  }

 private:
  std::atomic<bool> reserved_{false};
};

}  // namespace open_lmm
