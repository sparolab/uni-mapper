#pragma once

#include <open_lmm/common/runtime_api.hpp>

#include <cstddef>
#include <deque>
#include <mutex>
#include <vector>

namespace open_lmm {

struct GuiEventQueueStats {
  size_t queued = 0;
  uint64_t coalesced_progress = 0;
  uint64_t evicted_events = 0;
  bool resync_required = false;
};

class GuiEventQueue {
 public:
  explicit GuiEventQueue(size_t capacity = 1024);
  bool Push(ExecutionEvent event);
  std::vector<ExecutionEvent> Drain(size_t max_events);
  [[nodiscard]] GuiEventQueueStats Stats() const;
  void ResetEpoch();
  void MarkResynchronized();
 private:
  static bool IsProgress(const ExecutionEvent& event);
  static bool SameProgressStream(const ExecutionEvent& lhs,
                                 const ExecutionEvent& rhs);
  mutable std::mutex mutex_;
  std::deque<ExecutionEvent> events_;
  size_t capacity_;
  uint64_t coalesced_progress_ = 0;
  uint64_t evicted_events_ = 0;
  bool resync_required_ = false;
};

}  // namespace open_lmm
