#include <open_lmm/gui/gui_event_queue.hpp>

#include <algorithm>

namespace open_lmm {

GuiEventQueue::GuiEventQueue(size_t capacity)
    : capacity_(std::max<size_t>(capacity, 1)) {}

bool GuiEventQueue::IsProgress(const ExecutionEvent& event) {
  return event.type == EventType::kProgressUpdated;
}

bool GuiEventQueue::SameProgressStream(const ExecutionEvent& lhs,
                                       const ExecutionEvent& rhs) {
  return lhs.job_id == rhs.job_id && lhs.stage == rhs.stage &&
         lhs.node == rhs.node && lhs.agent == rhs.agent;
}

bool GuiEventQueue::Push(ExecutionEvent event) {
  std::lock_guard lock(mutex_);
  if (IsProgress(event)) {
    // Only the tail can be replaced without moving a newer sequence in front
    // of an older queued event (or creating a gap before that older event).
    if (!events_.empty() && IsProgress(events_.back()) &&
        SameProgressStream(events_.back(), event)) {
      events_.back() = std::move(event);
      ++coalesced_progress_;
      return true;
    }
  }

  bool preserved_without_resync = true;
  if (events_.size() == capacity_) {
    auto progress = std::find_if(events_.begin(), events_.end(), IsProgress);
    if (progress != events_.end()) {
      events_.erase(progress);
    } else {
      events_.pop_front();
      resync_required_ = true;
      preserved_without_resync = false;
    }
    ++evicted_events_;
  }
  events_.push_back(std::move(event));
  return preserved_without_resync;
}

std::vector<ExecutionEvent> GuiEventQueue::Drain(size_t max_events) {
  std::lock_guard lock(mutex_);
  const size_t count = std::min(max_events, events_.size());
  std::vector<ExecutionEvent> drained;
  drained.reserve(count);
  for (size_t i = 0; i < count; ++i) {
    drained.push_back(std::move(events_.front()));
    events_.pop_front();
  }
  return drained;
}

GuiEventQueueStats GuiEventQueue::Stats() const {
  std::lock_guard lock(mutex_);
  return {events_.size(), coalesced_progress_, evicted_events_,
          resync_required_};
}

void GuiEventQueue::ResetEpoch() {
  std::lock_guard lock(mutex_);
  events_.clear();
  coalesced_progress_ = 0;
  evicted_events_ = 0;
  resync_required_ = false;
}

void GuiEventQueue::MarkResynchronized() {
  std::lock_guard lock(mutex_);
  resync_required_ = false;
}

}  // namespace open_lmm
