#include "stage_event_recorder.hpp"

#include <chrono>

namespace open_lmm::test::benchmark {
namespace {

uint64_t NowNanoseconds() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

}  // namespace

void StageEventRecorder::Record(const ExecutionEvent& event) noexcept {
  RecordAt(event, NowNanoseconds());
}

void StageEventRecorder::RecordAt(const ExecutionEvent& event,
                                  uint64_t callback_time_ns) noexcept {
  try {
    std::lock_guard lock(mutex_);
    events_.push_back({event, callback_time_ns});
  } catch (...) {
    try {
      std::lock_guard lock(mutex_);
      dropped_event_ = true;
    } catch (...) {
    }
  }
}

std::vector<TimedExecutionEvent> StageEventRecorder::Snapshot() const {
  std::lock_guard lock(mutex_);
  return events_;
}

std::optional<StageEventWindow> StageEventRecorder::FindStageWindow(
    uint64_t job_id, StageId stage, std::string* error) const {
  const auto events = Snapshot();
  const auto fail = [&](std::string message)
      -> std::optional<StageEventWindow> {
    if (error) *error = std::move(message);
    return std::nullopt;
  };
  uint64_t previous_sequence = 0;
  std::optional<TimedExecutionEvent> started;
  for (const auto& timed : events) {
    if (timed.event.sequence <= previous_sequence) {
      return fail("execution event sequence is not strictly increasing");
    }
    previous_sequence = timed.event.sequence;
    if (timed.event.job_id != job_id || timed.event.stage != stage) continue;
    if (timed.event.type == EventType::kStageStarted) {
      if (started) return fail("stage has multiple start events");
      started = timed;
      continue;
    }
    if (timed.event.type != EventType::kStageCompleted &&
        timed.event.type != EventType::kStageFailed) {
      continue;
    }
    if (!started) return fail("stage terminal event precedes start");
    if (timed.callback_time_ns < started->callback_time_ns ||
        timed.event.sequence <= started->event.sequence) {
      return fail("stage terminal event has invalid order");
    }
    return StageEventWindow{job_id,
                            stage,
                            started->event.sequence,
                            timed.event.sequence,
                            timed.callback_time_ns - started->callback_time_ns,
                            timed.event.type};
  }
  return fail(started ? "stage terminal event is missing"
                      : "stage start event is missing");
}

bool StageEventRecorder::DroppedEvent() const noexcept {
  try {
    std::lock_guard lock(mutex_);
    return dropped_event_;
  } catch (...) {
    return true;
  }
}

}  // namespace open_lmm::test::benchmark
