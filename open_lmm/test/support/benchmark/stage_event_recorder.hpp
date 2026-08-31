#pragma once

#include <open_lmm/common/runtime_api.hpp>

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm::test::benchmark {

struct TimedExecutionEvent {
  ExecutionEvent event;
  uint64_t callback_time_ns = 0;
};

struct StageEventWindow {
  uint64_t job_id = 0;
  StageId stage = StageId::kDataLoad;
  uint64_t start_sequence = 0;
  uint64_t terminal_sequence = 0;
  uint64_t latency_ns = 0;
  EventType terminal_type = EventType::kStageCompleted;
};

class StageEventRecorder {
 public:
  void Record(const ExecutionEvent& event) noexcept;
  void RecordAt(const ExecutionEvent& event, uint64_t callback_time_ns) noexcept;
  [[nodiscard]] std::vector<TimedExecutionEvent> Snapshot() const;
  [[nodiscard]] std::optional<StageEventWindow> FindStageWindow(
      uint64_t job_id, StageId stage, std::string* error = nullptr) const;
  [[nodiscard]] bool DroppedEvent() const noexcept;

 private:
  mutable std::mutex mutex_;
  std::vector<TimedExecutionEvent> events_;
  bool dropped_event_ = false;
};

}  // namespace open_lmm::test::benchmark
