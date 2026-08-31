#include "support/benchmark/stage_event_recorder.hpp"
#include "support/benchmark/test_assert.hpp"

#include <iostream>
#include <string>

namespace {

using open_lmm::test::benchmark::Check;

open_lmm::ExecutionEvent Event(uint64_t sequence, open_lmm::EventType type,
                               uint64_t job, open_lmm::StageId stage) {
  open_lmm::ExecutionEvent event;
  event.sequence = sequence;
  event.type = type;
  event.job_id = job;
  event.stage = stage;
  return event;
}

}  // namespace

int main() {
  using namespace open_lmm;
  using namespace open_lmm::test::benchmark;
  StageEventRecorder recorder;
  recorder.RecordAt(Event(1, EventType::kJobStarted, 7, StageId::kDataLoad),
                    90);
  recorder.RecordAt(Event(2, EventType::kStageStarted, 7, StageId::kDataLoad),
                    100);
  recorder.RecordAt(Event(3, EventType::kProgressUpdated, 7,
                          StageId::kDataLoad),
                    120);
  recorder.RecordAt(Event(4, EventType::kStageCompleted, 7,
                          StageId::kDataLoad),
                    175);
  std::string error;
  const auto window = recorder.FindStageWindow(7, StageId::kDataLoad, &error);
  Check(window && window->start_sequence == 2 &&
            window->terminal_sequence == 4 && window->latency_ns == 75 &&
            window->terminal_type == EventType::kStageCompleted &&
            error.empty() && !recorder.DroppedEvent(),
        "recorder derives latency from callback entry and event sequence");

  StageEventRecorder invalid;
  invalid.RecordAt(Event(2, EventType::kStageStarted, 8, StageId::kSave), 10);
  invalid.RecordAt(Event(1, EventType::kStageCompleted, 8, StageId::kSave), 20);
  Check(!invalid.FindStageWindow(8, StageId::kSave, &error) &&
            error.find("strictly increasing") != std::string::npos,
        "recorder fails closed on out-of-order event delivery");
  std::cout << "benchmark stage event recorder tests passed\n";
}
