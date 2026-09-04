#include "support/benchmark/fixture_generator.hpp"
#include "support/benchmark/process_window_sampler.hpp"
#include "support/benchmark/stage_event_recorder.hpp"
#include "support/check.hpp"

#include <open_lmm/server/runtime_client.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

#include <unistd.h>

namespace {

namespace fs = std::filesystem;
using namespace open_lmm;
using namespace open_lmm::test::benchmark;

class ScopedCurrentPath {
 public:
  explicit ScopedCurrentPath(const fs::path& path)
      : previous_(fs::current_path()) {
    fs::current_path(path);
  }
  ~ScopedCurrentPath() {
    std::error_code ignored;
    fs::current_path(previous_, ignored);
  }

 private:
  fs::path previous_;
};

}  // namespace

int main() {
  const fs::path root =
      fs::temp_directory_path() /
      ("open_lmm_benchmark_smoke_" +
       std::to_string(static_cast<uint64_t>(getpid())) + "_" +
       std::to_string(
           std::chrono::steady_clock::now().time_since_epoch().count()));
  const auto fixture = GenerateFixture(root, "small-v1");
  {
    ScopedCurrentPath working_directory(root);
    RuntimeClient client(2);

    ProcessWindowSampler open_sampler;
    open_sampler.Start();
    const auto opened = client.Open({"config", "benchmark-small-v1"});
    const auto open_window = open_sampler.Stop();
    Check(opened.IsOk(), opened ? "" : opened.GetError().Message());
    const auto opened_snapshot = client.Snapshot();
    Check(opened_snapshot && opened_snapshot.Value().pipeline.agents.size() == 2 &&
              open_window.wall_time_ns > 0 && open_window.sample_count >= 2,
          "P01 Open observes a ready two-agent runtime in one target window");

    auto recorder = std::make_shared<StageEventRecorder>();
    auto subscription = client.SubscribeEvents(
        [recorder](const ExecutionEvent& event) { recorder->Record(event); });
    Check(subscription.IsOk(), "benchmark subscriber attaches to runtime");
    ProcessWindowSampler load_sampler;
    load_sampler.Start();
    const auto submitted = client.Submit(
        {ExecutionRequestKind::kStage, StageId::kDataLoad});
    Check(submitted.IsOk(), submitted ? "" : submitted.GetError().Message());
    const auto completed = client.Wait(submitted.Value());
    const auto load_window = load_sampler.Stop();
    Check(completed.IsOk(),
          completed ? "" : completed.GetError().Message());
    std::string event_error;
    const auto stage = recorder->FindStageWindow(
        submitted.Value().value, StageId::kDataLoad, &event_error);
    const auto loaded_snapshot = client.Snapshot();
    Check(stage && stage->terminal_type == EventType::kStageCompleted &&
              loaded_snapshot &&
              loaded_snapshot.Value().pipeline.runtime_revision ==
                  opened_snapshot.Value().pipeline.runtime_revision + 1 &&
              load_window.cpu_time_ns && load_window.sampled_peak_rss_bytes &&
              load_window.io.rchar && !recorder->DroppedEvent(),
          "P02 DataLoad records ordered stage latency and process resources: " +
              event_error);

    const auto agent = AgentId::Parse("agent-00");
    Check(agent.IsOk(), "generated agent ID is valid");
    const auto visualization =
        client.Visualization({agent.Value(), false, 0.4F, 1});
    Check(visualization && visualization.Value().revision ==
                               loaded_snapshot.Value().pipeline.runtime_revision &&
              visualization.Value().points.empty() &&
              visualization.Value().source_point_count == 0,
          "DataLoad metadata query does not materialize a hidden point copy");
    Check(client.Close().IsOk(), "benchmark runtime closes cleanly");
  }
  std::error_code error;
  fs::remove_all(root, error);
  Check(!error, "benchmark smoke removes only its exact fixture root");
  std::cout << "benchmark small Open/DataLoad smoke passed\n";
}
