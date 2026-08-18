#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/gui/gui_controller_bridge.hpp>
#include <open_lmm/gui/config_editor.hpp>
#include <open_lmm/gui/gui_event_queue.hpp>
#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/gui/visualization_repository.hpp>
#include <open_lmm/gui/visualization_snapshot_worker.hpp>
#include <open_lmm/server/stage_runner.hpp>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

namespace {
open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

class FakeGui final : public open_lmm::GuiPlugin {
 public:
  open_lmm::Result<void> Start(open_lmm::GuiServices) override { started = open = true; return open_lmm::Result<void>::Ok(); }
  bool IsOpen() const override { return open; }
  void RequestStop() override { requested_stop = true; open = false; }
  void Join() override { joined = true; }
  bool started = false, requested_stop = false, joined = false, open = false;
};

class NoopRunner final : public open_lmm::StageRunner {
 public:
  void SetCancellationToken(
      std::shared_ptr<open_lmm::CancellationToken>) override {}
  open_lmm::Result<void> RunStage(open_lmm::StageId) override {
    return open_lmm::Result<void>::Ok();
  }
  open_lmm::Result<void> RunNode(
      open_lmm::NodeId, std::optional<open_lmm::AgentId>) override {
    return open_lmm::Result<void>::Ok();
  }
  open_lmm::Result<void> RunOptimizeThrough(const open_lmm::AgentId&) override {
    return open_lmm::Result<void>::Ok();
  }
  std::vector<open_lmm::AgentId> AgentIds() const override { return {Id("A")}; }
  open_lmm::Result<open_lmm::VisualizationSnapshot>
  CreateVisualizationSnapshot(const open_lmm::AgentId& agent) const override {
    open_lmm::VisualizationSnapshot snapshot;
    snapshot.agent = agent;
    snapshot.poses.push_back({});
    return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
        std::move(snapshot));
  }
};
void Require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}
}  // namespace

int main() {
  const std::string config_text = R"({
    "global": {
      "config_map_server":"map.json", "config_data_loader":"loader.json",
      "config_loop_detector":"loop.json", "config_backend_optimizer":"optimizer.json",
      "config_dynamic_remover":"remover.json"
    },
    "directory": {"root_dir_path":"/data", "sub_dir_list":["test1","test2"],
                  "root_save_dir":"/output"}
  })";
  auto config_document = open_lmm::ConfigEditorDocument::Parse(config_text);
  Require(config_document.IsOk(), "schema config document parses");
  auto config_values = config_document.Value().Values();
  Require(config_values.IsOk() && config_values.Value().sub_dir_list.size() == 2,
          "schema config fields are projected");
  auto invalid_values = config_values.Value();
  invalid_values.sub_dir_list.clear();
  auto mutable_config_document = std::move(config_document).Value();
  Require(!mutable_config_document.SetValues(invalid_values).IsOk(),
          "schema rejects an empty agent list");
  Require(!open_lmm::ConfigEditorDocument::Parse("{}").IsOk(),
          "schema rejects missing required fields");
  const auto dataset_root = std::filesystem::temp_directory_path() /
                            "open_lmm_gui_dataset_catalog_test";
  std::filesystem::remove_all(dataset_root);
  std::filesystem::create_directories(dataset_root / "agent_b");
  std::filesystem::create_directories(dataset_root / "agent_a");
  std::ofstream(dataset_root / "not_a_dataset.txt") << "ignored";
  auto datasets = open_lmm::DiscoverDatasetDirectories(dataset_root);
  Require(datasets.IsOk() && datasets.Value() ==
              std::vector<std::string>({"agent_a", "agent_b"}),
          "dataset catalog contains sorted child directories only");
  const auto alignment_path = dataset_root / "loop.json";
  std::ofstream(alignment_path)
      << R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"}})";
  auto alignment = open_lmm::LoadAlignmentConfig(alignment_path);
  Require(alignment.IsOk() && alignment.Value().kiss_voxel_size == 2.0,
          "alignment editor provides backward-compatible defaults");
  auto alignment_values = alignment.Value();
  alignment_values.kiss_voxel_size = 1.25;
  alignment_values.kiss_use_quatro = true;
  alignment_values.pose_nn_distance_threshold = 7.5;
  Require(open_lmm::SaveAlignmentConfig(alignment_path, alignment_values).IsOk(),
          "alignment editor saves validated values");
  const auto saved_before_invalid = [&] {
    std::ifstream input(alignment_path);
    return std::string(std::istreambuf_iterator<char>(input), {});
  }();
  auto invalid_alignment_values = alignment_values;
  invalid_alignment_values.kiss_voxel_size = -1.0;
  Require(!open_lmm::SaveAlignmentConfig(alignment_path,
                                         invalid_alignment_values).IsOk(),
          "alignment editor rejects values rejected by runtime schema");
  const auto saved_after_invalid = [&] {
    std::ifstream input(alignment_path);
    return std::string(std::istreambuf_iterator<char>(input), {});
  }();
  Require(saved_after_invalid == saved_before_invalid,
          "failed alignment save leaves the original file unchanged");
  auto saved_alignment = open_lmm::LoadAlignmentConfig(alignment_path);
  Require(saved_alignment.IsOk() && saved_alignment.Value().kiss_use_quatro &&
              saved_alignment.Value().pose_nn_distance_threshold == 7.5,
          "alignment editor reloads saved values");
  std::filesystem::remove_all(dataset_root);
  auto plugin = std::make_shared<FakeGui>();
  {
    open_lmm::GuiPluginHost host(plugin);
    Require(host.Start({}).IsOk(), "host starts plugin");
    Require(plugin->started && host.IsOpen(), "open state is forwarded");
    Require(!host.Start({}).IsOk(), "duplicate start is rejected");
  }
  Require(plugin->requested_stop, "host destructor requests stop");
  Require(plugin->joined, "host destructor joins plugin");
  open_lmm::GuiPluginHost null_host(nullptr);
  Require(!null_host.Start({}).IsOk(), "null plugin is rejected");
  auto missing = open_lmm::GuiPluginHost::Load("/definitely/missing/libopen_lmm_gui.so");
  Require(!missing.IsOk(), "missing GUI library returns failure");

  open_lmm::GuiEventQueue queue(2);
  open_lmm::ExecutionEvent progress;
  progress.job_id = 7;
  progress.type = open_lmm::EventType::kProgressUpdated;
  progress.progress_current = 1;
  progress.progress_total = 10;
  Require(queue.Push(progress), "initial progress is queued");
  progress.progress_current = 9;
  Require(queue.Push(progress), "progress update is coalesced");
  auto drained = queue.Drain(2);
  Require(drained.size() == 1 && drained[0].progress_current == 9,
          "only latest progress is retained");
  Require(queue.Stats().coalesced_progress == 1,
          "progress coalescing is counted");

  open_lmm::ExecutionEvent first;
  first.type = open_lmm::EventType::kJobStarted;
  open_lmm::ExecutionEvent second;
  second.type = open_lmm::EventType::kStageStarted;
  open_lmm::ExecutionEvent third;
  third.type = open_lmm::EventType::kJobCompleted;
  Require(queue.Push(first) && queue.Push(second), "critical events fill queue");
  Require(!queue.Push(third), "critical eviction requires resync");
  Require(queue.Stats().resync_required, "resync flag is set");
  queue.MarkResynchronized();
  Require(!queue.Stats().resync_required, "resync flag can be cleared");

  auto controller = std::make_shared<open_lmm::PipelineController>(
      std::make_shared<NoopRunner>());
  auto services = open_lmm::MakeGuiServices(controller);
  Require(services.apply_config(open_lmm::ConfigDomain::kOptimizer, 2).IsOk(),
          "command bridge applies config revision");
  Require(services.snapshot().config_revision == 2,
          "applied config revision is visible in snapshot");
  Require(services.node_descriptors().size() == 5,
          "node descriptors are exposed to GUI controls");
  auto visualization = services.visualization_snapshot(Id("A"));
  Require(visualization.IsOk() && visualization.Value().agent == Id("A"),
          "visualization snapshot is bridged without exposing runner state");
  int subscriber_one = 0;
  int subscriber_two = 0;
  auto subscription_one = services.subscribe_events(
      [&](const open_lmm::ExecutionEvent&) { ++subscriber_one; });
  {
    auto subscription_two = services.subscribe_events(
        [&](const open_lmm::ExecutionEvent&) { ++subscriber_two; });
    auto job = services.submit_run_all();
    Require(job.IsOk(), "command bridge submits RunAll");
    Require(controller->Wait(job.Value()).IsOk(), "bridged RunAll completes");
    Require(subscriber_one > 0 && subscriber_two > 0,
            "multiple subscribers receive events");
  }
  const int stopped_count = subscriber_two;
  auto second_job = services.submit_run_all();
  Require(second_job.IsOk(), "second bridged RunAll submits");
  Require(controller->Wait(second_job.Value()).IsOk(), "second RunAll completes");
  Require(subscriber_one > 0 && subscriber_two == stopped_count,
          "RAII subscription stops callbacks");
  controller.reset();
  Require(!services.submit_run_all().IsOk(),
          "expired controller is reported through command port");

  open_lmm::GuiModel model;
  open_lmm::PipelineSnapshot initial;
  initial.config_revision = 4;
  initial.agents = {Id("A"), Id("B")};
  model.Synchronize(initial);
  Require(model.ConfigRevision() == 4 && model.Agents().size() == 2,
          "snapshot initializes GUI projection");
  Require(model.CanSubmitCommand() && !model.CanCancel(),
          "idle model allows commands");

  open_lmm::ExecutionEvent queued;
  queued.job_id = 42;
  queued.sequence = 1;
  queued.type = open_lmm::EventType::kJobQueued;
  Require(model.Apply(queued), "first event is accepted");
  Require(model.EventLog().size() == 1, "accepted event is retained for log UI");
  Require(!model.CanSubmitCommand() && model.CanCancel(),
          "queued job changes command eligibility");
  open_lmm::ExecutionEvent stage_started;
  stage_started.job_id = 42;
  stage_started.sequence = 2;
  stage_started.type = open_lmm::EventType::kStageStarted;
  stage_started.stage = open_lmm::StageId::kDataLoad;
  Require(model.Apply(stage_started), "contiguous stage event is accepted");
  Require(model.Stage(open_lmm::StageId::kDataLoad).state ==
              open_lmm::GuiStageState::kRunning,
          "stage projection becomes running");
  open_lmm::ExecutionEvent gap = stage_started;
  gap.sequence = 4;
  Require(!model.Apply(gap), "sequence gap requests resynchronization");
  open_lmm::ExecutionEvent completed;
  completed.job_id = 42;
  completed.sequence = 3;
  completed.type = open_lmm::EventType::kJobCompleted;
  Require(model.Apply(completed), "completion event is accepted");
  Require(model.CanSubmitCommand() && !model.CanCancel(),
          "completed job re-enables commands");

  open_lmm::VisualizationRepository repository;
  auto revision_one = std::make_shared<open_lmm::VisualizationSnapshot>();
  revision_one->agent = Id("A");
  revision_one->revision = 1;
  revision_one->points.resize(10);
  auto first_update = repository.Commit(revision_one);
  Require(first_update.changed && first_update.remove_drawables.empty(),
          "first visualization revision creates drawables");
  Require(repository.Latest(Id("A"))->points.empty(),
          "visualization repository retains metadata without point cache");
  Require(!repository.Commit(revision_one).changed,
          "same visualization revision is idempotent");
  auto stale = std::make_shared<open_lmm::VisualizationSnapshot>();
  stale->agent = Id("A");
  Require(!repository.Commit(stale).changed,
          "older visualization revision cannot replace the latest");
  auto revision_two = std::make_shared<open_lmm::VisualizationSnapshot>();
  revision_two->agent = Id("A");
  revision_two->revision = 2;
  auto replacement = repository.Commit(revision_two);
  Require(replacement.changed && replacement.remove_drawables.size() == 4 &&
              repository.Latest(Id("A"))->revision == 2,
          "new revision atomically replaces logical drawables");
  Require(open_lmm::VisualizationRepository::MapName(Id("A"), 2) ==
              "agent/A/map/2",
          "drawable names are stable and revisioned");

  std::atomic<int> snapshot_calls = 0;
  open_lmm::VisualizationSnapshotWorker snapshot_worker(
      [&](const open_lmm::AgentId& agent) {
        const int call = ++snapshot_calls;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        if (agent == Id("B")) {
          return open_lmm::Result<open_lmm::VisualizationSnapshot>::Failure(
              open_lmm::Error::IoError("synthetic load failure"));
        }
        open_lmm::VisualizationSnapshot value;
        value.agent = agent;
        value.revision = static_cast<uint64_t>(call);
        return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
            std::move(value));
      });
  snapshot_worker.Request(Id("A"));
  while (snapshot_calls.load() < 1) std::this_thread::yield();
  snapshot_worker.Request(Id("A"));
  snapshot_worker.Request(Id("A"));
  while (snapshot_calls.load() < 2) std::this_thread::yield();
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  auto worker_results = snapshot_worker.Drain();
  Require(worker_results.size() == 2,
          "active request and latest coalesced revision both complete");
  snapshot_worker.Request(Id("B"));
  while (snapshot_calls.load() < 3) std::this_thread::yield();
  std::this_thread::sleep_for(std::chrono::milliseconds(15));
  auto failed_results = snapshot_worker.Drain();
  Require(failed_results.size() == 1 && !failed_results.front().result.IsOk(),
          "snapshot load failures cross the worker boundary as Result");
  snapshot_worker.Stop();
  snapshot_worker.Join();
  std::cout << "GUI plugin lifecycle tests passed\n";
  return 0;
}
