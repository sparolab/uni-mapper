#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/gui/gui_controller_bridge.hpp>
#include <open_lmm/gui/config_editor.hpp>
#include <open_lmm/gui/gui_event_queue.hpp>
#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/gui/visualization_repository.hpp>
#include <open_lmm/gui/visualization_snapshot_worker.hpp>
#include <cstdlib>
#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
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

void Require(bool condition, const char* message) {
  if (!condition) { std::cerr << "FAILED: " << message << '\n'; std::exit(1); }
}

void WriteRuntimeFixture(const std::filesystem::path& config,
                         const std::filesystem::path& data,
                         const std::filesystem::path& output) {
  namespace fs = std::filesystem;
  fs::create_directories(data / "agent1/Scans");
  fs::create_directories(config / "server");
  fs::create_directories(config / "core");
  std::ofstream(data / "agent1/poses.txt")
      << "1 0 0 0 0 1 0 0 0 0 1 0\n";
  std::ofstream(data / "agent1/Scans/000000.pcd")
      << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
      << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 1\n"
      << "HEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS 1\nDATA ascii\n"
      << "10 0 0 1\n";
  std::ofstream(config / "config.json")
      << "{\"global\":{\"config_map_server\":\"server/map.json\","
      << "\"config_data_loader\":\"core/data.json\","
      << "\"config_loop_detector\":\"core/loop.json\","
      << "\"config_backend_optimizer\":\"core/optimizer.json\","
      << "\"config_dynamic_remover\":\"core/remover.json\"},"
      << "\"directory\":{\"root_dir_path\":\"" << data.string()
      << "\",\"sub_dir_list\":[\"agent1\"],\"root_save_dir\":\""
      << output.string() << "\"}}";
  std::ofstream(config / "server/map.json")
      << R"({"map_server":{"enable_map_updater":false,"anchor_agent_index":0,"save_voxel_size":0.2,"parallel_data_load":false,"parallel_map_update":false,"max_parallel_agents":1}})";
  std::ofstream(config / "core/data.json")
      << R"({"data_loader":{"data_loader_type":"file_based","pose_format":"kitti","pose_file_name":"poses.txt","extrinsic":[0,0,0,0,0,0,1],"scan_type":"pcd","scan_dir_name":"Scans","voxel_size":0.5,"min_range":1.0,"max_range":60.0,"delimiter":" "}})";
  std::ofstream(config / "core/loop.json")
      << R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"database":{"descriptor_vector_dim":20,"distance_threshold":0.15,"num_candidates":3,"rebuild_threshold":50},"alignment":{"pcm_translation_threshold":10.0,"pcm_rotation_threshold_deg":20.0,"pcm_solver":"heuristic","pcm_threads":1,"pcm_max_candidates":0}})";
  std::ofstream(config / "core/optimizer.json")
      << R"({"backend_optimizer":{"backend_optimizer_type":"incremental","relinearizeThreshold":0.1,"relinearizeSkip":1,"isam_extra_updates":1,"min_loop_frame_gap":30,"icp_search_num":1}})";
  std::ofstream(config / "core/remover.json")
      << R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom"}})";
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

  open_lmm::GuiEventQueue ordered_queue(8);
  progress.sequence = 10;
  Require(ordered_queue.Push(progress), "ordered progress is queued");
  open_lmm::ExecutionEvent intervening;
  intervening.job_id = 7;
  intervening.sequence = 11;
  intervening.type = open_lmm::EventType::kStageStarted;
  Require(ordered_queue.Push(intervening),
          "intervening lifecycle event is queued");
  progress.sequence = 12;
  progress.progress_current = 10;
  Require(ordered_queue.Push(progress),
          "non-tail progress stream is retained in sequence order");
  auto ordered = ordered_queue.Drain(8);
  Require(ordered.size() == 3 && ordered[0].sequence == 10 &&
              ordered[1].sequence == 11 && ordered[2].sequence == 12,
          "progress coalescing never reverses event sequence");

  progress.sequence = 13;
  Require(ordered_queue.Push(progress), "tail progress starts stream");
  progress.sequence = 14;
  Require(ordered_queue.Push(progress), "adjacent progress is coalesced");
  ordered = ordered_queue.Drain(8);
  Require(ordered.size() == 1 && ordered[0].sequence == 14,
          "tail coalescing retains the newest sequence");

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
  queue.ResetEpoch();
  Require(queue.Push(first), "old epoch event can be queued");
  queue.ResetEpoch();
  const auto reset_stats = queue.Stats();
  Require(reset_stats.queued == 0 && reset_stats.coalesced_progress == 0 &&
              reset_stats.evicted_events == 0 &&
              !reset_stats.resync_required,
          "epoch reset clears queued events and resync accounting");

  const auto bridge_root = std::filesystem::temp_directory_path() /
                           "open_lmm_gui_runtime_bridge_test";
  std::filesystem::remove_all(bridge_root);
  WriteRuntimeFixture(bridge_root / "config", bridge_root / "data",
                      bridge_root / "output-one");
  auto runtime = std::make_shared<open_lmm::RuntimeClient>(4);
  auto created = runtime->CreateSession({bridge_root / "config", "gui-test"});
  Require(created.IsOk(), "GUI runtime session is created");
  auto binding_result = open_lmm::RuntimeSessionClient::Create(
      runtime, created.Value());
  Require(binding_result.IsOk(), "GUI runtime session binding is created");
  auto binding = std::move(binding_result).Value();
  auto services = open_lmm::MakeGuiServices(
      binding, (bridge_root / "config/config.json").string());
  const auto initial_snapshot = services.snapshot();
  open_lmm::ConfigCandidate optimizer_candidate;
  optimizer_candidate.domain = open_lmm::ConfigDomain::kOptimizer;
  optimizer_candidate.document_json =
      R"({"backend_optimizer":{"backend_optimizer_type":"incremental","relinearizeThreshold":0.1,"relinearizeSkip":1,"isam_extra_updates":1,"min_loop_frame_gap":30,"icp_search_num":1}})";
  Require(services.apply_config(
              binding->CurrentSession(), std::move(optimizer_candidate),
              {initial_snapshot.session_revision,
               initial_snapshot.config_revision}).IsOk(),
          "command bridge applies config revision");
  Require(services.snapshot().config_revision == 2,
          "applied config revision is visible in snapshot");
  Require(services.node_descriptors().size() == 6,
          "node descriptors are exposed to GUI controls");
  std::atomic<int> subscriber_one{0};
  std::atomic<int> subscriber_two{0};
  std::atomic<int> ros_adapter_observer{0};
  std::atomic<uint64_t> last_gui_event_job{0};
  auto throwing_observer_result = binding->Subscribe(
      [](const open_lmm::SessionExecutionEvent&) {
        throw std::runtime_error("observer fault");
      });
  Require(throwing_observer_result.IsOk(), "throwing observer binds");
  auto throwing_observer = std::move(throwing_observer_result).Value();
  auto subscription_one_result = services.subscribe_events(
      [&](const open_lmm::ExecutionEvent& event) {
        ++subscriber_one;
        last_gui_event_job = event.job_id;
      });
  Require(subscription_one_result.IsOk(), "first GUI event subscriber binds");
  auto subscription_one = std::move(subscription_one_result).Value();
  auto ros_observer_result = binding->Subscribe(
      [&](const open_lmm::SessionExecutionEvent&) {
        ++ros_adapter_observer;
      });
  Require(ros_observer_result.IsOk(), "ROS adapter observer binds");
  auto ros_observer = std::move(ros_observer_result).Value();
  {
    auto subscription_two_result = services.subscribe_events(
        [&](const open_lmm::ExecutionEvent&) { ++subscriber_two; });
    Require(subscription_two_result.IsOk(), "second GUI event subscriber binds");
    auto subscription_two = std::move(subscription_two_result).Value();
    auto job = services.submit_run_all();
    Require(job.IsOk(), "command bridge submits RunAll");
    Require(runtime->Wait(binding->CurrentSession(), job.Value()).IsOk(),
            "bridged RunAll completes");
    auto visualization = services.visualization_snapshot(Id("agent1"));
    Require(visualization.IsOk() &&
                visualization.Value().agent == Id("agent1"),
            "visualization snapshot is bridged without exposing runner state");
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    throwing_observer.Reset();
    Require(subscriber_one.load() > 0 && subscriber_two.load() > 0,
            "throwing observer does not block later subscribers");
    auto session_node =
        services.submit_node(open_lmm::NodeId::kPoseSave, std::nullopt);
    Require(session_node &&
                runtime->Wait(binding->CurrentSession(),
                              session_node.Value()).IsOk(),
            "GUI bridge submits a session node without an agent target");
  }
  const int stopped_count = subscriber_two.load();
  auto second_job = services.submit_run_all();
  Require(second_job.IsOk(), "second bridged RunAll submits");
  Require(runtime->Wait(binding->CurrentSession(), second_job.Value()).IsOk(),
          "second RunAll completes");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Require(subscriber_one.load() > 0 &&
              subscriber_two.load() == stopped_count,
          "RAII subscription stops callbacks");
  open_lmm::GuiModel replacement_model;
  replacement_model.Synchronize(services.snapshot());
  Require(replacement_model.LastSequence() > 0,
          "old GUI epoch has a nonzero event sequence");
  const auto previous_session = binding->CurrentSession();
  auto root_document = open_lmm::ConfigEditorDocument::Load(
      bridge_root / "config/config.json");
  Require(root_document.IsOk(), "replacement root candidate loads");
  auto replacement_values = root_document.Value().Values();
  Require(replacement_values.IsOk(), "replacement root values project");
  auto values = std::move(replacement_values).Value();
  values.root_save_dir = (bridge_root / "output-two").string();
  auto replacement_document = std::move(root_document).Value();
  Require(replacement_document.SetValues(values).IsOk(),
          "replacement output root is valid");
  auto replacement_json = replacement_document.CanonicalJson();
  Require(replacement_json.IsOk(), "replacement root serializes");
  open_lmm::ConfigCandidate root_candidate;
  root_candidate.domain = open_lmm::ConfigDomain::kGlobal;
  root_candidate.document_json = std::move(replacement_json).Value();
  Require(services.replace_session(std::move(root_candidate)).IsOk(),
          "replacement atomically swaps the live session");
  const auto replacement_session = binding->CurrentSession();
  Require(replacement_session != previous_session,
          "DataLoad replacement changes SessionId");
  auto replacement_snapshot = runtime->RuntimeSnapshot(replacement_session);
  auto gui_runtime_snapshot = services.runtime_snapshot();
  Require(replacement_snapshot.IsOk() &&
              gui_runtime_snapshot.IsOk() &&
              gui_runtime_snapshot.Value().id == replacement_snapshot.Value().id &&
              gui_runtime_snapshot.Value().output_directory ==
                  replacement_snapshot.Value().output_directory &&
              gui_runtime_snapshot.Value().pipeline.session_revision ==
                  replacement_snapshot.Value().pipeline.session_revision &&
              gui_runtime_snapshot.Value().pipeline.config_revision ==
                  replacement_snapshot.Value().pipeline.config_revision &&
              replacement_snapshot.Value().output_directory.parent_path() ==
                  bridge_root / "output-two" &&
              replacement_snapshot.Value().pipeline.config_revision == 1,
          "replacement exposes its own output and initial revision");
  Require(!runtime->Snapshot(previous_session).IsOk(),
          "replaced session is closed after binding swap");
  open_lmm::GuiEventQueue replacement_events;
  replacement_events.ResetEpoch();
  replacement_model.Synchronize(services.snapshot());
  Require(replacement_model.LastSequence() == 0 &&
              replacement_model.SessionRevision() ==
                  services.snapshot().session_revision,
          "replacement resets the GUI event epoch from its new snapshot");
  auto replacement_model_subscription_result = services.subscribe_events(
      [&](const open_lmm::ExecutionEvent& event) {
        replacement_events.Push(event);
      });
  Require(replacement_model_subscription_result.IsOk(),
          "replacement model event observer binds");
  auto replacement_model_subscription =
      std::move(replacement_model_subscription_result).Value();
  const int before_rebound_event = subscriber_one.load();
  const int before_rebound_ros_event = ros_adapter_observer.load();
  auto rebound_job = services.submit_run_all();
  Require(rebound_job.IsOk(), "GUI submits through the rebound SessionId");
  auto synchronous_events = replacement_events.Drain(128);
  Require(!synchronous_events.empty() &&
              synchronous_events.front().type ==
                  open_lmm::EventType::kJobQueued &&
              synchronous_events.front().job_id == rebound_job.Value(),
          "synchronous queued event uses the reserved adapter token");
  for (const auto& event : synchronous_events) {
    Require(replacement_model.Apply(event),
            "replacement model accepts synchronous event sequence");
  }
  Require(rebound_job.Value() != second_job.Value(),
          "GUI job tokens remain unique when runtime JobIds restart");
  Require(!services.cancel_job(second_job.Value()).IsOk(),
          "stale pre-rebind GUI token cannot cancel the replacement job");
  for (int attempt = 0; attempt < 200; ++attempt) {
    auto current = services.runtime_snapshot();
    if (current && current.Value().pipeline.job &&
        current.Value().pipeline.job->state != open_lmm::JobState::kQueued &&
        current.Value().pipeline.job->state != open_lmm::JobState::kRunning) {
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  for (const auto& event : replacement_events.Drain(512)) {
    Require(replacement_model.Apply(event),
            "replacement model accepts live event sequence");
  }
  Require(replacement_model.Job() &&
              replacement_model.Job()->id == rebound_job.Value() &&
              replacement_model.Job()->state == open_lmm::JobState::kSucceeded,
          "replacement model reaches terminal state in adapter token space");
  const auto coherent_snapshot = services.snapshot();
  Require(coherent_snapshot.job &&
              coherent_snapshot.job->id == rebound_job.Value(),
          "GUI snapshot job uses the adapter token");
  for (const auto& event : coherent_snapshot.recent_events) {
    Require(event.job_id == 0 || event.job_id == rebound_job.Value(),
            "GUI snapshot recent events use the adapter token");
  }
  Require(subscriber_one.load() > before_rebound_event,
          "event subscription follows the replacement SessionId");
  Require(ros_adapter_observer.load() > before_rebound_ros_event,
          "ROS and GUI observers follow the same replacement SessionId");
  Require(last_gui_event_job.load() == rebound_job.Value(),
          "GUI events translate restarted runtime JobId to adapter token");
  uint64_t retired_token = rebound_job.Value();
  for (int repetition = 0; repetition < 4; ++repetition) {
    auto repeated = services.submit_run_all();
    Require(repeated.IsOk() && repeated.Value() != retired_token,
            "repeated runs receive unique adapter tokens");
    Require(!services.cancel_job(retired_token).IsOk(),
            "a subsequent run retires the prior terminal token");
    retired_token = repeated.Value();
    for (int attempt = 0; attempt < 200; ++attempt) {
      auto current = services.runtime_snapshot();
      if (current && current.Value().pipeline.job &&
          current.Value().pipeline.job->state != open_lmm::JobState::kQueued &&
          current.Value().pipeline.job->state != open_lmm::JobState::kRunning) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }

  std::atomic<bool> blocking_callback_entered{false};
  std::atomic<bool> release_blocking_callback{false};
  std::atomic<bool> reset_returned{false};
  auto blocking_result = binding->Subscribe(
      [&](const open_lmm::SessionExecutionEvent& event) {
        if (event.event.type != open_lmm::EventType::kJobQueued) return;
        blocking_callback_entered = true;
        while (!release_blocking_callback.load()) std::this_thread::yield();
      });
  Require(blocking_result.IsOk(), "blocking observer binds");
  auto blocking_subscription = std::move(blocking_result).Value();
  open_lmm::Result<open_lmm::BoundJob> blocked_submit =
      open_lmm::Result<open_lmm::BoundJob>::Failure(
          open_lmm::Error::InvalidArgument("submit not started"));
  std::thread submit_thread([&] {
    blocked_submit = binding->Submit(open_lmm::ExecutionRequest{});
  });
  for (int attempt = 0;
       attempt < 200 && !blocking_callback_entered.load(); ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  Require(blocking_callback_entered.load(), "blocking observer entered");
  std::thread reset_thread([&] {
    blocking_subscription.Reset();
    reset_returned = true;
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(10));
  Require(!reset_returned.load(),
          "subscription Reset waits for an in-flight callback");
  release_blocking_callback = true;
  submit_thread.join();
  reset_thread.join();
  Require(reset_returned.load() && blocked_submit.IsOk() &&
              binding->Wait(blocked_submit.Value()).IsOk(),
          "callback barrier releases after observer completion");

  open_lmm::ExecutionEventSubscription self_subscription;
  std::atomic<bool> self_reset_returned{false};
  auto self_result = binding->Subscribe(
      [&](const open_lmm::SessionExecutionEvent& event) {
        if (event.event.type != open_lmm::EventType::kJobQueued) return;
        self_subscription.Reset();
        self_reset_returned = true;
      });
  Require(self_result.IsOk(), "self-reset observer binds");
  self_subscription = std::move(self_result).Value();
  auto self_job = binding->Submit(open_lmm::ExecutionRequest{});
  Require(self_job && self_reset_returned.load() &&
              binding->Wait(self_job.Value()).IsOk(),
          "observer can reset itself without deadlock");

  const auto callback_replace_session = binding->CurrentSession();
  std::ifstream callback_root_input(bridge_root / "config/config.json");
  std::string callback_root_json{
      std::istreambuf_iterator<char>(callback_root_input),
      std::istreambuf_iterator<char>()};
  open_lmm::ConfigCandidate callback_root_candidate;
  callback_root_candidate.domain = open_lmm::ConfigDomain::kGlobal;
  callback_root_candidate.document_json = std::move(callback_root_json);
  std::atomic<bool> callback_replace_rejected{false};
  auto callback_replace_observer = binding->Subscribe(
      [&](const open_lmm::SessionExecutionEvent& event) {
        if (event.event.type != open_lmm::EventType::kJobQueued) return;
        callback_replace_rejected = !binding->ReplaceSession(
            {bridge_root / "config", "callback-replacement"},
            callback_root_candidate);
      });
  Require(callback_replace_observer.IsOk(),
          "callback replacement observer binds");
  auto callback_replace_subscription =
      std::move(callback_replace_observer).Value();
  auto callback_replace_job = binding->Submit(open_lmm::ExecutionRequest{});
  Require(callback_replace_job && callback_replace_rejected.load() &&
              binding->CurrentSession() == callback_replace_session &&
              binding->Wait(callback_replace_job.Value()).IsOk(),
          "replacement from an event callback is rejected before swap");

  binding.reset();
  runtime.reset();
  Require(!services.submit_run_all().IsOk(),
          "expired runtime binding is reported through command port");
  std::filesystem::remove_all(bridge_root);

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
