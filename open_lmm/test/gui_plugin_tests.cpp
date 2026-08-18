#include <open_lmm/gui/config_editor.hpp>
#include <open_lmm/gui/gui_controller_bridge.hpp>
#include <open_lmm/gui/gui_event_queue.hpp>
#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/gui/visualization_repository.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>

namespace {
namespace fs = std::filesystem;

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

void Require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

class FakeGui final : public open_lmm::GuiPlugin {
 public:
  open_lmm::Result<void> Start(open_lmm::GuiServices) override {
    started = open = true;
    return open_lmm::Result<void>::Ok();
  }
  bool IsOpen() const override { return open; }
  void RequestStop() override { requested_stop = true; open = false; }
  void Join() override { joined = true; }
  bool started = false;
  bool requested_stop = false;
  bool joined = false;
  bool open = false;
};

void WriteRuntimeFixture(const fs::path& config, const fs::path& data,
                         const fs::path& output) {
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

void TestGuiUtilities() {
  auto document = open_lmm::ConfigEditorDocument::Parse(R"({
    "global":{"config_map_server":"map.json","config_data_loader":"data.json",
    "config_loop_detector":"loop.json","config_backend_optimizer":"optimizer.json",
    "config_dynamic_remover":"remover.json"},
    "directory":{"root_dir_path":"/data","sub_dir_list":["a"],"root_save_dir":"/out"}})");
  Require(document && document.Value().Values(), "GUI config document is schema validated");
  Require(!open_lmm::ConfigEditorDocument::Parse("{}").IsOk(),
          "GUI rejects incomplete root config");

  open_lmm::GuiEventQueue queue(2);
  open_lmm::ExecutionEvent progress;
  progress.job_id = 7;
  progress.type = open_lmm::EventType::kProgressUpdated;
  progress.progress_current = 1;
  Require(queue.Push(progress), "queue accepts progress");
  progress.progress_current = 2;
  Require(queue.Push(progress) && queue.Drain(2).front().progress_current == 2,
          "queue coalesces adjacent progress safely");
  queue.ResetEpoch();
  Require(queue.Stats().queued == 0, "replacement epoch clears queued events");

  auto plugin = std::make_shared<FakeGui>();
  { open_lmm::GuiPluginHost host(plugin); Require(host.Start({}).IsOk(), "plugin host starts"); }
  Require(plugin->requested_stop && plugin->joined, "plugin host stops and joins");

  open_lmm::VisualizationRepository repository;
  auto snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  snapshot->agent = Id("A");
  snapshot->revision = 1;
  Require(repository.Commit(snapshot).changed, "visualization revision commits");
}

void TestSessionFreeBridgeAndModelReplacement() {
  const auto root = fs::temp_directory_path() / "open_lmm_gui_single_runtime";
  fs::remove_all(root);
  WriteRuntimeFixture(root / "config", root / "data", root / "output-one");
  auto runtime = std::make_shared<open_lmm::RuntimeClient>(1);
  auto opened = runtime->Open({root / "config", "gui"});
  if (!opened) std::cerr << "GUI runtime open: " << opened.GetError().Message() << '\n';
  Require(opened.IsOk(), "GUI opens the single RuntimeClient");
  auto services = open_lmm::MakeGuiServices(
      runtime, (root / "config/config.json").string());
  auto initial = services.snapshot();
  open_lmm::GuiModel model;
  model.Synchronize(initial);

  std::atomic<uint64_t> queued{0};
  auto subscription = services.subscribe_events([&](const open_lmm::ExecutionEvent& event) {
    if (event.type == open_lmm::EventType::kJobQueued) queued.store(event.job_id);
    model.Apply(event);
  });
  Require(subscription.IsOk(), "bridge subscribes without a runtime key");
  auto events = std::move(subscription).Value();
  auto job = services.submit_run_all();
  Require(job && job.Value() != 0, "bridge submits through unkeyed API");
  for (int attempt = 0; attempt != 200 && queued.load() == 0; ++attempt) {
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  Require(queued.load() == job.Value() && runtime->Wait({job.Value()}).IsOk(),
          "submitted handle and every queued event use one namespace");

  auto root_document = open_lmm::ConfigEditorDocument::Load(root / "config/config.json");
  Require(root_document.IsOk(), "root replacement document loads");
  auto values = root_document.Value().Values();
  Require(values.IsOk(), "root replacement values project");
  auto replacement_values = std::move(values).Value();
  replacement_values.root_save_dir = (root / "output-two").string();
  auto candidate_document = std::move(root_document).Value();
  Require(candidate_document.SetValues(replacement_values).IsOk(),
          "replacement root values validate");
  auto json = candidate_document.CanonicalJson();
  Require(json.IsOk(), "replacement root serializes canonically");
  open_lmm::ConfigCandidate root_candidate;
  root_candidate.domain = open_lmm::ConfigDomain::kGlobal;
  root_candidate.document_json = std::move(json).Value();
  const auto replacement_expected = services.runtime_snapshot().Value().pipeline;
  Require(services.replace_root_config(
              std::move(root_candidate),
              {replacement_expected.runtime_revision,
               replacement_expected.config_revision})
              .IsOk(),
          "GUI uses atomic root replacement rather than config pre-write");

  // The new controller restarts event sequence numbering.  Reset and
  // synchronize before accepting its events so the GUI never drops them as
  // stale events from the prior runtime epoch.
  model.Synchronize(services.snapshot());
  Require(model.LastSequence() == 0 &&
              services.runtime_snapshot().Value().output_directory.parent_path() ==
                  root / "output-two",
          "replacement resets the GUI model epoch and exposes new output");
  auto second = services.submit_run_all();
  Require(second && second.Value() != job.Value() &&
              runtime->Wait({second.Value()}).IsOk(),
          "job identity remains unique after GUI replacement");
  events.Reset();
  Require(runtime->Close().IsOk(), "GUI-owned runtime closes");
  fs::remove_all(root);
}

}  // namespace

int main() {
  TestGuiUtilities();
  TestSessionFreeBridgeAndModelReplacement();
  std::cout << "GUI plugin single-runtime tests passed\n";
  return 0;
}
