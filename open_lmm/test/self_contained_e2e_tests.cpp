#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>

#include <runtime/execution/pipeline.hpp>
#include <runtime/composition/map_server.hpp>
#include <runtime/resources/resource_governor.hpp>
#include <runtime/service/runtime_service.hpp>
#include <config/document/config.hpp>

namespace fs = std::filesystem;

namespace {

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

open_lmm::Result<open_lmm::ExecutionReceipt> Execute(
    open_lmm::MapServer& server, open_lmm::ExecutionCommand command) {
  const auto snapshot = server.Snapshot();
  open_lmm::ExecutionContext context{
      std::make_shared<open_lmm::CancellationToken>(),
      std::make_shared<open_lmm::AlignmentFeedbackBroker>(),
      snapshot.revision};
  return server.Execute(command, context);
}

class TemporaryTree {
 public:
  TemporaryTree() {
    const auto unique = std::to_string(::getpid()) + "_" +
        std::to_string(std::chrono::steady_clock::now()
                           .time_since_epoch().count());
    path_ = fs::temp_directory_path() / ("open_lmm_e2e_" + unique);
    fs::create_directories(path_);
  }

  ~TemporaryTree() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }

  const fs::path& path() const { return path_; }

 private:
  fs::path path_;
};

void WriteJson(const fs::path& path, const nlohmann::json& value) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  Require(static_cast<bool>(output), "open JSON fixture " + path.string());
  output << value.dump(2) << '\n';
  Require(static_cast<bool>(output), "write JSON fixture " + path.string());
}

nlohmann::json ReadJson(const fs::path& path) {
  std::ifstream input(path);
  Require(static_cast<bool>(input), "open JSON fixture " + path.string());
  nlohmann::json value;
  input >> value;
  return value;
}

void WritePoseFile(const fs::path& path) {
  std::ofstream output(path);
  Require(static_cast<bool>(output), "open pose fixture");
  for (int frame = 0; frame < 4; ++frame) {
    const double x = 12.0 * frame;
    output << "1 0 0 " << x << " 0 1 0 0 0 0 1 0\n";
  }
}

void WriteScan(const fs::path& path, int frame) {
  struct Point {
    float x;
    float y;
    float z;
    float intensity;
  };
  std::vector<Point> points;
  for (int x = 6; x <= 30; x += 2) {
    for (int y = -12; y <= 12; y += 2) {
      const float xf = static_cast<float>(x);
      const float yf = static_cast<float>(y);
      points.push_back({xf, yf, -1.0F + 0.05F * (x % 5),
                        static_cast<float>(frame)});
      points.push_back({xf, yf, 2.0F + 0.03F * (y + 12),
                        static_cast<float>(frame + 1)});
    }
  }
  for (int y = -10; y <= 10; y += 2) {
    for (int z = -2; z <= 4; ++z) {
      points.push_back({18.0F + 0.07F * z, static_cast<float>(y),
                        static_cast<float>(z), 9.0F});
    }
  }

  std::ofstream output(path);
  Require(static_cast<bool>(output), "open PCD fixture");
  output << "# .PCD v0.7\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << points.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points.size() << "\n"
         << "DATA ascii\n";
  for (const auto& point : points) {
    output << point.x << ' ' << point.y << ' ' << point.z << ' '
           << point.intensity << '\n';
  }
  Require(static_cast<bool>(output), "write PCD fixture");
}

void WriteAgent(const fs::path& root, const std::string& name) {
  const fs::path agent = root / name;
  fs::create_directories(agent / "Scans");
  WritePoseFile(agent / "poses.txt");
  for (int frame = 0; frame < 4; ++frame) {
    std::ostringstream name_stream;
    name_stream << std::setw(6) << std::setfill('0') << frame << ".pcd";
    WriteScan(agent / "Scans" / name_stream.str(), frame);
  }
}

void WriteIgnoredSparseFile(const fs::path& path, uint64_t bytes) {
  std::ofstream output(path, std::ios::binary);
  Require(static_cast<bool>(output), "create ignored sparse fixture");
  output.close();
  std::error_code error;
  fs::resize_file(path, bytes, error);
  Require(!error, "resize ignored sparse fixture");
}

void WriteConfiguration(const fs::path& config, const fs::path& data,
                        const fs::path& output,
                        bool parallel_data_load = false) {
  WriteJson(config / "config.json", {
      {"global", {
          {"config_map_server", "server/map_server.json"},
          {"config_data_loader", "core/data_loader.json"},
          {"config_loop_detector", "core/loop_detector.json"},
          {"config_backend_optimizer", "core/optimizer.json"},
          {"config_dynamic_remover", "core/remover.json"}}},
      {"directory", {
          {"root_dir_path", data.string()},
          {"sub_dir_list", {"agent1", "agent2"}},
          {"root_save_dir", output.string()}}}});
  WriteJson(config / "server/map_server.json", {
      {"map_server", {{"enable_map_updater", false},
                      {"anchor_agent_index", 0},
                      {"save_voxel_size", 0.4},
                      {"parallel_data_load", parallel_data_load},
                      {"parallel_map_update", false},
                      {"max_parallel_agents", parallel_data_load ? 2 : 1}}}});
  WriteJson(config / "core/data_loader.json", {
      {"data_loader", {{"data_loader_type", "file_based"},
                       {"pose_format", "kitti"},
                       {"pose_file_name", "poses.txt"},
                       {"extrinsic", {0, 0, 0, 0, 0, 0, 1}},
                       {"scan_type", "pcd"},
                       {"scan_dir_name", "Scans"},
                       {"voxel_size", 0.5},
                       {"min_range", 5.0},
                       {"max_range", 60.0},
                       {"delimiter", " "}}}});
  WriteJson(config / "core/loop_detector.json", {
      {"loop_detector", {{"loop_detector_type", "kdtree"},
                         {"model", "scan_context"},
                         {"num_ring", 20},
                         {"num_sector", 60},
                         {"max_range", 60.0}}},
      {"database", {{"descriptor_vector_dim", 20},
                    {"distance_threshold", 0.2},
                    {"num_candidates", 3},
                    {"rebuild_threshold", 10}}},
      {"alignment", {{"pcm_translation_threshold", 10.0},
                     {"pcm_rotation_threshold_deg", 20.0},
                     {"pcm_solver", "heuristic"},
                     {"pcm_threads", 1},
                     {"pcm_max_candidates", 0},
                     {"kiss_voxel_size", 1.0},
                     {"kiss_use_quatro", false},
                     {"pose_nn_distance_threshold", 4.0},
                     {"feedback_mode", "automatic"},
                     {"headless_policy", "kiss_only"}}}});
  WriteJson(config / "core/optimizer.json", {
      {"backend_optimizer", {{"backend_optimizer_type", "incremental"},
                             {"relinearizeThreshold", 0.1},
                             {"relinearizeSkip", 1},
                             {"isam_extra_updates", 1},
                             {"min_loop_frame_gap", 30},
                             {"icp_search_num", 1}}}});
  WriteJson(config / "core/remover.json", {
      {"dynamic_remover", {{"dynamic_remover_type", "offline"},
                           {"model", "free_dom"}}}});
}

std::size_t CountNamedFiles(const fs::path& root, const std::string& prefix,
                            const std::string& extension) {
  std::size_t count = 0;
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    const auto name = entry.path().filename().string();
    if (name.starts_with(prefix) && entry.path().extension() == extension) {
      ++count;
    }
  }
  return count;
}

fs::path FindNamedFile(const fs::path& root, const std::string& name) {
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (entry.is_regular_file() && entry.path().filename() == name) {
      return entry.path();
    }
  }
  return {};
}

std::vector<double> ReadNumbers(const fs::path& path) {
  std::ifstream input(path);
  Require(static_cast<bool>(input), "open numeric artifact " + path.string());
  std::vector<double> values;
  std::string line;
  while (std::getline(input, line)) {
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream fields(line);
    double value = 0.0;
    while (fields >> value) values.push_back(value);
    Require(fields.eof(), "parse numeric artifact " + path.string());
  }
  return values;
}

void RequireNear(const std::vector<double>& lhs, const std::vector<double>& rhs,
                 double tolerance, const std::string& message) {
  Require(lhs.size() == rhs.size(), message + " size");
  for (std::size_t index = 0; index < lhs.size(); ++index) {
    Require(std::abs(lhs[index] - rhs[index]) <= tolerance,
            message + " value " + std::to_string(index));
  }
}

open_lmm::BootstrapConfigSnapshot Bootstrap(const fs::path& directory) {
  auto loaded = open_lmm::LoadBootstrapConfig(directory);
  Require(loaded.IsOk(), "load bootstrap configuration " + directory.string());
  return std::move(loaded).Value();
}

}  // namespace

int main() {
  TemporaryTree fixture;
  const fs::path data = fixture.path() / "data";
  const fs::path config = fixture.path() / "config";
  const fs::path output = fixture.path() / "output";
  WriteAgent(data, "agent1");
  WriteAgent(data, "agent2");
  WriteIgnoredSparseFile(data / "agent1/ignored.payload", 128ULL << 20);
  WriteConfiguration(config, data, output);

  {
    open_lmm::RuntimeService production_runtime(1);
    auto opened = production_runtime.Open({config, "default-production-port"});
    Require(opened.IsOk(),
            "default RuntimeService factory opens the production port");
    Require(production_runtime.Close().IsOk(),
            "default RuntimeService production port closes cleanly");
  }

  const fs::path presentation_config =
      fixture.path() / "presentation-fault-config";
  const fs::path presentation_output =
      fixture.path() / "presentation-fault-output";
  WriteConfiguration(presentation_config, data, presentation_output);
  bool fail_next_presentation = true;
  open_lmm::MapServer presentation_server(
      Bootstrap(presentation_config), std::nullopt, {},
      [&fail_next_presentation] {
        if (!fail_next_presentation) return;
        fail_next_presentation = false;
        throw std::runtime_error("injected presentation failure");
      });
  const auto presentation_base = presentation_server.Snapshot().revision;
  auto presentation_load = Execute(
      presentation_server,
      open_lmm::ExecutionCommand::Stage(open_lmm::StageId::kDataLoad));
  Require(presentation_load &&
              presentation_load.Value().base_revision == presentation_base &&
              presentation_load.Value().committed_revision ==
                  presentation_base + 1 &&
              presentation_server.Snapshot().revision ==
                  presentation_load.Value().committed_revision,
          "post-commit presentation failure retains the valid receipt");
  Require(!presentation_server.Visualization(Id("agent1")),
          "failed committed presentation rolls back its candidate preview");
  auto presentation_alignment = Execute(
      presentation_server,
      open_lmm::ExecutionCommand::Stage(open_lmm::StageId::kAlignment));
  Require(presentation_alignment &&
              presentation_server.Snapshot().revision ==
                  presentation_alignment.Value().committed_revision &&
              presentation_server.Visualization(Id("agent1")),
          "a later presentation recovers after the committed fault");

  auto configured = open_lmm::LoadBootstrapConfig(config);
  Require(configured.IsOk(), "load self-contained configuration");

  const fs::path recovery_config = fixture.path() / "recovery-config";
  const fs::path recovery_output = fixture.path() / "recovery-output";
  WriteConfiguration(recovery_config, data, recovery_output);
  fs::create_directories(recovery_output / "agent_manifest.json/retained");
  std::ofstream(recovery_output / "agent_manifest.json/retained/entry")
      << "force committed manifest cleanup fault\n";
  auto recovery_bootstrap = open_lmm::LoadBootstrapConfig(recovery_config);
  Require(recovery_bootstrap.IsOk(), "load recovery configuration");
  open_lmm::MapServer recovery_server(
      std::move(recovery_bootstrap).Value(), recovery_output);
  const auto recovery_snapshot = recovery_server.Snapshot();
  Require(recovery_server.ValidateReady() &&
              recovery_snapshot.recovery_required &&
              recovery_snapshot.recovery_required->severity ==
                  open_lmm::Error::Severity::kFatalRuntime &&
              recovery_snapshot.recovery_required->context.runtime_revision ==
                  recovery_snapshot.revision,
          "bootstrap recovery keeps runtime structurally queryable and degraded");
  auto recovery_mutation = Execute(
      recovery_server,
      open_lmm::ExecutionCommand::Stage(open_lmm::StageId::kDataLoad));
  Require(!recovery_mutation &&
              recovery_mutation.GetError().severity ==
                  open_lmm::Error::Severity::kFatalRuntime &&
              recovery_server.Snapshot().revision == recovery_snapshot.revision,
          "bootstrap recovery rejects mutation before authority advances");

  const fs::path stage_recovery_config =
      fixture.path() / "stage-recovery-config";
  const fs::path stage_recovery_output =
      fixture.path() / "stage-recovery-output";
  WriteConfiguration(stage_recovery_config, data, stage_recovery_output);
  open_lmm::MapServer stage_recovery_server(
      Bootstrap(stage_recovery_config), stage_recovery_output);
  Require(stage_recovery_server.process().IsOk(),
          "prepare committed artifacts for stage recovery fault");
  const fs::path pose_destination =
      stage_recovery_output / "optimized_poses_agent1.txt";
  fs::remove(pose_destination);
  fs::create_directories(pose_destination / "retained-child");
  std::ofstream(pose_destination / "retained-child/entry") << "retain\n";
  const auto stage_recovery_before = stage_recovery_server.Snapshot();
  auto stage_recovery = Execute(
      stage_recovery_server,
      open_lmm::ExecutionCommand::Node(open_lmm::NodeId::kPoseSave));
  const auto stage_recovery_after = stage_recovery_server.Snapshot();
  Require(stage_recovery && stage_recovery.Value().recovery_required &&
              stage_recovery_after.revision ==
                  stage_recovery_before.revision + 1 &&
              stage_recovery_after.recovery_required &&
              fs::is_regular_file(pose_destination) &&
              fs::is_directory(pose_destination.string() +
                               ".open_lmm_backup"),
          "real Save commit preserves files, state, receipt, and recovery health");
  auto stage_recovery_rejected = Execute(
      stage_recovery_server,
      open_lmm::ExecutionCommand::Node(open_lmm::NodeId::kPoseSave));
  Require(!stage_recovery_rejected &&
              stage_recovery_rejected.GetError().severity ==
                  open_lmm::Error::Severity::kFatalRuntime &&
              stage_recovery_server.Visualization(Id("agent1")),
          "recovery health blocks later Save while preserving read-only queries");

  open_lmm::MapServer server(std::move(configured).Value());
  const uint64_t batch_base_revision = server.Snapshot().revision;
  auto completed = server.process();
  if (!completed) {
    std::cerr << completed.GetError().Message() << '\n';
  }
  Require(completed.IsOk(), "complete two-agent pipeline");

  const auto snapshot = server.Snapshot();
  Require(snapshot.revision ==
              batch_base_revision + open_lmm::PipelineStages().size(),
          "batch compatibility path commits one receipt per port command");
  Require(snapshot.ordered_agents ==
              std::vector<open_lmm::AgentId>({Id("agent1"), Id("agent2")}),
          "retain two ordered agents");
  Require(snapshot.descriptor_count == 8,
          "replaceable descriptor store contains one entry per scan");
  Require(CountNamedFiles(output, "optimized_poses_", ".txt") == 2,
          "save one optimized pose file per agent");
  Require(!FindNamedFile(output, "optimized_poses_agent1.txt").empty() &&
              !FindNamedFile(output, "optimized_poses_agent2.txt").empty(),
          "pose filenames preserve configured directory AgentIds");
  Require(CountNamedFiles(output, "global_map_", ".pcd") == 2,
          "save one fallback map per agent with Map Update disabled");
  auto map_before_pose_save = server.Visualization(Id("agent1"));
  Require(map_before_pose_save && map_before_pose_save.Value().map_available,
          "completed Save stage publishes the committed fallback map");
  auto pose_saved = Execute(
      server,
      open_lmm::ExecutionCommand::Node(open_lmm::NodeId::kPoseSave));
  Require(pose_saved.IsOk(), "run session PoseSave independently");
  auto map_after_pose_save = server.Visualization(Id("agent1"));
  Require(map_after_pose_save && map_after_pose_save.Value().map_available &&
              map_after_pose_save.Value().points.size() ==
                  map_before_pose_save.Value().points.size(),
          "PoseSave preserves an already committed map visualization");
  const auto fallback_before = server.Snapshot().revision;
  auto fallback_saved = Execute(
      server, open_lmm::ExecutionCommand::Node(
                  open_lmm::NodeId::kFallbackMapSave));
  Require(fallback_saved &&
              fallback_saved.Value().base_revision == fallback_before &&
              fallback_saved.Value().committed_revision ==
                  fallback_before + 1 &&
              fallback_saved.Value().affected_agents ==
                  std::vector<open_lmm::AgentId>({Id("agent1"), Id("agent2")}),
          "fallback map receipt derives both owners from artifact revisions");
  const fs::path manifest_path = FindNamedFile(output, "agent_manifest.json");
  Require(!manifest_path.empty(), "write the agent symbol manifest");
  std::ifstream manifest_input(manifest_path);
  nlohmann::json manifest;
  manifest_input >> manifest;
  Require(manifest["agents"].size() == 2 &&
              manifest["agents"][0]["id"] == "agent1" &&
              manifest["agents"][0]["symbol_byte"] == 1 &&
              manifest["agents"][1]["id"] == "agent2" &&
              manifest["agents"][1]["symbol_byte"] == 2,
          "manifest preserves config-order AgentId to byte mapping");

  const fs::path skip_config = fixture.path() / "skip-config";
  const fs::path skip_output = fixture.path() / "skip-output";
  WriteConfiguration(skip_config, data, skip_output);
  auto skip_map_config = ReadJson(skip_config / "server/map_server.json");
  skip_map_config["map_server"]["enable_map_updater"] = true;
  WriteJson(skip_config / "server/map_server.json", skip_map_config);
  open_lmm::MapServer skip_server(Bootstrap(skip_config));
  Require(Execute(skip_server, open_lmm::ExecutionCommand::Stage(
                                   open_lmm::StageId::kDataLoad)).IsOk() &&
              Execute(skip_server, open_lmm::ExecutionCommand::Stage(
                                   open_lmm::StageId::kAlignment)).IsOk(),
          "prepare optimized poses for fallback-map skip test");
  const auto skip_before = skip_server.Snapshot().revision;
  auto skipped_map = Execute(
      skip_server, open_lmm::ExecutionCommand::Node(
                       open_lmm::NodeId::kFallbackMapSave));
  Require(skipped_map &&
              skipped_map.Value().base_revision == skip_before &&
              skipped_map.Value().committed_revision == skip_before + 1 &&
              skip_server.Snapshot().revision ==
                  skipped_map.Value().committed_revision &&
              skipped_map.Value().affected_agents.empty() &&
              CountNamedFiles(skip_output, "global_map_", ".pcd") == 0,
          "fallback map node is an explicit no-op when Map Update is enabled");

  const auto valid_loop = ReadJson(config / "core/loop_detector.json");
  const auto valid_optimizer = ReadJson(config / "core/optimizer.json");
  const auto valid_remover = ReadJson(config / "core/remover.json");
  const auto valid_map = ReadJson(config / "server/map_server.json");
  WriteJson(config / "core/remover.json", nlohmann::json::object());
  Require(Execute(server, open_lmm::ExecutionCommand::Reconfigure(
                      open_lmm::ConfigDomain::kMapSave, 2)).IsOk(),
          "map-save reconfigure ignores unrelated invalid remover config");
  auto preserved = server.Visualization(Id("agent1"));
  Require(preserved && !preserved.Value().poses.empty() &&
              !preserved.Value().map_available,
          "map-save reconfigure preserves trajectory while invalidating map");
  WriteJson(config / "core/remover.json", valid_remover);
  WriteJson(config / "server/map_server.json", nlohmann::json::object());
  Require(Execute(server, open_lmm::ExecutionCommand::Reconfigure(
                      open_lmm::ConfigDomain::kDynamicRemover, 3)).IsOk(),
          "remover reconfigure ignores unrelated invalid map-save config");
  preserved = server.Visualization(Id("agent1"));
  Require(preserved && !preserved.Value().poses.empty(),
          "remover reconfigure preserves optimized trajectory");
  auto changed_identity_map = valid_map;
  changed_identity_map["map_server"]["anchor_agent_index"] = 1;
  WriteJson(config / "server/map_server.json", changed_identity_map);
  Require(!Execute(server, open_lmm::ExecutionCommand::Reconfigure(
                       open_lmm::ConfigDomain::kMapSave, 4)),
          "session identity field is rejected during map-save hot reload");
  WriteJson(config / "server/map_server.json", valid_map);
  WriteJson(config / "core/loop_detector.json", nlohmann::json::object());
  Require(Execute(server, open_lmm::ExecutionCommand::Reconfigure(
                      open_lmm::ConfigDomain::kOptimizer, 4)).IsOk(),
          "optimizer reconfigure ignores unrelated invalid loop config");
  auto after_optimizer_config = server.Visualization(Id("agent1"));
  Require(after_optimizer_config &&
              !after_optimizer_config.Value().poses.empty() &&
              after_optimizer_config.Value().points_available,
          "optimizer reconfigure preserves a valid committed presentation");
  WriteJson(config / "core/loop_detector.json", valid_loop);
  WriteJson(config / "core/optimizer.json", nlohmann::json::object());
  Require(Execute(server, open_lmm::ExecutionCommand::Reconfigure(
                      open_lmm::ConfigDomain::kLoopDetector, 5)).IsOk(),
          "loop reconfigure ignores unrelated invalid optimizer config");
  auto after_loop_config = server.Visualization(Id("agent1"));
  Require(after_loop_config && !after_loop_config.Value().poses.empty() &&
              after_loop_config.Value().points_available &&
              after_loop_config.Value().phase ==
                  open_lmm::VisualizationPhase::kDataLoad,
          "loop reconfigure preserves the valid DataLoad presentation");
  WriteJson(config / "core/optimizer.json", valid_optimizer);

  const auto reconfigured_snapshot = server.Snapshot();
  Require(reconfigured_snapshot.revision != 0,
          "publish reconfigured committed session snapshot");
  const uint64_t full_revision = reconfigured_snapshot.revision;
  auto replay_loop = Execute(server, open_lmm::ExecutionCommand::Node(
                                         open_lmm::NodeId::kLoopDetect,
                                         Id("agent2")));
  Require(replay_loop.IsOk(), "ordered LoopDetect replay for follower");
  auto loop_visualization = server.Visualization(
      open_lmm::VisualizationQuery{Id("agent2"), false});
  Require(loop_visualization &&
              loop_visualization.Value().phase ==
                  open_lmm::VisualizationPhase::kLoopDetection &&
              std::any_of(loop_visualization.Value().edges.begin(),
                          loop_visualization.Value().edges.end(),
                          [](const auto& edge) {
                            return edge.type !=
                                   open_lmm::VisualizationEdgeType::kTrajectory;
                          }),
          "LoopDetect commit publishes candidate loops instead of clearing them");
  auto replay_optimize = Execute(server, open_lmm::ExecutionCommand::Node(
                                             open_lmm::NodeId::kOptimize,
                                             Id("agent2")));
  Require(replay_optimize.IsOk(), "ordered Optimize replay for follower");
  const auto replayed = server.Snapshot();
  Require(replayed.revision == full_revision + 2,
          "commit one revision per ordered replay node");
  Require(replayed.descriptor_count == snapshot.descriptor_count,
          "ordered replay does not append duplicate descriptors");

  const fs::path parallel_config = fixture.path() / "parallel-config";
  const fs::path parallel_output = fixture.path() / "parallel-output";
  WriteConfiguration(parallel_config, data, parallel_output, true);
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{2, 2, 64ULL * 1024ULL * 1024ULL});
  uint64_t calibrated_resident_bytes = 0;
  {
    open_lmm::MapServer parallel_server(
        Bootstrap(parallel_config), std::nullopt, governor);
    auto parallel_completed = parallel_server.process();
    if (!parallel_completed) {
      std::cerr << parallel_completed.GetError().Message() << '\n';
    }
    Require(parallel_completed.IsOk(),
            "parallel DataLoad ignores unrelated large files for admission");
    const auto parallel_snapshot = parallel_server.Snapshot();
    Require(parallel_snapshot.ordered_agents == snapshot.ordered_agents &&
                parallel_snapshot.descriptor_count ==
                    snapshot.descriptor_count,
            "parallel DataLoad preserves agent order and descriptor cardinality");
    Require(governor->ReservedMemoryBytes() > 0,
            "committed raw payload keeps resident memory reserved");
    calibrated_resident_bytes = governor->ReservedMemoryBytes();
  }
  Require(governor->ReservedMemoryBytes() == 0,
          "destroyed session releases resident memory reservation");
  Require(CountNamedFiles(parallel_output, "optimized_poses_", ".txt") == 2 &&
              CountNamedFiles(parallel_output, "global_map_", ".pcd") == 2,
          "parallel DataLoad preserves deterministic output set");

  const uint64_t pressure_budget =
      calibrated_resident_bytes +
      std::max<uint64_t>(calibrated_resident_bytes / 4, 1);
  Require(pressure_budget > calibrated_resident_bytes &&
              pressure_budget < calibrated_resident_bytes * 2,
          "construct a budget that admits exactly one resident session");
  const fs::path pressure_config = fixture.path() / "pressure-config";
  WriteConfiguration(pressure_config, data, fixture.path() / "pressure-output",
                     true);
  auto pressure_governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{2, 2, pressure_budget});
  {
    open_lmm::MapServer first_session(
        Bootstrap(pressure_config), std::nullopt, pressure_governor);
    Require(Execute(first_session, open_lmm::ExecutionCommand::Stage(
                                       open_lmm::StageId::kDataLoad)).IsOk(),
            "first session is admitted under resident budget");
    const uint64_t initial_resident_bytes =
        pressure_governor->ReservedMemoryBytes();
    Require(Execute(first_session, open_lmm::ExecutionCommand::Node(
                                       open_lmm::NodeId::kDataLoad,
                                       Id("agent1"))).IsOk() &&
                pressure_governor->ReservedMemoryBytes() ==
                    initial_resident_bytes,
            "agent DataLoad replacement uses retiring resident ownership");
    auto data_load_visualization = first_session.Visualization(
        open_lmm::VisualizationQuery{Id("agent1"), false});
    Require(data_load_visualization &&
                data_load_visualization.Value().phase ==
                    open_lmm::VisualizationPhase::kDataLoad &&
                data_load_visualization.Value().pose_kind ==
                    open_lmm::VisualizationPoseKind::kOdometry &&
                !data_load_visualization.Value().poses.empty() &&
                data_load_visualization.Value().points.empty(),
            "DataLoad commit exposes odometry without eager point payload");
    const auto first_before = first_session.Snapshot();
    const uint64_t first_reserved = pressure_governor->ReservedMemoryBytes();
    Require(first_before.revision != 0 &&
                first_reserved == calibrated_resident_bytes,
            "first session owns its calibrated resident reservation");
    open_lmm::MapServer second_session(
        Bootstrap(pressure_config), std::nullopt, pressure_governor);
    auto rejected = Execute(second_session, open_lmm::ExecutionCommand::Stage(
                                                open_lmm::StageId::kDataLoad));
    Require(!rejected &&
                rejected.GetError().Message().find(
                    "memory admission rejected: class=resident_payload") !=
                    std::string::npos &&
                rejected.GetError().context.stage == "data_load" &&
                rejected.GetError().context.agent.has_value(),
            "actual filtered resident overflow is rejected with agent context");
    const auto first_after = first_session.Snapshot();
    Require(first_after.revision == first_before.revision &&
                pressure_governor->ReservedMemoryBytes() == first_reserved &&
                pressure_governor->MemoryAdmissionFailures() > 0,
            "failed second session preserves first committed state and budget");
  }
  Require(pressure_governor->ReservedMemoryBytes() == 0,
          "closing both sessions releases pressure-test reservations");

  std::cout << "self-contained two-agent E2E passed\n";
  return 0;
}
