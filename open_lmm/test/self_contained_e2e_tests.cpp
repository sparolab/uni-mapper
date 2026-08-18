#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>

#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/server/map_server.hpp>
#include <open_lmm/utils/config.hpp>

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

void WriteConfiguration(const fs::path& config, const fs::path& data,
                        const fs::path& output) {
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
                      {"save_voxel_size", 0.4}}}});
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

}  // namespace

int main() {
  TemporaryTree fixture;
  const fs::path data = fixture.path() / "data";
  const fs::path config = fixture.path() / "config";
  const fs::path output = fixture.path() / "output";
  WriteAgent(data, "agent1");
  WriteAgent(data, "agent2");
  WriteConfiguration(config, data, output);

  auto configured = open_lmm::GlobalConfig::reload(config.string());
  Require(configured.IsOk(), "load self-contained configuration");

  open_lmm::MapServer server;
  auto completed = server.process();
  if (!completed) {
    std::cerr << completed.GetError().Message() << '\n';
  }
  Require(completed.IsOk(), "complete two-agent pipeline");

  const auto snapshot = server.SessionSnapshot();
  Require(snapshot.has_value(), "publish committed session snapshot");
  Require(snapshot->ordered_agents ==
              std::vector<open_lmm::AgentId>({Id("agent1"), Id("agent2")}),
          "retain two ordered agents");
  Require(snapshot->descriptor_count == 8,
          "replaceable descriptor store contains one entry per scan");
  Require(CountNamedFiles(output, "optimized_poses_", ".txt") == 2,
          "save one optimized pose file per agent");
  Require(!FindNamedFile(output, "optimized_poses_agent1.txt").empty() &&
              !FindNamedFile(output, "optimized_poses_agent2.txt").empty(),
          "pose filenames preserve configured directory AgentIds");
  Require(CountNamedFiles(output, "global_map_", ".pcd") == 2,
          "save one fallback map per agent with Map Update disabled");
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

  const uint64_t full_revision = snapshot->revision;
  auto replay_loop = server.RunNode(open_lmm::NodeId::kLoopDetect, Id("agent2"));
  Require(replay_loop.IsOk(), "ordered LoopDetect replay for follower");
  auto replay_optimize = server.RunNode(open_lmm::NodeId::kOptimize, Id("agent2"));
  Require(replay_optimize.IsOk(), "ordered Optimize replay for follower");
  const auto replayed = server.SessionSnapshot();
  Require(replayed && replayed->revision == full_revision + 2,
          "commit one revision per ordered replay node");
  Require(replayed->descriptor_count == snapshot->descriptor_count,
          "ordered replay does not append duplicate descriptors");

  std::cout << "self-contained two-agent E2E passed\n";
  return 0;
}
