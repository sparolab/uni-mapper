#include "fixture_generator.hpp"

#include "fixture_manifest.hpp"
#include "tools/replay/replay_sha256.hpp"

#include <charconv>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>
#include <pcl/point_types.h>

namespace open_lmm::test::benchmark {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

struct Shape {
  std::size_t agents = 0;
  std::size_t scans_per_agent = 0;
  std::size_t points_per_scan = 0;
};

Shape FixtureShape(const std::string& fixture_id) {
  if (fixture_id == "small-v1") return {2, 8, 4096};
  if (fixture_id == "medium-v1") return {2, 64, 32768};
  throw std::invalid_argument("unsupported generated fixture: " + fixture_id);
}

void WriteJson(const fs::path& path, const Json& value) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("failed to create " + path.string());
  output << value.dump(2) << '\n';
  if (!output) throw std::runtime_error("failed to write " + path.string());
}

std::string AgentName(std::size_t index) {
  std::ostringstream name;
  name << "agent-" << std::setw(2) << std::setfill('0') << index;
  return name.str();
}

void WriteFixed6(std::ostream& output, double value) {
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value,
                                    std::chars_format::fixed, 6);
  if (result.ec != std::errc{}) {
    throw std::runtime_error("failed to format deterministic fixture value");
  }
  output.write(buffer, result.ptr - buffer);
}

void WritePoseFile(const fs::path& path, std::size_t scans,
                   std::size_t agent_index) {
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("failed to create " + path.string());
  static_cast<void>(agent_index);
  for (std::size_t frame = 0; frame < scans; ++frame) {
    const double x = static_cast<double>(frame) * 3.0;
    constexpr double y = 0.0;
    output << "1.000000 0.000000 0.000000 ";
    WriteFixed6(output, x);
    output << " 0.000000 1.000000 0.000000 ";
    WriteFixed6(output, y);
    output << " 0.000000 0.000000 1.000000 0.000000\n";
  }
  if (!output) throw std::runtime_error("failed to write " + path.string());
}

void WriteScan(const fs::path& path, std::size_t points,
               std::size_t agent_index, std::size_t frame) {
  std::ofstream output(path, std::ios::binary);
  if (!output) throw std::runtime_error("failed to create " + path.string());
  output << "# .PCD v0.7\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH " << points << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << points << "\n"
         << "DATA ascii\n";
  constexpr double kPi = 3.14159265358979323846;
  for (std::size_t index = 0; index < points; ++index) {
    const std::size_t sector = index % 128;
    const std::size_t band = (index / 128) % 256;
    const double angle = 2.0 * kPi * static_cast<double>(sector) / 128.0;
    const double radius = 6.0 + 0.125 * static_cast<double>(band);
    const double local_x = radius * std::cos(angle);
    const double local_y = radius * std::sin(angle);
    const double z = index % 7 == 0
                         ? 2.0 + 0.05 * static_cast<double>(band % 20)
                         : -1.0 + 0.01 * static_cast<double>(index % 11);
    const double intensity = static_cast<double>(
        (index + frame * 17 + agent_index * 31) % 256);
    WriteFixed6(output, local_x);
    output.put(' ');
    WriteFixed6(output, local_y);
    output.put(' ');
    WriteFixed6(output, z);
    output.put(' ');
    WriteFixed6(output, intensity);
    output.put('\n');
  }
  if (!output) throw std::runtime_error("failed to write " + path.string());
}

void WriteConfiguration(const fs::path& config, const Shape& shape,
                        const FixtureGenerationOptions& options) {
  std::vector<std::string> agents;
  for (std::size_t index = 0; index < shape.agents; ++index) {
    agents.push_back(AgentName(index));
  }
  WriteJson(config / "config.json",
            {{"global",
              {{"config_map_server", "server/map_server.json"},
               {"config_data_loader", "core/data_loader.json"},
               {"config_loop_detector", "core/loop_detector.json"},
               {"config_backend_optimizer", "core/optimizer.json"},
               {"config_dynamic_remover", "core/remover.json"}}},
             {"directory",
              {{"root_dir_path", "../data"},
               {"sub_dir_list", agents},
               {"root_save_dir", "../output"}}}});
  WriteJson(config / "server/map_server.json",
            {{"map_server",
              {{"enable_map_updater", options.enable_map_update},
               {"anchor_agent_index", 0},
               {"save_voxel_size", 0.4},
               {"parallel_data_load", false},
               {"parallel_map_update", options.parallel_map_update},
               {"max_parallel_agents", 2}}}});
  WriteJson(config / "core/data_loader.json",
            {{"data_loader",
              {{"data_loader_type", "file_based"},
               {"pose_format", "kitti"},
               {"pose_file_name", "poses.txt"},
               {"extrinsic", {0, 0, 0, 0, 0, 0, 1}},
               {"scan_type", "pcd"},
               {"scan_dir_name", "Scans"},
               {"voxel_size", 0.1},
               {"min_range", 1.0},
               {"max_range", 60.0},
               {"delimiter", " "}}}});
  WriteJson(config / "core/loop_detector.json",
            {{"loop_detector",
              {{"loop_detector_type", "kdtree"},
               {"model", "scan_context"},
               {"num_ring", 20},
               {"num_sector", 60},
               {"max_range", 60.0}}},
             {"database",
              {{"descriptor_vector_dim", 20},
               {"distance_threshold", 0.2},
               {"num_candidates", 3},
               {"rebuild_threshold", 10}}},
             {"alignment",
              {{"pcm_translation_threshold", 10.0},
               {"pcm_rotation_threshold_deg", 20.0},
               {"pcm_solver", "heuristic"},
               {"pcm_threads", 1},
               {"pcm_max_candidates", 0},
               {"kiss_voxel_size", 1.0},
               {"kiss_use_quatro", false},
               {"pose_nn_distance_threshold", 4.0},
               {"feedback_mode", "automatic"},
               {"headless_policy", "kiss_only"}}}});
  WriteJson(config / "core/optimizer.json",
            {{"backend_optimizer",
              {{"backend_optimizer_type", "incremental"},
               {"relinearizeThreshold", 0.1},
               {"relinearizeSkip", 1},
               {"isam_extra_updates", 1},
               {"min_loop_frame_gap", 30},
               {"icp_search_num", 1}}}});
  const std::string remover_model =
      options.enable_map_update ? "erasor" : "free_dom";
  Json remover = {{"dynamic_remover_type", "offline"},
                  {"model", remover_model}};
  if (options.enable_map_update) remover["internal_cpu_threads"] = 1;
  WriteJson(config / "core/remover.json",
            {{"dynamic_remover", std::move(remover)}});
}

Json FileRecord(const fs::path& root, const fs::path& path) {
  return {{"path", fs::relative(path, root).generic_string()},
          {"sha256", "sha256:" + replay::Sha256File(path)},
          {"bytes", fs::file_size(path)}};
}

}  // namespace

GeneratedFixture GenerateFixture(const fs::path& new_root,
                                 const std::string& fixture_id,
                                 FixtureGenerationOptions options) {
  if (options.parallel_map_update && !options.enable_map_update) {
    throw std::invalid_argument(
        "parallel MapUpdate requires MapUpdate to be enabled");
  }
  if (fs::exists(new_root)) {
    throw std::invalid_argument("fixture output path already exists: " +
                                new_root.string());
  }
  const Shape shape = FixtureShape(fixture_id);
  if (!fs::create_directories(new_root / "config") ||
      !fs::create_directories(new_root / "data") ||
      !fs::create_directories(new_root / "output")) {
    throw std::runtime_error("failed to create fixture root " +
                             new_root.string());
  }
  const fs::path config = new_root / "config";
  const fs::path data = new_root / "data";
  WriteConfiguration(config, shape, options);
  for (std::size_t agent = 0; agent < shape.agents; ++agent) {
    const fs::path directory = data / AgentName(agent);
    fs::create_directories(directory / "Scans");
    WritePoseFile(directory / "poses.txt", shape.scans_per_agent, agent);
    for (std::size_t frame = 0; frame < shape.scans_per_agent; ++frame) {
      std::ostringstream filename;
      filename << std::setw(6) << std::setfill('0') << frame << ".pcd";
      WriteScan(directory / "Scans" / filename.str(), shape.points_per_scan,
                agent, frame);
    }
  }

  Json config_files = Json::array();
  Json input_files = Json::array();
  uint64_t on_disk_bytes = 0;
  for (const auto& entry : fs::recursive_directory_iterator(new_root)) {
    if (!entry.is_regular_file()) continue;
    if (entry.path().parent_path() == new_root / "output") continue;
    const auto record = FileRecord(new_root, entry.path());
    on_disk_bytes += record.at("bytes").get<uint64_t>();
    if (entry.path().string().starts_with(config.string())) {
      config_files.push_back(record);
    } else if (entry.path().string().starts_with(data.string())) {
      input_files.push_back(record);
    }
  }
  const auto by_path = [](const Json& left, const Json& right) {
    return left.at("path").get<std::string>() <
           right.at("path").get<std::string>();
  };
  std::sort(config_files.begin(), config_files.end(), by_path);
  std::sort(input_files.begin(), input_files.end(), by_path);

  const uint64_t total_scans = shape.agents * shape.scans_per_agent;
  const uint64_t decoded_points = total_scans * shape.points_per_scan;
  Json manifest = {
      {"schema_version", 1},
      {"fixture_id", fixture_id},
      {"fixture_version", 1},
      {"generator_version", "open-lmm-benchmark-fixture-v1"},
      {"seed", 20260820},
      {"source_kind", "generated"},
      {"agent_count", shape.agents},
      {"scans_per_agent", shape.scans_per_agent},
      {"total_scan_count", total_scans},
      {"points_per_scan", shape.points_per_scan},
      {"decoded_point_count", decoded_points},
      {"sizeof_point", sizeof(pcl::PointXYZI)},
      {"decoded_point_bytes", decoded_points * sizeof(pcl::PointXYZI)},
      {"pose_count", total_scans},
      {"on_disk_bytes", on_disk_bytes},
      {"config_files", std::move(config_files)},
      {"input_files", std::move(input_files)},
      {"plugins",
       {{{"id", "builtin.scan_context"}, {"capability", "descriptor"}},
        {{"id", options.enable_map_update ? "builtin.erasor"
                                           : "builtin.free_dom"},
         {"capability", "dynamic_remover"}}}},
      {"voxel_sizes",
       {{"input_m", 0.1}, {"output_m", 0.4}, {"visualization_m", 0.4}}},
      {"resource_budget",
       {{"max_agent_tasks", 2},
        {"max_cpu_threads", 2},
        {"soft_memory_bytes", 4ULL * 1024ULL * 1024ULL * 1024ULL},
        {"enable_map_update", options.enable_map_update},
        {"parallel_map_update", options.parallel_map_update}}},
      {"license", "generated fixture; OpenLMM project license"},
      {"provenance", "deterministic generator seed 20260820"},
      {"redistribution", true}};
  const auto validation = ValidateFixtureManifest(manifest);
  if (!validation.Ok()) {
    throw std::runtime_error("generated invalid fixture manifest:\n" +
                             validation.Summary());
  }
  const fs::path manifest_path = new_root / "fixture_manifest.json";
  WriteJson(manifest_path, manifest);
  return {new_root, config, data, new_root / "output", manifest_path,
          std::move(manifest)};
}

ValidationResult VerifyMaterializedFixture(const fs::path& root,
                                           const Json& manifest) {
  ValidationResult result = ValidateFixtureManifest(manifest);
  if (!result.Ok()) return result;
  const fs::path canonical_root = fs::canonical(root);
  uint64_t observed_bytes = 0;
  for (const char* collection : {"config_files", "input_files"}) {
    for (std::size_t index = 0; index < manifest.at(collection).size();
         ++index) {
      const auto& record = manifest.at(collection).at(index);
      const fs::path relative = record.at("path").get<std::string>();
      const fs::path candidate = root / relative;
      const std::string pointer = std::string("/") + collection + "/" +
                                  std::to_string(index);
      std::error_code error;
      const auto status = fs::symlink_status(candidate, error);
      if (error || !fs::is_regular_file(status) || fs::is_symlink(status)) {
        result.issues.push_back(
            {pointer + "/path", "must resolve to a regular non-symlink file"});
        continue;
      }
      const fs::path canonical_file = fs::canonical(candidate, error);
      const fs::path relative_to_root =
          error ? fs::path{} : fs::relative(canonical_file, canonical_root, error);
      if (error || relative_to_root.empty() || relative_to_root.is_absolute() ||
          std::any_of(relative_to_root.begin(), relative_to_root.end(),
                      [](const fs::path& component) {
                        return component == "..";
                      })) {
        result.issues.push_back(
            {pointer + "/path", "canonical file escapes fixture root"});
        continue;
      }
      const uint64_t bytes = fs::file_size(candidate, error);
      if (error || bytes != record.at("bytes").get<uint64_t>()) {
        result.issues.push_back(
            {pointer + "/bytes", "materialized file size mismatch"});
        continue;
      }
      observed_bytes += bytes;
      const std::string digest = "sha256:" + replay::Sha256File(candidate);
      if (digest != record.at("sha256").get<std::string>()) {
        result.issues.push_back(
            {pointer + "/sha256", "materialized file digest mismatch"});
      }
    }
  }
  if (observed_bytes != manifest.at("on_disk_bytes").get<uint64_t>()) {
    result.issues.push_back(
        {"/on_disk_bytes", "materialized file byte total mismatch"});
  }
  return result;
}

}  // namespace open_lmm::test::benchmark
