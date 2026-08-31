#include "../../tools/replay/replay_contract.hpp"
#include "../../tools/replay/replay_sha256.hpp"
#include "../../support/replay/test_support.hpp"

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <sys/wait.h>

namespace fs = std::filesystem;
namespace replay = open_lmm::test::replay;
using Json = nlohmann::json;

namespace {

using replay::Check;
using replay::TemporaryTree;

void WriteJson(const fs::path& path, const Json& value) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  Check(static_cast<bool>(output), "open JSON fixture");
  output << value.dump(2) << '\n';
  Check(static_cast<bool>(output), "write JSON fixture");
}

void WritePoseFile(const fs::path& path) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  Check(static_cast<bool>(output), "open pose fixture");
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

  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  Check(static_cast<bool>(output), "open PCD fixture");
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
  Check(static_cast<bool>(output), "write PCD fixture");
}

Json WriteAgent(const fs::path& data, const std::string& id) {
  const fs::path pose = data / id / "poses.txt";
  WritePoseFile(pose);
  std::string index;
  for (int frame = 0; frame < 4; ++frame) {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frame << ".pcd";
    const fs::path relative = fs::path(id) / "Scans" / filename.str();
    const fs::path scan = data / relative;
    WriteScan(scan, frame);
    index += std::to_string(frame) + " " + replay::Sha256File(scan) +
             "  " + relative.string() + "\n";
  }
  const fs::path scan_index = data / id / "scans.sha256";
  {
    std::ofstream output(scan_index);
    output << index;
    Check(static_cast<bool>(output), "write scan index");
  }
  return {{"id", id},
          {"frames", Json::array({0, 1, 2, 3})},
          {"pose_file",
           {{"path", (fs::path(id) / "poses.txt").string()},
            {"sha256", replay::Sha256File(pose)}}},
          {"scan_index",
           {{"path", (fs::path(id) / "scans.sha256").string()},
            {"sha256", replay::Sha256File(scan_index)}}}};
}

void WriteConfiguration(const fs::path& config) {
  WriteJson(config / "config.json",
            {{"global",
              {{"config_map_server", "server/map_server.json"},
               {"config_data_loader", "core/data_loader.json"},
               {"config_loop_detector", "core/loop_detector.json"},
               {"config_backend_optimizer", "core/optimizer.json"},
               {"config_dynamic_remover", "core/remover.json"}}},
             {"directory",
              {{"root_dir_path", "."},
               {"sub_dir_list", {"agent1", "agent2"}},
               {"root_save_dir", "."}}}});
  WriteJson(config / "server/map_server.json",
            {{"map_server",
              {{"enable_map_updater", false},
               {"anchor_agent_index", 0},
               {"save_voxel_size", 0.4},
               {"parallel_data_load", false},
               {"parallel_map_update", false},
               {"max_parallel_agents", 1}}}});
  WriteJson(config / "core/data_loader.json",
            {{"data_loader",
              {{"data_loader_type", "file_based"},
               {"pose_format", "kitti"},
               {"pose_file_name", "poses.txt"},
               {"extrinsic", {0, 0, 0, 0, 0, 0, 1}},
               {"scan_type", "pcd"},
               {"scan_dir_name", "Scans"},
               {"voxel_size", 0.5},
               {"min_range", 5.0},
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
  WriteJson(config / "core/remover.json",
            {{"dynamic_remover",
              {{"dynamic_remover_type", "offline"},
               {"model", "free_dom"}}}});
}

Json FileLock(const fs::path& root, const fs::path& relative) {
  return {{"path", relative.string()},
          {"sha256", replay::Sha256File(root / relative)}};
}

Json WriteCase(const fs::path& tree, const fs::path& data,
               const fs::path& config) {
  Json agents = Json::array(
      {WriteAgent(data, "agent1"), WriteAgent(data, "agent2")});
  const std::vector<fs::path> config_paths = {
      "config.json", "server/map_server.json", "core/data_loader.json",
      "core/loop_detector.json", "core/optimizer.json", "core/remover.json"};
  Json config_files = Json::array();
  for (const auto& path : config_paths) {
    config_files.push_back(FileLock(tree, fs::path("config") / path));
  }
  Json manifest{
      {"schema_version", 1},
      {"case_id", "synthetic-representative-v1"},
      {"tier", "tiny"},
      {"dataset",
       {{"bundle_id", "generated-synthetic-representative-v1"},
        {"bundle_sha256", replay::Sha256("generated fixture bundle v1")},
        {"source", "generated by replay_runner_e2e_tests"},
        {"license", "CC0-1.0"},
        {"redistributable", true},
        {"agents", std::move(agents)}}},
      {"provenance",
       {{"acquired_at", "2026-08-20"},
        {"source_version", "generator-v1"},
        {"attribution_file", "ATTRIBUTION.md"},
        {"transformations", Json::array({"deterministic generation"})},
        {"original_sha256", replay::Sha256("generated fixture source v1")}}},
      {"config",
       {{"root", "config/config.json"},
        {"files", std::move(config_files)},
        {"plugins",
         Json::array({{{"kind", "descriptor"},
                       {"id", "scan_context"},
                       {"capability", "descriptor-v1"}},
                      {{"kind", "dynamic_remover"},
                       {"id", "free_dom"},
                       {"capability", "offline-v1"}}})}}},
      {"workflow",
       Json::array({{{"stage", "DataLoad"}, {"agents", "all"}},
                    {{"stage", "Alignment"}, {"agents", "all"}},
                    {{"stage", "MapUpdate"}, {"agents", "all"}},
                    {{"stage", "Save"}, {"agents", "all"}}})},
      {"expected",
       {{"result", "success"},
        {"baseline_id", "synthetic-representative-baseline-v1"}}},
      {"tolerances",
       {{"pose_translation_m", 0.001},
        {"map_point_count_ratio", 0.01}}}};
  const fs::path manifest_path = tree / "case.json";
  WriteJson(manifest_path, manifest);
  return manifest;
}

std::string Quote(const fs::path& path) {
  const std::string value = path.string();
  Check(value.find('\'') == std::string::npos,
        "temporary test path does not contain a quote");
  return "'" + value + "'";
}

int RunRunner(const fs::path& manifest, const fs::path& data,
              const fs::path& config_root, const fs::path& output,
              const fs::path& report) {
  const std::string command =
      Quote(OPEN_LMM_REPLAY_RUNNER) + " --case " + Quote(manifest) +
      " --data-root " + Quote(data) + " --config-root " +
      Quote(config_root) + " --output-root " + Quote(output) +
      " --report " + Quote(report) +
      " --git-commit 0000000000000000000000000000000000000000" +
      " --container-digest " + replay::Sha256("fixture-container");
  return std::system(command.c_str());
}

void CheckSuccessfulProcess(int status, const std::string& operation) {
  Check(status != -1 && WIFEXITED(status) && WEXITSTATUS(status) == 0,
        operation);
}

std::string RebuildScanIndex(const fs::path& data, const std::string& agent,
                             const std::vector<int>& frames) {
  std::string index;
  for (int frame : frames) {
    std::ostringstream filename;
    filename << std::setw(6) << std::setfill('0') << frame << ".pcd";
    const fs::path relative = fs::path(agent) / "Scans" / filename.str();
    index += std::to_string(frame) + " " +
             replay::Sha256File(data / relative) + "  " +
             relative.string() + "\n";
  }
  const fs::path path = data / agent / "scans.sha256";
  std::ofstream output(path);
  output << index;
  Check(static_cast<bool>(output), "rewrite failure scan index");
  output.close();
  return replay::Sha256File(path);
}

bool HasSavedOutput(const fs::path& root) {
  for (const auto& entry : fs::recursive_directory_iterator(root)) {
    if (!entry.is_regular_file()) continue;
    const std::string name = entry.path().filename().string();
    if (name.starts_with("optimized_poses_") ||
        name.starts_with("global_map_") || name.ends_with(".tmp") ||
        name.find("open_lmm_backup") != std::string::npos) {
      return true;
    }
  }
  return false;
}

void TestRunnerEndToEnd() {
  TemporaryTree fixture("open_lmm_replay_e2e_");
  const fs::path data = fixture.Path() / "data";
  const fs::path config = fixture.Path() / "config";
  const fs::path output = fixture.Path() / "output";
  const fs::path report_path = fixture.Path() / "report.json";
  fs::create_directories(data);
  fs::create_directories(config);
  WriteConfiguration(config);
  std::ofstream(fixture.Path() / "ATTRIBUTION.md")
      << "Generated CC0 fixture.\n";
  const Json manifest = WriteCase(fixture.Path(), data, config);
  const fs::path manifest_path = fixture.Path() / "case.json";

  CheckSuccessfulProcess(
      RunRunner(manifest_path, data, fixture.Path(), output, report_path),
      "replay runner process succeeds");

  const Json report = replay::LoadJsonFile(report_path);
  const auto validation = replay::ValidateReplayReport(report);
  Check(validation.Ok(), "runner report validates:\n" + validation.Summary());
  Check(report.at("agents") == Json::array({"agent1", "agent2"}),
        "runner preserves manifest agent order");
  Check(report.at("steps").size() == 5 &&
            report.at("steps").front().at("stage") == "Open" &&
            report.at("steps").back().at("stage") == "Save",
        "runner records the complete ordered workflow");
  for (const Json& step : report.at("steps")) {
    Check(step.at("result") == "succeeded",
          "every synthetic replay step succeeds");
  }
  Check(report.at("close_result") == "succeeded" &&
            report.at("diagnostics").at("expectations_met") == true,
        "runner closes and satisfies case expectations");
  Check(report.at("metrics")
                .at("visualization")
                .at("agent1")
                .at("pose_count") == 4,
        "runner records deterministic pose cardinality");
  Check(report.at("metrics")
                .at("visualization")
                .at("agent1")
                .at("has_bounds") == true &&
            report.at("metrics")
                    .at("visualization")
                    .at("agent1")
                    .at("displayed_point_count")
                    .get<std::size_t>() > 0,
        "runner records real map bounds and point cardinality");

  Json baseline{
      {"schema_version", 1},
      {"baseline_id", "synthetic-representative-baseline-v1"},
      {"case_id", manifest.at("case_id")},
      {"case_manifest_sha256", replay::Sha256File(manifest_path)},
      {"generator", {{"kind", "test-fixture"}}},
      {"exact",
       Json::array({{{"pointer", "/agents"},
                     {"expected", Json::array({"agent1", "agent2"})}},
                    {{"pointer", "/close_result"},
                     {"expected", "succeeded"}}})},
      {"absolute_tolerances", Json::array()},
      {"ranges",
       Json::array({{{"pointer",
                      "/metrics/visualization/agent1/pose_count"},
                     {"minimum", 4},
                     {"maximum", 4}}})}};
  Check(replay::CompareReplayReport(baseline, report).Passed(),
        "manifest baseline accepts the generated report");
  Json reordered = report;
  reordered["agents"] = Json::array({"agent2", "agent1"});
  Check(!replay::CompareReplayReport(baseline, reordered).Passed(),
        "baseline rejects agent-order regression");

  const fs::path corrupt_scan = data / "agent2/Scans/000003.pcd";
  {
    std::ofstream output_stream(corrupt_scan);
    output_stream << "locked but invalid PCD\n";
    Check(static_cast<bool>(output_stream), "write invalid PCD fixture");
  }
  Json failure_manifest = manifest;
  failure_manifest["case_id"] = "synthetic-failure-v1";
  failure_manifest["tier"] = "failure";
  failure_manifest["dataset"]["bundle_id"] =
      "generated-synthetic-failure-v1";
  failure_manifest["dataset"]["bundle_sha256"] =
      replay::Sha256("generated failure fixture bundle v1");
  failure_manifest["dataset"]["agents"][1]["scan_index"]["sha256"] =
      RebuildScanIndex(data, "agent2", {0, 1, 2, 3});
  failure_manifest["expected"] =
      {{"result", "failure"},
       {"failure",
        {{"stage", "DataLoad"},
         {"error_code", "io_error"},
         {"revision_unchanged", true},
         {"close_succeeds", true}}}};
  const fs::path failure_case = fixture.Path() / "failure-case.json";
  const fs::path failure_output = fixture.Path() / "failure-output";
  const fs::path failure_report_path = fixture.Path() / "failure-report.json";
  WriteJson(failure_case, failure_manifest);
  const int failure_status =
      RunRunner(failure_case, data, fixture.Path(), failure_output,
                failure_report_path);
  if (!(failure_status != -1 && WIFEXITED(failure_status) &&
        WEXITSTATUS(failure_status) == 0) &&
      fs::is_regular_file(failure_report_path)) {
    std::cerr << replay::LoadJsonFile(failure_report_path).dump(2) << '\n';
  }
  CheckSuccessfulProcess(failure_status,
                         "expected failure replay process succeeds");
  const Json failure_report = replay::LoadJsonFile(failure_report_path);
  Check(replay::ValidateReplayReport(failure_report).Ok(),
        "failure replay report validates");
  Check(failure_report.at("steps").back().at("stage") == "DataLoad" &&
            failure_report.at("steps").back().at("result") == "failed" &&
            failure_report.at("steps").back().at("error_code") ==
                "io_error",
        "failure replay records authoritative DataLoad error");
  Check(failure_report.at("steps").back().at("revision_before") ==
            failure_report.at("steps").back().at("revision_after"),
        "failure replay preserves the committed revision");
  Check(failure_report.at("close_result") == "succeeded" &&
            failure_report.at("diagnostics").at("expectations_met") == true,
        "failure replay closes and satisfies rollback expectations");
  Check(!HasSavedOutput(failure_output),
        "failure replay leaves no Save or transaction residue");
}

}  // namespace

int main() {
  TestRunnerEndToEnd();
  std::cout << "Replay runner E2E tests passed\n";
  return 0;
}
