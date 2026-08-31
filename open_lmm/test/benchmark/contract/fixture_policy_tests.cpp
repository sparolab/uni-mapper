#include "support/benchmark/fixture_manifest.hpp"
#include "support/benchmark/fixture_generator.hpp"

#include <chrono>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace {

using Json = nlohmann::json;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

Json ValidSmallManifest() {
  const std::string digest = "sha256:" + std::string(64, 'c');
  return {
      {"schema_version", 1},
      {"fixture_id", "small-v1"},
      {"fixture_version", 1},
      {"generator_version", "open-lmm-benchmark-fixture-v1"},
      {"seed", 20260820},
      {"source_kind", "generated"},
      {"agent_count", 2},
      {"scans_per_agent", 8},
      {"total_scan_count", 16},
      {"points_per_scan", 4096},
      {"decoded_point_count", 65536},
      {"sizeof_point", 16},
      {"decoded_point_bytes", 1048576},
      {"pose_count", 16},
      {"on_disk_bytes", 65536},
      {"config_files",
       {{{"path", "config/runtime.json"}, {"sha256", digest}, {"bytes", 12}}}},
      {"input_files",
       {{{"path", "agent-00/scans.bin"}, {"sha256", digest}, {"bytes", 32}}}},
      {"plugins",
       {{{"id", "builtin.scan_context"}, {"capability", "descriptor"}}}},
      {"voxel_sizes",
       {{"input_m", 0.1}, {"output_m", 0.2}, {"visualization_m", 0.4}}},
      {"resource_budget",
       {{"max_agent_tasks", 2},
        {"max_cpu_threads", 2},
        {"soft_memory_bytes", 1073741824},
        {"enable_map_update", false},
        {"parallel_map_update", false}}},
      {"license", "generated fixture; project license"},
      {"provenance", "deterministic generator seed 20260820"},
      {"redistribution", true}};
}

Json ReadJson(const std::filesystem::path& path) {
  std::ifstream input(path);
  Json value;
  input >> value;
  return value;
}

}  // namespace

int main() {
  using open_lmm::test::benchmark::ValidateFixtureManifest;
  auto manifest = ValidSmallManifest();
  Check(ValidateFixtureManifest(manifest).Ok(),
        "small-v1 manifest matches its immutable deterministic shape");

  auto wrong_shape = manifest;
  wrong_shape["points_per_scan"] = 4095;
  Check(!ValidateFixtureManifest(wrong_shape).Ok(),
        "fixture shape and decoded byte mismatch fail closed");

  auto traversal = manifest;
  traversal["input_files"][0]["path"] = "../outside.bin";
  Check(!ValidateFixtureManifest(traversal).Ok(),
        "fixture path traversal is rejected");

  auto unknown = manifest;
  unknown["wall_clock"] = "now";
  Check(!ValidateFixtureManifest(unknown).Ok(),
        "fixture manifest rejects nondeterministic unknown fields");

  const auto root = std::filesystem::temp_directory_path() /
                    ("open_lmm_benchmark_fixture_" +
                     std::to_string(static_cast<uint64_t>(getpid())) + "_" +
                     std::to_string(std::chrono::steady_clock::now()
                                        .time_since_epoch()
                                        .count()));
  const auto generated =
      open_lmm::test::benchmark::GenerateFixture(root, "small-v1");
  Check(ValidateFixtureManifest(generated.manifest).Ok() &&
            open_lmm::test::benchmark::VerifyMaterializedFixture(
                root, generated.manifest)
                .Ok() &&
            generated.manifest.at("decoded_point_count") == 65536 &&
            generated.manifest.at("config_files").size() == 6 &&
            generated.manifest.at("input_files").size() == 18 &&
            std::filesystem::is_regular_file(generated.manifest_path),
        "generator materializes the exact small-v1 input and manifest");
  std::ofstream(root / "data/agent-00/poses.txt", std::ios::app)
      << "tampered\n";
  Check(!open_lmm::test::benchmark::VerifyMaterializedFixture(
             root, generated.manifest)
             .Ok(),
        "materialized fixture size and digest are verified before execution");
  bool refused_overwrite = false;
  try {
    static_cast<void>(
        open_lmm::test::benchmark::GenerateFixture(root, "small-v1"));
  } catch (const std::invalid_argument&) {
    refused_overwrite = true;
  }
  std::error_code error;
  std::filesystem::remove_all(root, error);
  Check(refused_overwrite && !error,
        "fixture generator refuses overwrite and cleans its exact test root");

  const auto sequential_root = root.string() + "_sequential";
  const auto parallel_root = root.string() + "_parallel";
  const auto sequential = open_lmm::test::benchmark::GenerateFixture(
      sequential_root, "small-v1", {true, false});
  const auto parallel = open_lmm::test::benchmark::GenerateFixture(
      parallel_root, "small-v1", {true, true});
  auto sequential_map =
      ReadJson(sequential.config_directory / "server/map_server.json");
  auto parallel_map =
      ReadJson(parallel.config_directory / "server/map_server.json");
  sequential_map["map_server"]["parallel_map_update"] = true;
  Check(sequential_map == parallel_map &&
            ReadJson(sequential.config_directory / "core/remover.json") ==
                ReadJson(parallel.config_directory / "core/remover.json") &&
            sequential.manifest.at("plugins") ==
                parallel.manifest.at("plugins") &&
            sequential.manifest.at("plugins").at(1).at("id") ==
                "builtin.erasor",
        "sequential and parallel MapUpdate fixtures use the same isolated "
        "plugin and differ only in the execution-mode switch");
  std::filesystem::remove_all(sequential_root, error);
  Check(!error, "sequential comparison fixture cleanup succeeds");
  std::filesystem::remove_all(parallel_root, error);
  Check(!error, "parallel comparison fixture cleanup succeeds");
  std::cout << "benchmark fixture policy tests passed\n";
}
