#include "../../tools/replay/replay_contract.hpp"
#include "../../tools/replay/replay_input_lock.hpp"
#include "../../tools/replay/replay_sha256.hpp"
#include "../../support/replay/test_support.hpp"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

#include <nlohmann/json.hpp>

namespace replay = open_lmm::test::replay;
using Json = nlohmann::json;

namespace {

constexpr const char* kDigest =
    "0123456789abcdef0123456789abcdef0123456789abcdef0123456789abcdef";
constexpr const char* kCommit = "0123456789abcdef0123456789abcdef01234567";
using replay::Check;
using replay::TemporaryTree;

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::filesystem::create_directories(path.parent_path());
  std::ofstream output(path, std::ios::binary);
  Check(static_cast<bool>(output), "open fixture file");
  output << text;
  Check(static_cast<bool>(output), "write fixture file");
}

bool InputVerificationThrows(const Json& manifest,
                             const std::filesystem::path& data,
                             const std::filesystem::path& config) {
  try {
    replay::VerifyReplayInputs(manifest, data, config);
    return false;
  } catch (const std::exception&) {
    return true;
  }
}

Json ValidCase() {
  return {
      {"schema_version", 1},
      {"case_id", "tiny-fixture-v1"},
      {"tier", "tiny"},
      {"dataset",
       {{"bundle_id", "fixture-bundle-v1"},
        {"bundle_sha256", kDigest},
        {"source", "fixture://generated"},
        {"license", "CC0-1.0"},
        {"redistributable", true},
        {"agents",
         Json::array({{{"id", "agent-a"},
                       {"frames", Json::array({1, 2, 3})},
                       {"pose_file", {{"path", "agent-a/poses.txt"}, {"sha256", kDigest}}},
                       {"scan_index",
                        {{"path", "agent-a/scans.sha256"},
                         {"sha256", kDigest}}}},
                      {{"id", "agent-b"},
                       {"frames", Json::array({4, 5, 6})},
                       {"pose_file", {{"path", "agent-b/poses.txt"}, {"sha256", kDigest}}},
                       {"scan_index",
                        {{"path", "agent-b/scans.sha256"},
                         {"sha256", kDigest}}}}})}}},
      {"provenance",
       {{"acquired_at", "2026-08-20"},
        {"source_version", "fixture-v1"},
        {"attribution_file", "ATTRIBUTION.md"},
        {"transformations", Json::array({"generated fixture"})},
        {"original_sha256", kDigest}}},
      {"config",
       {{"root", "configs/root.json"},
        {"files", Json::array({{{"path", "configs/root.json"}, {"sha256", kDigest}}})},
        {"plugins", Json::array({{{"kind", "descriptor"},
                                   {"id", "scan-context"},
                                   {"capability", "descriptor-v1"}}})}}},
      {"workflow",
       Json::array({{{"stage", "DataLoad"}, {"agents", "all"}},
                    {{"stage", "Alignment"}, {"agents", "all"}},
                    {{"stage", "MapUpdate"}, {"agents", "all"}},
                    {{"stage", "Save"}, {"agents", "all"}}})},
      {"expected", {{"result", "success"}, {"baseline_id", "tiny-baseline-v1"}}},
      {"tolerances", {{"pose_translation_m", 0.001}, {"map_point_count_ratio", 0.01}}}};
}

Json ValidReport() {
  return {
      {"schema_version", 1},
      {"case_id", "tiny-fixture-v1"},
      {"case_manifest_sha256", kDigest},
      {"dataset_sha256", kDigest},
      {"config_sha256", kDigest},
      {"git", {{"commit", kCommit}, {"dirty", false}}},
      {"environment", {{"compiler", "fixture"}, {"container", "fixture"}}},
      {"agents", Json::array({"agent-a", "agent-b"})},
      {"steps",
       Json::array({{{"stage", "DataLoad"},
                     {"result", "succeeded"},
                     {"error_code", nullptr},
                     {"revision_before", 0},
                     {"revision_after", 1},
                     {"artifacts", Json::array()},
                     {"events", Json::array()}}})},
      {"health", {{"state", "healthy"}, {"recovery_required", false}}},
      {"metrics", {{"pose", {{"translation_max_m", 0.0015}}},
                    {"map", {{"point_count_ratio", 0.02}}}}},
      {"artifacts", Json::array()},
      {"diagnostics", Json::object()},
      {"close_result", "succeeded"}};
}

Json ValidBaseline() {
  return {
      {"schema_version", 1},
      {"baseline_id", "tiny-baseline-v1"},
      {"case_id", "tiny-fixture-v1"},
      {"case_manifest_sha256", kDigest},
      {"generator", {{"commit", kCommit}, {"runs", 5}}},
      {"exact",
       Json::array({{{"pointer", "/agents"},
                     {"expected", Json::array({"agent-a", "agent-b"})}},
                    {{"pointer", "/close_result"}, {"expected", "succeeded"}}})},
      {"absolute_tolerances",
       Json::array({{{"pointer", "/metrics/pose/translation_max_m"},
                     {"expected", 0.001},
                     {"tolerance", 0.0005}}})},
      {"ranges",
       Json::array({{{"pointer", "/metrics/map/point_count_ratio"},
                     {"minimum", 0.0},
                     {"maximum", 0.02}}})}};
}

void TestCaseValidation() {
  const Json valid = ValidCase();
  const auto validation = replay::ValidateCaseManifest(valid);
  Check(validation.Ok(), "valid case accepted:\n" + validation.Summary());

  Json unknown = valid;
  unknown["parallel_owner"] = true;
  Check(!replay::ValidateCaseManifest(unknown).Ok(), "unknown top-level field rejected");

  Json traversal = valid;
  traversal["config"]["files"][0]["path"] = "../outside.json";
  Check(!replay::ValidateCaseManifest(traversal).Ok(), "path traversal rejected");

  Json unordered = valid;
  unordered["dataset"]["agents"][0]["frames"] = Json::array({2, 1});
  Check(!replay::ValidateCaseManifest(unordered).Ok(), "unordered frames rejected");

  Json missing_stage = valid;
  missing_stage["workflow"].erase(2);
  Check(!replay::ValidateCaseManifest(missing_stage).Ok(),
        "required workflow stage rejected");

  Json failure = valid;
  failure["tier"] = "failure";
  failure["expected"] = {
      {"result", "failure"},
      {"failure",
       {{"stage", "DataLoad"},
        {"agent", "agent-b"},
        {"error_code", "invalid_scan"},
        {"revision_unchanged", true},
        {"close_succeeds", true}}}};
  Check(replay::ValidateCaseManifest(failure).Ok(),
        "complete failure expectation accepted");
  failure["expected"]["failure"]["stage"] = "Open";
  Check(replay::ValidateCaseManifest(failure).Ok(),
        "Open failure expectation accepted");
  failure["expected"]["failure"].erase("revision_unchanged");
  Check(!replay::ValidateCaseManifest(failure).Ok(),
        "incomplete failure expectation rejected");

  Json reconfigure = valid;
  reconfigure["config"]["files"].push_back(
      {{"path", "configs/map-save.json"}, {"sha256", kDigest}});
  reconfigure["workflow"].push_back(
      {{"stage", "ApplyConfig"},
       {"agents", "all"},
       {"config_change",
        {{"domain", "map_save"},
         {"document", "configs/map-save.json"}}}});
  Check(replay::ValidateCaseManifest(reconfigure).Ok(),
        "locked domain config change accepted");
  reconfigure["workflow"].back()["config_change"]["document"] =
      "configs/unlocked.json";
  Check(!replay::ValidateCaseManifest(reconfigure).Ok(),
        "unlocked config change rejected");
}

void TestCanonicalJson() {
  Json left = {{"z", 1}, {"a", {{"d", 2}, {"b", 3}}}};
  Json right = {{"a", {{"b", 3}, {"d", 2}}}, {"z", 1}};
  Check(replay::CanonicalJson(left) == replay::CanonicalJson(right),
        "object insertion order does not affect canonical form");
}

void TestSha256() {
  Check(replay::Sha256("") ==
            "e3b0c44298fc1c149afbf4c8996fb924"
            "27ae41e4649b934ca495991b7852b855",
        "SHA-256 empty test vector");
  Check(replay::Sha256("abc") ==
            "ba7816bf8f01cfea414140de5dae2223"
            "b00361a396177a9cb410ff61f20015ad",
        "SHA-256 abc test vector");
  Check(replay::Sha256(std::string(1'000'000, 'a')) ==
            "cdc76e5c9914fb9281a1c7e284d73e67"
            "f1809a48a497200e046d39ccc7112cd0",
        "SHA-256 multi-block test vector");
}

void TestInputLocks() {
  TemporaryTree tree("open_lmm_replay_contract_");
  const auto data = tree.Path() / "data";
  const auto config = tree.Path() / "config";
  Json manifest = ValidCase();

  const auto config_path = config / "configs/root.json";
  WriteText(config_path, "{}\n");
  manifest["config"]["files"][0]["sha256"] =
      replay::Sha256File(config_path);

  std::filesystem::path first_scan;
  for (Json& agent : manifest["dataset"]["agents"]) {
    const std::string id = agent["id"];
    const auto pose = data / agent["pose_file"]["path"].get<std::string>();
    WriteText(pose, "pose-1\npose-2\npose-3\n");
    agent["pose_file"]["sha256"] = replay::Sha256File(pose);

    std::string index;
    for (const std::uint64_t frame :
         agent["frames"].get<std::vector<std::uint64_t>>()) {
      const auto relative = std::filesystem::path(id) / "Scans" /
                            (std::to_string(frame) + ".pcd");
      const auto scan = data / relative;
      WriteText(scan, "scan-" + id + "-" + std::to_string(frame) + "\n");
      if (first_scan.empty()) first_scan = scan;
      index += std::to_string(frame) + " " + replay::Sha256File(scan) +
               "  " + relative.string() + "\n";
    }
    const auto index_path =
        data / agent["scan_index"]["path"].get<std::string>();
    WriteText(index_path, index);
    agent["scan_index"]["sha256"] = replay::Sha256File(index_path);
  }

  replay::VerifyReplayInputs(manifest, data, config);
  WriteText(first_scan, "tampered scan\n");
  Check(InputVerificationThrows(manifest, data, config),
        "indexed scan tamper rejected before Open");

  WriteText(first_scan, "scan-agent-a-1\n");
  replay::VerifyReplayInputs(manifest, data, config);

  const auto outside_scan = tree.Path() / "outside-scan.pcd";
  WriteText(outside_scan, "scan-agent-a-1\n");
  std::filesystem::remove(first_scan);
  std::filesystem::create_symlink(outside_scan, first_scan);
  Check(InputVerificationThrows(manifest, data, config),
        "scan symlink escape rejected before Open");
  std::filesystem::remove(first_scan);
  WriteText(first_scan, "scan-agent-a-1\n");
  replay::VerifyReplayInputs(manifest, data, config);

  const auto unexpected_scan = first_scan.parent_path() / "unexpected.pcd";
  WriteText(unexpected_scan, "unexpected scan\n");
  Check(InputVerificationThrows(manifest, data, config),
        "unindexed scan rejected before Open");
  std::filesystem::remove(unexpected_scan);
  replay::VerifyReplayInputs(manifest, data, config);

  WriteText(config_path, "{\"tampered\":true}\n");
  Check(InputVerificationThrows(manifest, data, config),
        "config tamper rejected before Open");
}

void TestBaselineComparison() {
  const Json report = ValidReport();
  const Json baseline = ValidBaseline();
  Check(replay::ValidateReplayReport(report).Ok(), "valid report accepted");
  Check(replay::ValidateReplayBaseline(baseline).Ok(),
        "valid baseline accepted");
  Check(replay::CompareReplayReport(baseline, report).Passed(),
        "inclusive tolerance and range boundaries pass");

  Json exact_failure = report;
  exact_failure["agents"] = Json::array({"agent-b", "agent-a"});
  auto result = replay::CompareReplayReport(baseline, exact_failure);
  Check(!result.Passed() && result.differences.front().pointer == "/agents",
        "agent ordering is exact");

  Json tolerance_failure = report;
  tolerance_failure["metrics"]["pose"]["translation_max_m"] = 0.0015001;
  Check(!replay::CompareReplayReport(baseline, tolerance_failure).Passed(),
        "numeric value outside tolerance fails");

  Json range_failure = report;
  range_failure["metrics"]["map"]["point_count_ratio"] = 0.020001;
  Check(!replay::CompareReplayReport(baseline, range_failure).Passed(),
        "numeric value outside range fails");

  Json missing = report;
  missing["metrics"]["map"].erase("point_count_ratio");
  Check(!replay::CompareReplayReport(baseline, missing).Passed(),
        "missing baseline pointer fails closed");

  Json duplicate_rule = baseline;
  duplicate_rule["ranges"].push_back(
      {{"pointer", "/agents"}, {"minimum", 0}, {"maximum", 1}});
  Check(!replay::ValidateReplayBaseline(duplicate_rule).Ok(),
        "one comparison rule per pointer enforced");

  Json missing_expected = baseline;
  missing_expected["exact"][0].erase("expected");
  Check(!replay::ValidateReplayBaseline(missing_expected).Ok(),
        "exact rule requires an expected value");

  Json invalid_commit = report;
  invalid_commit["git"]["commit"] =
      "zzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzzz";
  Check(!replay::ValidateReplayReport(invalid_commit).Ok(),
        "non-hexadecimal commit rejected");
}

}  // namespace

int main() {
  TestCaseValidation();
  TestCanonicalJson();
  TestSha256();
  TestInputLocks();
  TestBaselineComparison();
  std::cout << "Replay contract tests passed\n";
  return 0;
}
