#include "benchmark_pair.hpp"

#include "benchmark_bundle.hpp"
#include "tools/replay/replay_sha256.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <initializer_list>
#include <map>
#include <set>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace open_lmm::test::benchmark {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

Json ReadJson(const fs::path& path) {
  std::ifstream input(path);
  if (!input) throw std::invalid_argument("failed to open " + path.string());
  Json value;
  input >> value;
  return value;
}

std::string Digest(const fs::path& path) {
  return "sha256:" + replay::Sha256File(path);
}

void RequireBundle(const Json& bundle, const char* scenario,
                   const char* parallelism, const char* measurement_role,
                   const fs::path& path) {
  const auto validation = ValidatePerformanceBundle(bundle);
  if (!validation.Ok()) {
    throw std::invalid_argument("invalid bundle " + path.string() + ":\n" +
                                validation.Summary());
  }
  if (bundle.at("scenario") != scenario ||
      bundle.at("key").at("parallelism") != parallelism ||
      bundle.at("key").at("measurement_role") != measurement_role) {
    throw std::invalid_argument("bundle has the wrong map-update mode: " +
                                path.string());
  }
}

std::map<uint64_t, Json> LoadReports(const Json& bundle) {
  std::map<uint64_t, Json> reports;
  for (const auto& reference : bundle.at("reports")) {
    const fs::path path = reference.at("path").get<std::string>();
    if (Digest(path) != reference.at("sha256")) {
      throw std::invalid_argument("raw report hash changed: " + path.string());
    }
    Json report = ReadJson(path);
    const auto validation = ValidatePerformanceReport(report);
    if (!validation.Ok()) {
      throw std::invalid_argument("invalid raw report " + path.string());
    }
    const uint64_t iteration = report.at("iteration").get<uint64_t>();
    if (!reports.emplace(iteration, std::move(report)).second) {
      throw std::invalid_argument("duplicate pair iteration");
    }
  }
  return reports;
}

Json PairIdentity(Json key) {
  key.erase("scenario");
  key.erase("parallelism");
  key.erase("fixture_manifest_sha256");
  key.erase("config_fingerprint");
  return key;
}

Json OwnerJoinIdentity(Json key) {
  key.erase("measurement_role");
  return key;
}

std::string CombinedComparison(
    const std::initializer_list<const Json*>& bundles) {
  bool all_pass = true;
  bool all_uncalibrated = true;
  for (const Json* bundle : bundles) {
    const std::string comparison = bundle->at("comparison");
    if (comparison == "fail") return "fail";
    if (comparison == "baseline_mismatch") return "baseline_mismatch";
    all_pass &= comparison == "pass";
    all_uncalibrated &= comparison == "uncalibrated";
  }
  if (all_pass) return "pass";
  if (all_uncalibrated) return "uncalibrated";
  return "baseline_mismatch";
}

Json Reference(const fs::path& path) {
  return {{"path", path.generic_string()}, {"sha256", Digest(path)}};
}

Json OptionalUnsigned(const Json& group, const char* field) {
  return group.at(field).is_number_unsigned() ||
                 (group.at(field).is_number_integer() &&
                  group.at(field).get<int64_t>() >= 0)
             ? group.at(field)
             : Json(nullptr);
}

Json Ratio(const Json& numerator, const Json& denominator) {
  if (!numerator.is_number() || !denominator.is_number() ||
      denominator.get<double>() <= 0.0) {
    return nullptr;
  }
  return numerator.get<double>() / denominator.get<double>();
}

bool RawPassed(const Json& report) {
  return report.at("result") != "fail" && report.at("result") != "invalid" &&
         report.at("failures").empty() &&
         report.at("artifacts").at("correctness") == "pass";
}

void Issue(ValidationResult& result, std::string pointer,
           std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

}  // namespace

Json BuildMapUpdatePairReport(
    const fs::path& sequential_bundle_path,
    const fs::path& parallel_bundle_path,
    const fs::path& sequential_owner_bundle_path,
    const fs::path& parallel_owner_bundle_path,
    const std::vector<PairParityEvidence>& parity) {
  const Json sequential_bundle = ReadJson(sequential_bundle_path);
  const Json parallel_bundle = ReadJson(parallel_bundle_path);
  const Json sequential_owner_bundle = ReadJson(sequential_owner_bundle_path);
  const Json parallel_owner_bundle = ReadJson(parallel_owner_bundle_path);
  RequireBundle(sequential_bundle, "map-update-sequential", "sequential",
                "public", sequential_bundle_path);
  RequireBundle(parallel_bundle, "map-update-parallel", "parallel", "public",
                parallel_bundle_path);
  RequireBundle(sequential_owner_bundle, "map-update-sequential",
                "sequential", "owner", sequential_owner_bundle_path);
  RequireBundle(parallel_owner_bundle, "map-update-parallel", "parallel",
                "owner", parallel_owner_bundle_path);
  if (PairIdentity(sequential_bundle.at("key")) !=
          PairIdentity(parallel_bundle.at("key")) ||
      OwnerJoinIdentity(sequential_bundle.at("key")) !=
          OwnerJoinIdentity(sequential_owner_bundle.at("key")) ||
      OwnerJoinIdentity(parallel_bundle.at("key")) !=
          OwnerJoinIdentity(parallel_owner_bundle.at("key")) ||
      sequential_bundle.at("profile") != parallel_bundle.at("profile")) {
    throw std::invalid_argument(
        "paired bundles do not share one exact workload identity");
  }

  const auto sequential = LoadReports(sequential_bundle);
  const auto parallel = LoadReports(parallel_bundle);
  const auto sequential_owner = LoadReports(sequential_owner_bundle);
  const auto parallel_owner = LoadReports(parallel_owner_bundle);
  std::map<uint64_t, PairParityEvidence> parity_by_iteration;
  for (const auto& evidence : parity) {
    if (evidence.iteration == 0 || !fs::is_regular_file(evidence.log_path) ||
        !parity_by_iteration.emplace(evidence.iteration, evidence).second) {
      throw std::invalid_argument("invalid or duplicate parity evidence");
    }
  }
  if (sequential.size() != parallel.size() ||
      sequential.size() != sequential_owner.size() ||
      sequential.size() != parallel_owner.size() ||
      sequential.size() != parity_by_iteration.size()) {
    throw std::invalid_argument("paired evidence iteration sets differ");
  }

  bool passed = true;
  Json pairs = Json::array();
  for (const auto& [iteration, sequential_report] : sequential) {
    if (!parallel.contains(iteration) || !sequential_owner.contains(iteration) ||
        !parallel_owner.contains(iteration) ||
        !parity_by_iteration.contains(iteration)) {
      throw std::invalid_argument("paired evidence is missing an iteration");
    }
    const auto& parallel_report = parallel.at(iteration);
    const auto& sequential_owner_report = sequential_owner.at(iteration);
    const auto& parallel_owner_report = parallel_owner.at(iteration);
    const auto& parity_evidence = parity_by_iteration.at(iteration);
    const bool iteration_passed =
        parity_evidence.passed && RawPassed(sequential_report) &&
        RawPassed(parallel_report) && RawPassed(sequential_owner_report) &&
        RawPassed(parallel_owner_report);
    passed = passed && iteration_passed;

    const Json sequential_wall =
        OptionalUnsigned(sequential_report.at("timing"), "wall_time_ns");
    const Json parallel_wall =
        OptionalUnsigned(parallel_report.at("timing"), "wall_time_ns");
    const Json sequential_cpu =
        OptionalUnsigned(sequential_report.at("timing"), "cpu_time_ns");
    const Json parallel_cpu =
        OptionalUnsigned(parallel_report.at("timing"), "cpu_time_ns");
    pairs.push_back(
        {{"iteration", iteration},
         {"semantic_parity",
          {{"result", parity_evidence.passed ? "pass" : "fail"},
           {"log", Reference(parity_evidence.log_path)}}},
         {"timing",
          {{"sequential_wall_time_ns", sequential_wall},
           {"parallel_wall_time_ns", parallel_wall},
           {"wall_speedup", Ratio(sequential_wall, parallel_wall)},
           {"sequential_cpu_time_ns", sequential_cpu},
           {"parallel_cpu_time_ns", parallel_cpu},
           {"cpu_speedup", Ratio(sequential_cpu, parallel_cpu)}}},
         {"process_memory",
          {{"sequential_sampled_peak_rss_bytes",
            OptionalUnsigned(sequential_report.at("process_memory"),
                             "sampled_peak_rss_bytes")},
           {"parallel_sampled_peak_rss_bytes",
            OptionalUnsigned(parallel_report.at("process_memory"),
                             "sampled_peak_rss_bytes")}}},
         {"owner_memory",
          {{"sequential_peak_total_bytes",
            OptionalUnsigned(sequential_owner_report.at("owner_memory"),
                             "governor_peak_reserved_total_bytes")},
           {"parallel_peak_total_bytes",
            OptionalUnsigned(parallel_owner_report.at("owner_memory"),
                             "governor_peak_reserved_total_bytes")},
           {"sequential_peak_heavy_bytes",
            OptionalUnsigned(sequential_owner_report.at("owner_memory"),
                             "governor_peak_heavy_map_bytes")},
           {"parallel_peak_heavy_bytes",
            OptionalUnsigned(parallel_owner_report.at("owner_memory"),
                             "governor_peak_heavy_map_bytes")},
           {"sequential_max_active_tasks",
            OptionalUnsigned(sequential_owner_report.at("owner_memory"),
                             "executor_max_active_tasks")},
           {"parallel_max_active_tasks",
            OptionalUnsigned(parallel_owner_report.at("owner_memory"),
                             "executor_max_active_tasks")}}},
         {"output",
          {{"sequential_points",
            OptionalUnsigned(sequential_report.at("points"),
                             "map_output_points")},
           {"parallel_points",
            OptionalUnsigned(parallel_report.at("points"),
                             "map_output_points")},
           {"sequential_bytes",
            OptionalUnsigned(sequential_report.at("points"),
                             "map_output_file_bytes")},
           {"parallel_bytes",
            OptionalUnsigned(parallel_report.at("points"),
                             "map_output_file_bytes")},
           {"byte_digest_equal",
            sequential_report.at("artifacts").at("output_digest") ==
                parallel_report.at("artifacts").at("output_digest")}}},
         {"result", iteration_passed ? "pass" : "fail"}});
  }

  const std::string comparison = CombinedComparison(
      {&sequential_bundle, &parallel_bundle, &sequential_owner_bundle,
       &parallel_owner_bundle});
  Json report = {
      {"schema_version", 1},
      {"pair_id", sequential_bundle.at("profile").get<std::string>() + "-" +
                      sequential_bundle.at("key").at("fixture_id")
                          .get<std::string>() +
                      "-map-update-pair"},
      {"profile", sequential_bundle.at("profile")},
      {"fixture_id", sequential_bundle.at("key").at("fixture_id")},
      {"pair_fingerprint",
       sequential_bundle.at("key").at("pair_fingerprint")},
      {"sequential_bundle", Reference(sequential_bundle_path)},
      {"parallel_bundle", Reference(parallel_bundle_path)},
      {"sequential_owner_bundle", Reference(sequential_owner_bundle_path)},
      {"parallel_owner_bundle", Reference(parallel_owner_bundle_path)},
      {"parity_policy",
       {{"pcd_rms_m", 0.04},
        {"pcd_max_m", 0.4},
        {"point_count_ratio", 0.001},
        {"basis", "RMS 10% and max 100% of 0.4 m output voxel"}}},
      {"pairs", std::move(pairs)},
      {"comparison", comparison},
      {"result", passed ? "pass" : "fail"}};
  const auto validation = ValidateMapUpdatePairReport(report);
  if (!validation.Ok()) {
    throw std::logic_error("generated invalid map-update pair report:\n" +
                           validation.Summary());
  }
  return report;
}

ValidationResult ValidateMapUpdatePairReport(const Json& report) {
  ValidationResult result;
  if (!report.is_object()) {
    Issue(result, "", "must be an object");
    return result;
  }
  const std::set<std::string> fields = {
      "schema_version", "pair_id", "profile", "fixture_id",
      "pair_fingerprint", "sequential_bundle", "parallel_bundle",
      "sequential_owner_bundle", "parallel_owner_bundle", "parity_policy", "pairs",
      "comparison", "result"};
  for (const auto& field : fields) {
    if (!report.contains(field)) Issue(result, "/" + field, "required");
  }
  for (auto field = report.begin(); field != report.end(); ++field) {
    if (!fields.contains(field.key()))
      Issue(result, "/" + field.key(), "unknown field");
  }
  if (report.value("schema_version", 0) != 1)
    Issue(result, "/schema_version", "must equal 1");
  for (const char* field : {"pair_id", "profile", "fixture_id",
                            "pair_fingerprint", "comparison", "result"}) {
    if (!report.contains(field) || !report.at(field).is_string() ||
        report.at(field).get<std::string>().empty()) {
      Issue(result, std::string("/") + field, "must be non-empty string");
    }
  }
  if (!report.contains("pairs") || !report.at("pairs").is_array() ||
      report.at("pairs").empty()) {
    Issue(result, "/pairs", "must be a non-empty array");
  }
  if (!report.contains("parity_policy") ||
      !report.at("parity_policy").is_object()) {
    Issue(result, "/parity_policy", "must be an object");
  }
  if (report.contains("comparison") && report.at("comparison").is_string() &&
      !std::set<std::string>{"pass", "fail", "uncalibrated",
                             "baseline_mismatch"}
           .contains(report.at("comparison").get<std::string>())) {
    Issue(result, "/comparison", "unsupported value");
  }
  if (report.contains("result") && report.at("result") != "pass" &&
      report.at("result") != "fail") {
    Issue(result, "/result", "must be pass or fail");
  }
  return result;
}

}  // namespace open_lmm::test::benchmark
