#include "benchmark_bundle.hpp"

#include "benchmark_statistics.hpp"
#include "tools/replay/replay_sha256.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>

#include <nlohmann/json.hpp>

namespace open_lmm::test::benchmark {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

Json ReadJson(const fs::path& path) {
  std::ifstream input(path);
  if (!input) throw std::invalid_argument("failed to open " + path.string());
  Json value;
  try {
    input >> value;
  } catch (const std::exception& error) {
    throw std::invalid_argument("invalid JSON " + path.string() + ": " +
                                error.what());
  }
  return value;
}

std::string Digest(const fs::path& path) {
  return "sha256:" + replay::Sha256File(path);
}

Json BaselineKey(const Json& report) {
  return {{"report_schema", report.at("schema_version")},
          {"measurement_role", report.at("measurement_role")},
          {"fixture_id", report.at("fixture").at("id")},
          {"fixture_manifest_sha256",
           report.at("fixture").at("manifest_sha256")},
          {"scenario", report.at("scenario")},
          {"compiler", report.at("build").at("compiler")},
          {"build_type", report.at("build").at("build_type")},
          {"sanitizer", report.at("build").at("sanitizer")},
          {"cpu_model", report.at("machine").at("cpu_model")},
          {"cpu_count", report.at("machine").at("cpu_count")},
          {"memory_class", report.at("machine").at("memory_class")},
          {"container_digest",
           report.at("build").at("container_digest")},
          {"plugin_ids", report.at("fixture").at("plugin_ids")},
          {"config_fingerprint",
           report.at("fixture").at("config_fingerprint")},
          {"pair_fingerprint",
           report.at("fixture").at("pair_fingerprint")},
          {"parallelism", report.at("fixture").at("parallelism")}};
}

void CollectMetricGroup(const Json& group, const std::string& prefix,
                        std::map<std::string, std::vector<uint64_t>>& samples) {
  for (auto field = group.begin(); field != group.end(); ++field) {
    // This diagnostic is intentionally signed: RSS can shrink during a target
    // window.  Bundle summaries and baseline thresholds are non-negative, so
    // including only its positive samples would make metric availability
    // depend on allocator timing.  Preserve it in every raw report, but do not
    // promote it to an aggregate regression metric.
    if (prefix == "process_memory" &&
        field.key() == "retained_rss_delta_bytes") {
      continue;
    }
    if (field.key() == "unavailable_reasons" || field.value().is_null() ||
        !field.value().is_number_integer() ||
        (field.value().is_number_integer() &&
         !field.value().is_number_unsigned() &&
         field.value().get<int64_t>() < 0)) {
      continue;
    }
    samples[prefix + "." + field.key()].push_back(
        field.value().get<uint64_t>());
  }
}

Json Summary(const IntegerStatistics& value) {
  return {{"sample_count", value.sample_count},
          {"median", value.median},
          {"p95", value.p95 ? Json(*value.p95) : Json(nullptr)},
          {"mad", value.median_absolute_deviation},
          {"min", value.minimum},
          {"max", value.maximum}};
}

double StatisticValue(const Json& summary, const std::string& statistic) {
  if (!summary.contains(statistic) || summary.at(statistic).is_null() ||
      !summary.at(statistic).is_number()) {
    throw std::invalid_argument("baseline selects unavailable statistic " +
                                statistic);
  }
  return summary.at(statistic).get<double>();
}

bool IsSha256(const Json& value) {
  if (!value.is_string()) return false;
  const std::string text = value.get<std::string>();
  if (text.size() != 71 || !text.starts_with("sha256:")) return false;
  return std::all_of(text.begin() + 7, text.end(), [](unsigned char c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
  });
}

bool HasExactFields(const Json& value, const std::set<std::string>& fields) {
  if (!value.is_object() || value.size() != fields.size()) return false;
  return std::all_of(fields.begin(), fields.end(), [&](const auto& field) {
    return value.contains(field);
  });
}

bool IsNonEmptyString(const Json& value) {
  return value.is_string() && !value.get<std::string>().empty();
}

bool IsBaselineKey(const Json& key) {
  const std::set<std::string> key_fields = {
      "report_schema", "measurement_role", "fixture_id",
      "fixture_manifest_sha256", "scenario", "compiler", "build_type",
      "sanitizer", "cpu_model", "cpu_count", "memory_class",
      "container_digest", "plugin_ids", "config_fingerprint",
      "pair_fingerprint", "parallelism"};
  if (!HasExactFields(key, key_fields) || key.value("report_schema", 0) != 1 ||
      !key.at("measurement_role").is_string() ||
      !std::set<std::string>{"public", "owner"}.contains(
          key.at("measurement_role").get<std::string>()) ||
      !key.at("scenario").is_string() ||
      !std::set<std::string>{
           "open", "data-load", "alignment", "map-update-sequential",
           "map-update-parallel", "save-fallback", "visualization-cold",
           "visualization-warm", "full-pipeline", "cancellation"}
           .contains(key.at("scenario").get<std::string>()) ||
      !key.at("parallelism").is_string() ||
      !std::set<std::string>{"map-update-disabled", "sequential", "parallel"}
           .contains(key.at("parallelism").get<std::string>())) {
    return false;
  }
  for (const char* field : {"fixture_id", "compiler", "build_type",
                            "sanitizer", "cpu_model", "memory_class"}) {
    if (!IsNonEmptyString(key.at(field))) return false;
  }
  for (const char* field : {"fixture_manifest_sha256", "container_digest",
                            "config_fingerprint", "pair_fingerprint"}) {
    if (!IsSha256(key.at(field))) return false;
  }
  const auto& cpu_count = key.at("cpu_count");
  const bool positive_cpu_count =
      (cpu_count.is_number_unsigned() && cpu_count.get<uint64_t>() > 0) ||
      (cpu_count.is_number_integer() && cpu_count.get<int64_t>() > 0);
  if (!positive_cpu_count || !key.at("plugin_ids").is_array() ||
      key.at("plugin_ids").empty()) {
    return false;
  }
  std::set<std::string> plugin_ids;
  for (const auto& plugin_id : key.at("plugin_ids")) {
    if (!IsNonEmptyString(plugin_id) ||
        !plugin_ids.insert(plugin_id.get<std::string>()).second) {
      return false;
    }
  }
  return true;
}

bool IsBaselineEntry(const Json& value) {
  const std::set<std::string> entry_fields = {
      "schema_version", "baseline_id", "key", "source_bundle_sha256",
      "metrics"};
  return HasExactFields(value, entry_fields) &&
         value.value("schema_version", 0) == 1 &&
         value.contains("baseline_id") &&
         value.at("baseline_id").is_string() &&
         !value.at("baseline_id").get<std::string>().empty() &&
         value.contains("key") && IsBaselineKey(value.at("key")) &&
         value.contains("source_bundle_sha256") &&
         IsSha256(value.at("source_bundle_sha256")) &&
         value.contains("metrics") && value.at("metrics").is_array();
}

void ValidateBaselineEntry(const Json& value) {
  if (!IsBaselineEntry(value) || value.at("metrics").empty()) {
    throw std::invalid_argument("invalid performance baseline entry");
  }
  std::set<std::pair<std::string, std::string>> selected;
  for (const auto& threshold : value.at("metrics")) {
    const std::set<std::string> metric_fields = {
        "name", "statistic", "expected", "relative_allowance",
        "absolute_allowance"};
    const bool shape =
        HasExactFields(threshold, metric_fields) && threshold.contains("name") &&
        threshold.at("name").is_string() &&
        !threshold.at("name").get<std::string>().empty() &&
        threshold.contains("statistic") &&
        threshold.at("statistic").is_string() &&
        std::set<std::string>{"median", "p95", "mad", "min", "max"}
            .contains(threshold.at("statistic").get<std::string>()) &&
        threshold.contains("expected") &&
        threshold.at("expected").is_number() &&
        threshold.contains("relative_allowance") &&
        threshold.at("relative_allowance").is_number() &&
        threshold.contains("absolute_allowance") &&
        threshold.at("absolute_allowance").is_number();
    if (!shape) {
      throw std::invalid_argument("invalid baseline metric threshold");
    }
    const auto metric_key = std::pair{
        threshold.at("name").get<std::string>(),
        threshold.at("statistic").get<std::string>()};
    if (!selected.insert(metric_key).second) {
      throw std::invalid_argument("duplicate baseline metric threshold");
    }
    for (const char* field : {"expected", "relative_allowance",
                              "absolute_allowance"}) {
      const double number = threshold.at(field).get<double>();
      if (!std::isfinite(number) || number < 0.0) {
        throw std::invalid_argument(
            "baseline threshold values must be finite and non-negative");
      }
    }
  }
}

struct BaselineSelection {
  const Json* entry = nullptr;
  std::string id;
};

BaselineSelection SelectBaseline(const Json& document, const Json& key) {
  if (document.is_object() && document.contains("baseline_id")) {
    ValidateBaselineEntry(document);
    return {document.at("key") == key ? &document : nullptr,
            document.at("baseline_id").get<std::string>()};
  }
  const bool catalog_shape =
      HasExactFields(document,
                     {"schema_version", "catalog_id", "baselines"}) &&
      document.value("schema_version", 0) == 1 &&
      document.contains("catalog_id") &&
      document.at("catalog_id").is_string() &&
      !document.at("catalog_id").get<std::string>().empty() &&
      document.contains("baselines") &&
      document.at("baselines").is_array() &&
      !document.at("baselines").empty();
  if (!catalog_shape) {
    throw std::invalid_argument("invalid performance baseline contract");
  }

  BaselineSelection selection{
      nullptr, document.at("catalog_id").get<std::string>()};
  std::set<std::string> keys;
  for (const auto& entry : document.at("baselines")) {
    ValidateBaselineEntry(entry);
    if (!keys.insert(entry.at("key").dump()).second) {
      throw std::invalid_argument(
          "duplicate exact key in performance baseline catalog");
    }
    if (entry.at("key") == key) {
      selection.entry = &entry;
      selection.id = entry.at("baseline_id").get<std::string>();
    }
  }
  return selection;
}

void Issue(ValidationResult& result, std::string pointer,
           std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

}  // namespace

Json AggregatePerformanceReports(
    const std::vector<fs::path>& report_paths,
    const std::optional<fs::path>& baseline_path) {
  if (report_paths.empty()) {
    throw std::invalid_argument("aggregation requires at least one report");
  }
  std::vector<std::pair<fs::path, Json>> reports;
  reports.reserve(report_paths.size());
  std::set<uint64_t> iterations;
  Json key;
  Json run_identity;
  Json references = Json::array();
  std::map<std::string, std::vector<uint64_t>> samples;
  bool failed_repetition = false;
  for (const auto& path : report_paths) {
    Json report = ReadJson(path);
    const auto validation = ValidatePerformanceReport(report);
    if (!validation.Ok()) {
      throw std::invalid_argument("invalid raw report " + path.string() +
                                  ":\n" + validation.Summary());
    }
    const Json current_key = BaselineKey(report);
    const Json current_identity =
        {{"profile", report.at("profile")}, {"git", report.at("git")}};
    if (reports.empty()) {
      key = current_key;
      run_identity = current_identity;
    }
    if (current_key != key) {
      throw std::invalid_argument(
          "raw reports do not share one exact baseline key");
    }
    if (current_identity != run_identity) {
      throw std::invalid_argument(
          "raw reports do not share one profile and exact git state");
    }
    const uint64_t iteration = report.at("iteration").get<uint64_t>();
    if (!iterations.insert(iteration).second) {
      throw std::invalid_argument("duplicate measured iteration");
    }
    references.push_back({{"path", path.generic_string()},
                          {"sha256", Digest(path)},
                          {"run_id", report.at("run_id")},
                          {"iteration", iteration},
                          {"result", report.at("result")}});
    failed_repetition |= report.at("result") == "fail" ||
                         report.at("result") == "invalid" ||
                         !report.at("failures").empty() ||
                         report.at("artifacts").at("correctness") != "pass";
    for (const char* group : {"timing", "process_memory", "owner_memory",
                              "points", "io", "cache", "artifacts"}) {
      CollectMetricGroup(report.at(group), group, samples);
    }
    reports.emplace_back(path, std::move(report));
  }
  for (const auto& [name, values] : samples) {
    if (values.size() != reports.size()) {
      throw std::invalid_argument("metric availability differs across runs: " +
                                  name);
    }
  }

  Json metrics = Json::array();
  std::map<std::string, Json> summaries;
  for (const auto& [name, values] : samples) {
    Json summary = Summary(SummarizeIntegers(values));
    summaries.emplace(name, summary);
    metrics.push_back({{"name", name}, {"summary", std::move(summary)}});
  }

  Json baseline = nullptr;
  Json comparisons = Json::array();
  std::string comparison = failed_repetition ? "fail" : "uncalibrated";
  if (baseline_path) {
    const Json baseline_document = ReadJson(*baseline_path);
    const auto selection = SelectBaseline(baseline_document, key);
    baseline = {{"path", baseline_path->generic_string()},
                {"sha256", Digest(*baseline_path)},
                {"id", selection.id}};
    if (selection.entry == nullptr) {
      comparison = "baseline_mismatch";
    } else if (!failed_repetition) {
      const Json& expected = *selection.entry;
      comparison = "pass";
      std::set<std::pair<std::string, std::string>> selected;
      for (const auto& threshold : expected.at("metrics")) {
        if (!threshold.is_object() || !threshold.contains("name") ||
            !threshold.contains("statistic") ||
            !threshold.contains("expected") ||
            !threshold.contains("relative_allowance") ||
            !threshold.contains("absolute_allowance") ||
            !threshold.at("name").is_string() ||
            !threshold.at("statistic").is_string() ||
            !threshold.at("expected").is_number() ||
            !threshold.at("relative_allowance").is_number() ||
            !threshold.at("absolute_allowance").is_number()) {
          throw std::invalid_argument("invalid baseline metric threshold");
        }
        const std::string name = threshold.at("name");
        const std::string statistic = threshold.at("statistic");
        if (!selected.emplace(name, statistic).second ||
            !summaries.contains(name)) {
          throw std::invalid_argument(
              "baseline metric is duplicate or unavailable: " + name);
        }
        const double actual = StatisticValue(summaries.at(name), statistic);
        const double expected_value = threshold.at("expected").get<double>();
        const double relative =
            threshold.at("relative_allowance").get<double>();
        const double absolute =
            threshold.at("absolute_allowance").get<double>();
        if (!std::isfinite(expected_value) || expected_value < 0.0 ||
            !std::isfinite(relative) || relative < 0.0 ||
            !std::isfinite(absolute) || absolute < 0.0) {
          throw std::invalid_argument(
              "baseline threshold values must be finite and non-negative");
        }
        const double allowance = std::max(expected_value * relative, absolute);
        const double delta = actual - expected_value;
        const bool passed = delta <= allowance;
        if (!passed) comparison = "fail";
        comparisons.push_back({{"name", name},
                               {"statistic", statistic},
                               {"expected", expected_value},
                               {"actual", actual},
                               {"allowance", allowance},
                               {"delta", delta},
                               {"result", passed ? "pass" : "fail"}});
      }
      if (expected.at("metrics").empty()) {
        throw std::invalid_argument("baseline must select at least one metric");
      }
    }
  }

  Json bundle = {
      {"schema_version", 1},
      {"bundle_id", reports.front().second.at("profile").get<std::string>() +
                        "-" + key.at("fixture_id").get<std::string>() + "-" +
                        key.at("scenario").get<std::string>() + "-" +
                        key.at("measurement_role").get<std::string>()},
      {"profile", reports.front().second.at("profile")},
      {"scenario", key.at("scenario")},
      {"key", std::move(key)},
      {"reports", std::move(references)},
      {"metrics", std::move(metrics)},
      {"baseline", std::move(baseline)},
      {"comparisons", std::move(comparisons)},
      {"comparison", comparison},
      {"failures", Json::array()}};
  if (failed_repetition) {
    bundle["failures"].push_back(
        {{"phase", "raw-repetition"},
         {"message", "one or more measured repetitions failed"}});
  }
  const auto validation = ValidatePerformanceBundle(bundle);
  if (!validation.Ok()) {
    throw std::logic_error("generated invalid bundle:\n" +
                           validation.Summary());
  }
  return bundle;
}

ValidationResult ValidatePerformanceBundle(const Json& bundle) {
  ValidationResult result;
  if (!bundle.is_object()) {
    Issue(result, "", "must be an object");
    return result;
  }
  const std::set<std::string> fields = {
      "schema_version", "bundle_id", "profile", "scenario", "key",
      "reports", "metrics", "baseline", "comparisons", "comparison",
      "failures"};
  for (const auto& field : fields) {
    if (!bundle.contains(field)) Issue(result, "/" + field, "required");
  }
  for (auto field = bundle.begin(); field != bundle.end(); ++field) {
    if (!fields.contains(field.key()))
      Issue(result, "/" + field.key(), "unknown field");
  }
  if (bundle.value("schema_version", 0) != 1)
    Issue(result, "/schema_version", "must equal 1");
  for (const char* field : {"bundle_id", "profile", "scenario",
                            "comparison"}) {
    if (!bundle.contains(field) || !bundle.at(field).is_string() ||
        bundle.at(field).get<std::string>().empty())
      Issue(result, std::string("/") + field, "must be non-empty string");
  }
  if (bundle.contains("comparison") && bundle.at("comparison").is_string() &&
      !std::set<std::string>{"pass", "fail", "uncalibrated",
                             "baseline_mismatch"}
           .contains(bundle.at("comparison").get<std::string>())) {
    Issue(result, "/comparison", "unsupported value");
  }
  if (!bundle.contains("key") || !bundle.at("key").is_object())
    Issue(result, "/key", "must be an object");
  if (!bundle.contains("reports") || !bundle.at("reports").is_array() ||
      bundle.at("reports").empty())
    Issue(result, "/reports", "must be a non-empty array");
  if (!bundle.contains("metrics") || !bundle.at("metrics").is_array())
    Issue(result, "/metrics", "must be an array");
  if (!bundle.contains("comparisons") ||
      !bundle.at("comparisons").is_array())
    Issue(result, "/comparisons", "must be an array");
  if (!bundle.contains("failures") || !bundle.at("failures").is_array())
    Issue(result, "/failures", "must be an array");
  return result;
}

}  // namespace open_lmm::test::benchmark
