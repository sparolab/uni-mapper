#include "soak_metrics.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cctype>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <system_error>
#include <utility>

#include <nlohmann/json.hpp>
#include <fcntl.h>
#include <sys/resource.h>
#include <unistd.h>

namespace open_lmm::test::soak {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

std::optional<uint64_t> StatusBytes(std::string_view key) {
  std::ifstream input("/proc/self/status");
  for (std::string line; std::getline(input, line);) {
    if (!line.starts_with(key)) continue;
    std::istringstream fields(line.substr(key.size()));
    uint64_t kibibytes = 0;
    std::string unit;
    if (!(fields >> kibibytes >> unit) || unit != "kB" ||
        kibibytes > std::numeric_limits<uint64_t>::max() / 1024) {
      return std::nullopt;
    }
    return kibibytes * 1024;
  }
  return std::nullopt;
}

std::optional<uint64_t> DirectoryEntryCount(const fs::path& path,
                                            bool subtract_sampler_fd) {
  std::error_code error;
  uint64_t count = 0;
  for (fs::directory_iterator entry(path, error), end;
       !error && entry != end; entry.increment(error)) {
    ++count;
  }
  if (error || (subtract_sampler_fd && count == 0)) return std::nullopt;
  return subtract_sampler_fd ? count - 1 : count;
}

std::optional<uint64_t> CpuTimeNanoseconds() {
  rusage usage{};
  if (getrusage(RUSAGE_SELF, &usage) != 0) return std::nullopt;
  constexpr uint64_t kNanosecondsPerSecond = 1'000'000'000;
  constexpr uint64_t kNanosecondsPerMicrosecond = 1'000;
  const auto timeval_ns = [](const timeval& value) -> uint64_t {
    return static_cast<uint64_t>(value.tv_sec) * kNanosecondsPerSecond +
           static_cast<uint64_t>(value.tv_usec) *
               kNanosecondsPerMicrosecond;
  };
  return timeval_ns(usage.ru_utime) + timeval_ns(usage.ru_stime);
}

double Median(std::vector<double> values) {
  if (values.empty()) throw std::invalid_argument("median requires samples");
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  return values.size() % 2 == 0
             ? (values[middle - 1] + values[middle]) / 2.0
             : values[middle];
}

void Add(ValidationResult& result, std::string pointer,
         std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

bool IsNonEmptyString(const Json& value) {
  return value.is_string() && !value.get_ref<const std::string&>().empty();
}

bool IsNonNegativeInteger(const Json& value) {
  return value.is_number_unsigned() ||
         (value.is_number_integer() && value.get<int64_t>() >= 0);
}

void RequireKeys(const Json& object, const std::vector<std::string>& keys,
                 std::string_view pointer, ValidationResult& result) {
  for (const auto& key : keys) {
    if (!object.contains(key)) {
      Add(result, std::string(pointer) + "/" + key, "required field is missing");
    }
  }
}

void RejectUnknown(const Json& object, const std::vector<std::string>& allowed,
                   std::string_view pointer, ValidationResult& result) {
  for (auto field = object.begin(); field != object.end(); ++field) {
    if (std::find(allowed.begin(), allowed.end(), field.key()) ==
        allowed.end()) {
      Add(result, std::string(pointer) + "/" + field.key(),
          "unknown field");
    }
  }
}

bool IsNullableNonNegativeInteger(const Json& value) {
  return value.is_null() || IsNonNegativeInteger(value);
}

bool IsFiniteNumber(const Json& value) {
  return value.is_number() && std::isfinite(value.get<double>());
}

bool IsNullablePositiveInteger(const Json& value) {
  return value.is_null() ||
         (IsNonNegativeInteger(value) && value.get<uint64_t>() > 0);
}

bool IsSha256Digest(const Json& value) {
  if (value.is_null()) return true;
  if (!value.is_string()) return false;
  const auto& digest = value.get_ref<const std::string&>();
  return digest.size() == 71 && digest.starts_with("sha256:") &&
         std::all_of(digest.begin() + 7, digest.end(),
                     [](unsigned char character) {
                       return std::isdigit(character) != 0 ||
                              (character >= 'a' && character <= 'f');
                     });
}

void ValidateClosedObject(const Json& value,
                          const std::vector<std::string>& fields,
                          std::string_view pointer,
                          ValidationResult& result) {
  if (!value.is_object()) {
    Add(result, std::string(pointer), "must be an object");
    return;
  }
  RequireKeys(value, fields, pointer, result);
  RejectUnknown(value, fields, pointer, result);
}

void ValidateMetadata(const Json& report, ValidationResult& result) {
  if (report.contains("git")) {
    const auto& value = report.at("git");
    ValidateClosedObject(value, {"commit", "dirty"}, "/git", result);
    if (value.is_object()) {
      if (!value.contains("commit") || !value.at("commit").is_string() ||
          value.at("commit").get_ref<const std::string&>().size() != 40 ||
          !std::all_of(value.at("commit").get_ref<const std::string&>().begin(),
                       value.at("commit").get_ref<const std::string&>().end(),
                       [](unsigned char character) {
                         return std::isxdigit(character) != 0;
                       })) {
        Add(result, "/git/commit", "must contain 40 hexadecimal digits");
      }
      if (!value.contains("dirty") || !value.at("dirty").is_boolean())
        Add(result, "/git/dirty", "must be a boolean");
    }
  }
  for (const char* key : {"build", "machine"}) {
    if (!report.contains(key)) continue;
    const auto fields = std::string_view(key) == "build"
                            ? std::vector<std::string>{"compiler", "sanitizer",
                                                       "build_type"}
                            : std::vector<std::string>{"os", "kernel", "cpu_count",
                                                       "memory_bytes",
                                                       "container_digest"};
    ValidateClosedObject(report.at(key), fields, std::string("/") + key,
                         result);
    if (!report.at(key).is_object()) continue;
    if (std::string_view(key) == "build") {
      for (const auto& field : fields) {
        if (report.at(key).contains(field) &&
            !IsNonEmptyString(report.at(key).at(field))) {
          Add(result, "/build/" + field, "must be a non-empty string");
        }
      }
    } else {
      for (const char* field : {"os", "kernel"}) {
        if (report.at(key).contains(field) &&
            !IsNonEmptyString(report.at(key).at(field))) {
          Add(result, std::string("/machine/") + field,
              "must be a non-empty string");
        }
      }
      for (const char* field : {"cpu_count", "memory_bytes"}) {
        if (report.at(key).contains(field) &&
            !IsNullablePositiveInteger(report.at(key).at(field))) {
          Add(result, std::string("/machine/") + field,
              "must be null or a positive integer");
        }
      }
      if (report.at(key).contains("container_digest") &&
          !IsSha256Digest(report.at(key).at("container_digest"))) {
        Add(result, "/machine/container_digest",
            "must be null or a lowercase sha256 digest");
      }
    }
  }
}

void ValidateSamples(const Json& report, ValidationResult& result) {
  if (!report.contains("samples") || !report.at("samples").is_array()) return;
  const std::vector<std::string> required_sample_fields = {
      "iteration", "checkpoint", "process", "runtime_revision",
      "recent_event_count", "reserved_memory_bytes", "executor"};
  auto allowed_sample_fields = required_sample_fields;
  allowed_sample_fields.push_back("owner");
  const std::vector<std::string> process_fields = {
      "rss_bytes", "peak_rss_bytes", "thread_count", "fd_count",
      "cpu_time_ns", "unavailable"};
  const std::vector<std::string> executor_fields = {
      "worker_count", "queue_capacity", "queued_tasks", "active_tasks",
      "waiting_submitters", "completed_tasks", "cancelled_queued_tasks"};
  const std::vector<std::string> owner_fields = {
      "live_jobs", "live_subscriptions", "callbacks",
      "visualization_cache_entries", "visualization_cache_bytes",
      "output_final_files", "output_temporary_files", "output_backup_entries",
      "output_recovery_manifests", "output_directory_count",
      "loaded_plugin_mappings"};
  for (std::size_t index = 0; index < report.at("samples").size(); ++index) {
    const auto pointer = "/samples/" + std::to_string(index);
    const auto& sample = report.at("samples").at(index);
    if (!sample.is_object()) {
      Add(result, pointer, "must be an object");
      continue;
    }
    RequireKeys(sample, required_sample_fields, pointer, result);
    RejectUnknown(sample, allowed_sample_fields, pointer, result);
    if (sample.contains("process"))
      ValidateClosedObject(sample.at("process"), process_fields,
                           pointer + "/process", result);
    if (sample.contains("process") && sample.at("process").is_object()) {
      for (const char* key : {"rss_bytes", "peak_rss_bytes", "thread_count",
                              "fd_count", "cpu_time_ns"}) {
        if (sample.at("process").contains(key) &&
            !IsNullableNonNegativeInteger(sample.at("process").at(key))) {
          Add(result, pointer + "/process/" + key,
              "must be null or a non-negative integer");
        }
      }
      if (sample.at("process").contains("unavailable")) {
        const auto& unavailable = sample.at("process").at("unavailable");
        if (!unavailable.is_array() ||
            !std::all_of(unavailable.begin(), unavailable.end(),
                         [](const Json& item) { return item.is_string(); })) {
          Add(result, pointer + "/process/unavailable",
              "must be an array of strings");
        } else {
          std::vector<std::string> names = unavailable.get<std::vector<std::string>>();
          std::sort(names.begin(), names.end());
          if (std::adjacent_find(names.begin(), names.end()) != names.end()) {
            Add(result, pointer + "/process/unavailable",
                "must contain unique values");
          }
        }
      }
    }
    if (sample.contains("executor") && !sample.at("executor").is_null())
      ValidateClosedObject(sample.at("executor"), executor_fields,
                           pointer + "/executor", result);
    if (sample.contains("executor") && sample.at("executor").is_object()) {
      for (const auto& key : executor_fields) {
        if (sample.at("executor").contains(key) &&
            !IsNonNegativeInteger(sample.at("executor").at(key))) {
          Add(result, pointer + "/executor/" + key,
              "must be a non-negative integer");
        }
      }
    }
    if (sample.contains("owner")) {
      ValidateClosedObject(sample.at("owner"), owner_fields,
                           pointer + "/owner", result);
      if (sample.at("owner").is_object()) {
        for (const auto& key : owner_fields) {
          if (sample.at("owner").contains(key) &&
              !IsNullableNonNegativeInteger(sample.at("owner").at(key))) {
            Add(result, pointer + "/owner/" + key,
                "must be null or a non-negative integer");
          }
        }
      }
    }
    if (sample.contains("iteration") &&
        !IsNonNegativeInteger(sample.at("iteration")))
      Add(result, pointer + "/iteration", "must be a non-negative integer");
    if (sample.contains("checkpoint") &&
        !IsNonEmptyString(sample.at("checkpoint")))
      Add(result, pointer + "/checkpoint", "must be a non-empty string");
    for (const char* key : {"runtime_revision", "recent_event_count",
                            "reserved_memory_bytes"}) {
      if (sample.contains(key) &&
          !IsNullableNonNegativeInteger(sample.at(key)))
        Add(result, pointer + "/" + key,
            "must be null or a non-negative integer");
    }
  }
}

void ValidateAnalyses(const Json& report, ValidationResult& result) {
  const std::vector<std::string> slope_fields = {
      "sample_count", "warmup_samples", "theil_sen_per_iteration",
      "previous_quarter_median", "last_quarter_median",
      "quarter_median_delta", "minimum", "maximum"};
  const std::vector<std::string> latency_fields = {
      "samples", "p50_ms", "p95_ms", "max_ms"};
  for (const auto& [key, fields] :
       std::array<std::pair<const char*, const std::vector<std::string>*>, 2>{
           std::pair{"slopes", &slope_fields},
           std::pair{"latencies", &latency_fields}}) {
    if (!report.contains(key) || !report.at(key).is_object()) continue;
    for (auto item = report.at(key).begin(); item != report.at(key).end();
         ++item) {
      ValidateClosedObject(item.value(), *fields,
                           std::string("/") + key + "/" + item.key(), result);
      if (!item.value().is_object()) continue;
      if (std::string_view(key) == "slopes") {
        for (const char* field : {"sample_count", "warmup_samples"}) {
          if (item.value().contains(field) &&
              !IsNonNegativeInteger(item.value().at(field))) {
            Add(result, std::string("/") + key + "/" + item.key() + "/" +
                            field,
                "must be a non-negative integer");
          }
        }
        if (item.value().contains("sample_count") &&
            IsNonNegativeInteger(item.value().at("sample_count")) &&
            item.value().at("sample_count").get<uint64_t>() < 4) {
          Add(result, "/slopes/" + item.key() + "/sample_count",
              "must be at least 4");
        }
        for (const char* field : {"theil_sen_per_iteration",
                                  "previous_quarter_median",
                                  "last_quarter_median", "quarter_median_delta",
                                  "minimum", "maximum"}) {
          if (item.value().contains(field) &&
              !IsFiniteNumber(item.value().at(field))) {
            Add(result, "/slopes/" + item.key() + "/" + field,
                "must be a finite number");
          }
        }
      } else {
        if (item.value().contains("samples") &&
            (!IsNonNegativeInteger(item.value().at("samples")) ||
             item.value().at("samples").get<uint64_t>() == 0)) {
          Add(result, "/latencies/" + item.key() + "/samples",
              "must be a positive integer");
        }
        for (const char* field : {"p50_ms", "p95_ms", "max_ms"}) {
          if (item.value().contains(field) &&
              (!IsFiniteNumber(item.value().at(field)) ||
               item.value().at(field).get<double>() < 0.0)) {
            Add(result, "/latencies/" + item.key() + "/" + field,
                "must be a non-negative finite number");
          }
        }
      }
    }
  }
  if (!report.contains("failures") || !report.at("failures").is_array()) return;
  const std::vector<std::string> failure_fields = {"iteration", "phase",
                                                   "message"};
  for (std::size_t index = 0; index < report.at("failures").size(); ++index) {
    const auto pointer = "/failures/" + std::to_string(index);
    const auto& failure = report.at("failures").at(index);
    ValidateClosedObject(failure, failure_fields, pointer, result);
    if (!failure.is_object()) continue;
    if (failure.contains("iteration") &&
        !IsNullableNonNegativeInteger(failure.at("iteration"))) {
      Add(result, pointer + "/iteration",
          "must be null or a non-negative integer");
    }
    for (const char* field : {"phase", "message"}) {
      if (failure.contains(field) && !IsNonEmptyString(failure.at(field))) {
        Add(result, pointer + "/" + field, "must be a non-empty string");
      }
    }
  }
}

}  // namespace

ProcessMetrics SampleProcessMetrics() {
  ProcessMetrics metrics;
#if defined(__linux__)
  metrics.rss_bytes = StatusBytes("VmRSS:");
  metrics.peak_rss_bytes = StatusBytes("VmHWM:");
  metrics.thread_count = DirectoryEntryCount("/proc/self/task", false);
  metrics.fd_count = DirectoryEntryCount("/proc/self/fd", true);
  metrics.cpu_time_ns = CpuTimeNanoseconds();
  if (!metrics.rss_bytes) metrics.unavailable.push_back("rss_bytes");
  if (!metrics.peak_rss_bytes)
    metrics.unavailable.push_back("peak_rss_bytes");
  if (!metrics.thread_count) metrics.unavailable.push_back("thread_count");
  if (!metrics.fd_count) metrics.unavailable.push_back("fd_count");
  if (!metrics.cpu_time_ns) metrics.unavailable.push_back("cpu_time_ns");
#else
  metrics.unavailable = {"rss_bytes", "peak_rss_bytes", "thread_count",
                         "fd_count", "cpu_time_ns"};
#endif
  return metrics;
}

SlopeAnalysis AnalyzeSlope(const std::vector<MetricPoint>& points,
                           std::size_t warmup_samples) {
  if (warmup_samples >= points.size() ||
      points.size() - warmup_samples < 4 || points.size() > 5'000) {
    throw std::invalid_argument(
        "slope analysis requires 4..5000 samples after validation");
  }
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (!std::isfinite(points[index].value) ||
        (index != 0 &&
         points[index - 1].iteration >= points[index].iteration)) {
      throw std::invalid_argument(
          "metric iterations must increase and values must be finite");
    }
  }

  std::vector<double> slopes;
  const std::size_t retained = points.size() - warmup_samples;
  constexpr std::size_t kMaximumRetainedSlopes = 1'000'000;
  const std::size_t pair_count = retained * (retained - 1) / 2;
  const std::size_t pair_stride =
      std::max<std::size_t>(1, (pair_count + kMaximumRetainedSlopes - 1) /
                                   kMaximumRetainedSlopes);
  slopes.reserve(std::min(pair_count, kMaximumRetainedSlopes));
  std::vector<double> values;
  values.reserve(retained);
  std::size_t pair_index = 0;
  for (std::size_t left = warmup_samples; left < points.size(); ++left) {
    values.push_back(points[left].value);
    for (std::size_t right = left + 1; right < points.size(); ++right) {
      if (pair_index++ % pair_stride == 0) {
        slopes.push_back(
            (points[right].value - points[left].value) /
            static_cast<double>(points[right].iteration -
                                points[left].iteration));
      }
    }
  }

  const std::size_t quarter = std::max<std::size_t>(1, retained / 4);
  const std::size_t last_begin = values.size() - quarter;
  const std::size_t previous_begin = last_begin - quarter;
  const double previous = Median(std::vector<double>(
      values.begin() + static_cast<std::ptrdiff_t>(previous_begin),
      values.begin() + static_cast<std::ptrdiff_t>(last_begin)));
  const double last = Median(std::vector<double>(
      values.begin() + static_cast<std::ptrdiff_t>(last_begin), values.end()));
  const auto [minimum, maximum] =
      std::minmax_element(values.begin(), values.end());
  return {retained,
          warmup_samples,
          Median(std::move(slopes)),
          previous,
          last,
          last - previous,
          *minimum,
          *maximum};
}

Json ProcessMetricsJson(const ProcessMetrics& metrics) {
  auto optional = [](const std::optional<uint64_t>& value) {
    return value ? Json(*value) : Json(nullptr);
  };
  return {{"rss_bytes", optional(metrics.rss_bytes)},
          {"peak_rss_bytes", optional(metrics.peak_rss_bytes)},
          {"thread_count", optional(metrics.thread_count)},
          {"fd_count", optional(metrics.fd_count)},
          {"cpu_time_ns", optional(metrics.cpu_time_ns)},
          {"unavailable", metrics.unavailable}};
}

Json SlopeAnalysisJson(const SlopeAnalysis& analysis) {
  return {{"sample_count", analysis.sample_count},
          {"warmup_samples", analysis.warmup_samples},
          {"theil_sen_per_iteration", analysis.theil_sen_per_iteration},
          {"previous_quarter_median", analysis.previous_quarter_median},
          {"last_quarter_median", analysis.last_quarter_median},
          {"quarter_median_delta", analysis.quarter_median_delta},
          {"minimum", analysis.minimum},
          {"maximum", analysis.maximum}};
}

std::string ValidationResult::Summary() const {
  std::string summary;
  for (const auto& issue : issues) {
    summary += issue.pointer + ": " + issue.message + "\n";
  }
  return summary;
}

ValidationResult ValidateSoakReport(const Json& report) {
  ValidationResult result;
  if (!report.is_object()) {
    Add(result, "", "report must be an object");
    return result;
  }
  const std::vector<std::string> top_level = {
      "schema_version", "run_id", "profile", "scenario", "iterations",
      "warmup_iterations", "seed", "git", "build", "machine", "samples",
      "slopes", "latencies", "failures", "result"};
  RequireKeys(report, top_level, "", result);
  RejectUnknown(report, top_level, "", result);
  if (!report.contains("schema_version") ||
      !IsNonNegativeInteger(report.at("schema_version")) ||
      report.at("schema_version").get<uint64_t>() != 1) {
    Add(result, "/schema_version", "must equal 1");
  }
  for (const char* key : {"run_id", "scenario"}) {
    if (report.contains(key) && !IsNonEmptyString(report.at(key))) {
      Add(result, std::string("/") + key, "must be a non-empty string");
    }
  }
  if (report.contains("profile") &&
      report.at("profile").is_string()) {
    constexpr std::array<std::string_view, 6> profiles = {
        "contract", "fast", "sanitizer", "nightly", "external", "gpu"};
    if (std::find(profiles.begin(), profiles.end(),
                  report.at("profile").get_ref<const std::string&>()) ==
        profiles.end()) {
      Add(result, "/profile", "unsupported profile");
    }
  } else if (report.contains("profile")) {
    Add(result, "/profile", "unsupported profile");
  }
  if (report.contains("result") &&
      (!report.at("result").is_string() ||
       (report.at("result") != "pass" && report.at("result") != "fail" &&
        report.at("result") != "not_available"))) {
    Add(result, "/result", "unsupported result");
  }
  if (report.contains("iterations") &&
      (!IsNonNegativeInteger(report.at("iterations")) ||
       report.at("iterations").get<uint64_t>() == 0)) {
    Add(result, "/iterations", "must be a positive integer");
  }
  for (const char* key : {"warmup_iterations", "seed"}) {
    if (report.contains(key) && !IsNonNegativeInteger(report.at(key))) {
      Add(result, std::string("/") + key, "must be a non-negative integer");
    }
  }
  for (const char* key : {"git", "build", "machine", "slopes",
                          "latencies"}) {
    if (report.contains(key) && !report.at(key).is_object()) {
      Add(result, std::string("/") + key, "must be an object");
    }
  }
  for (const char* key : {"samples", "failures"}) {
    if (report.contains(key) && !report.at(key).is_array()) {
      Add(result, std::string("/") + key, "must be an array");
    }
  }
  if (report.contains("warmup_iterations") && report.contains("iterations") &&
      IsNonNegativeInteger(report.at("warmup_iterations")) &&
      IsNonNegativeInteger(report.at("iterations")) &&
      report.at("warmup_iterations").get<uint64_t>() >=
          report.at("iterations").get<uint64_t>()) {
    Add(result, "/warmup_iterations", "must be smaller than iterations");
  }
  ValidateMetadata(report, result);
  ValidateSamples(report, result);
  ValidateAnalyses(report, result);
  return result;
}

void WriteJsonExclusive(const fs::path& path, const Json& value) {
  if (path.empty() || fs::exists(path)) {
    throw std::runtime_error("soak report path must be new");
  }
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  const int descriptor =
      ::open(path.c_str(), O_CREAT | O_EXCL | O_WRONLY | O_CLOEXEC, 0644);
  if (descriptor < 0) throw std::runtime_error("failed to create soak report");
  const std::string encoded = value.dump(2) + "\n";
  std::size_t offset = 0;
  while (offset < encoded.size()) {
    const ssize_t written =
        ::write(descriptor, encoded.data() + offset, encoded.size() - offset);
    if (written <= 0) {
      const int saved_errno = errno;
      ::close(descriptor);
      std::error_code ignored;
      fs::remove(path, ignored);
      throw std::system_error(saved_errno, std::generic_category(),
                              "failed to write soak report");
    }
    offset += static_cast<std::size_t>(written);
  }
  if (::close(descriptor) != 0) {
    const int saved_errno = errno;
    std::error_code ignored;
    fs::remove(path, ignored);
    throw std::system_error(saved_errno, std::generic_category(),
                            "failed to close soak report");
  }
}

}  // namespace open_lmm::test::soak
