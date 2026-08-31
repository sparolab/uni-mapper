#include "benchmark_report.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <fcntl.h>
#include <nlohmann/json.hpp>
#include <sched.h>
#include <sys/utsname.h>
#include <unistd.h>

namespace open_lmm::test::benchmark {
namespace {

using Json = nlohmann::json;

void Add(ValidationResult& result, std::string pointer,
         std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

bool NonEmptyString(const Json& value) {
  return value.is_string() && !value.get_ref<const std::string&>().empty();
}

bool NonNegativeInteger(const Json& value) {
  return value.is_number_unsigned() ||
         (value.is_number_integer() && value.get<int64_t>() >= 0);
}

bool FiniteNonNegative(const Json& value) {
  return value.is_number() && std::isfinite(value.get<double>()) &&
         value.get<double>() >= 0.0;
}

bool LowerHex(const Json& value, std::size_t size) {
  if (!value.is_string()) return false;
  const auto& text = value.get_ref<const std::string&>();
  return text.size() == size &&
         std::all_of(text.begin(), text.end(), [](unsigned char character) {
           return (character >= '0' && character <= '9') ||
                  (character >= 'a' && character <= 'f');
         });
}

bool Sha256(const Json& value) {
  return value.is_string() &&
         value.get_ref<const std::string&>().starts_with("sha256:") &&
         LowerHex(Json(value.get_ref<const std::string&>().substr(7)), 64);
}

void ClosedObject(const Json& value, const std::vector<std::string>& fields,
                  std::string_view pointer, ValidationResult& result) {
  if (!value.is_object()) {
    Add(result, std::string(pointer), "must be an object");
    return;
  }
  for (const auto& field : fields) {
    if (!value.contains(field)) {
      Add(result, std::string(pointer) + "/" + field,
          "required field is missing");
    }
  }
  for (auto field = value.begin(); field != value.end(); ++field) {
    if (std::find(fields.begin(), fields.end(), field.key()) == fields.end()) {
      Add(result, std::string(pointer) + "/" + field.key(), "unknown field");
    }
  }
}

void StringEnum(const Json& object, const char* field,
                const std::set<std::string>& allowed,
                std::string_view pointer, ValidationResult& result) {
  if (!object.contains(field) || !object.at(field).is_string() ||
      !allowed.contains(object.at(field).get<std::string>())) {
    Add(result, std::string(pointer) + "/" + field,
        "must be a supported value");
  }
}

void ValidateMetricGroup(
    const Json& value, const std::vector<std::string>& unsigned_fields,
    const std::vector<std::string>& signed_fields,
    const std::vector<std::string>& ratio_fields, std::string_view pointer,
    ValidationResult& result,
    const std::vector<std::string>& extra_fields = {}) {
  std::vector<std::string> fields = unsigned_fields;
  fields.insert(fields.end(), signed_fields.begin(), signed_fields.end());
  fields.insert(fields.end(), ratio_fields.begin(), ratio_fields.end());
  fields.insert(fields.end(), extra_fields.begin(), extra_fields.end());
  fields.emplace_back("unavailable_reasons");
  ClosedObject(value, fields, pointer, result);
  if (!value.is_object()) return;
  const Json* reasons = nullptr;
  if (value.contains("unavailable_reasons") &&
      value.at("unavailable_reasons").is_object()) {
    reasons = &value.at("unavailable_reasons");
  } else {
    Add(result, std::string(pointer) + "/unavailable_reasons",
        "must be an object");
  }

  const auto validate_nullability = [&](const std::string& field,
                                        bool valid) {
    if (!value.contains(field)) return;
    const bool is_null = value.at(field).is_null();
    const bool has_reason = reasons && reasons->contains(field);
    if (!is_null && !valid) {
      Add(result, std::string(pointer) + "/" + field,
          "must be null or a valid non-negative value");
    }
    if (is_null != has_reason) {
      Add(result, std::string(pointer) + "/unavailable_reasons/" + field,
          is_null ? "null metric requires a reason"
                  : "available metric must not have a reason");
    }
    if (has_reason && !NonEmptyString(reasons->at(field))) {
      Add(result, std::string(pointer) + "/unavailable_reasons/" + field,
          "reason must be a non-empty string");
    }
  };
  for (const auto& field : unsigned_fields) {
    validate_nullability(
        field, value.contains(field) && NonNegativeInteger(value.at(field)));
  }
  for (const auto& field : signed_fields) {
    validate_nullability(
        field, value.contains(field) && value.at(field).is_number_integer());
  }
  for (const auto& field : ratio_fields) {
    validate_nullability(
        field, value.contains(field) && FiniteNonNegative(value.at(field)));
  }
  if (reasons) {
    for (auto reason = reasons->begin(); reason != reasons->end(); ++reason) {
      if (std::find(fields.begin(), fields.end(), reason.key()) == fields.end() ||
          reason.key() == "unavailable_reasons") {
        Add(result, std::string(pointer) + "/unavailable_reasons/" +
                        reason.key(),
            "reason names an unknown metric");
      }
    }
  }
}

void ValidateMetadata(const Json& report, ValidationResult& result) {
  if (report.contains("git")) {
    const auto& git = report.at("git");
    ClosedObject(git, {"commit", "dirty"}, "/git", result);
    if (git.is_object()) {
      if (!git.contains("commit") || !LowerHex(git.at("commit"), 40)) {
        Add(result, "/git/commit", "must be 40 lowercase hexadecimal digits");
      }
      if (!git.contains("dirty") || !git.at("dirty").is_boolean()) {
        Add(result, "/git/dirty", "must be a boolean");
      }
    }
  }
  if (report.contains("build")) {
    const auto& build = report.at("build");
    ClosedObject(build,
                 {"compiler", "build_type", "sanitizer",
                  "container_digest"},
                 "/build", result);
    if (build.is_object()) {
      for (const char* field : {"compiler", "build_type", "sanitizer"}) {
        if (!build.contains(field) || !NonEmptyString(build.at(field))) {
          Add(result, std::string("/build/") + field,
              "must be a non-empty string");
        }
      }
      if (!build.contains("container_digest") ||
          !Sha256(build.at("container_digest"))) {
        Add(result, "/build/container_digest",
            "must be a lowercase sha256 digest");
      }
    }
  }
  if (report.contains("machine")) {
    const auto& machine = report.at("machine");
    ClosedObject(machine,
                 {"os", "kernel", "cpu_model", "cpu_count",
                  "cpu_affinity", "memory_bytes", "memory_class"},
                 "/machine", result);
    if (machine.is_object()) {
      for (const char* field : {"os", "kernel", "cpu_model",
                                "cpu_affinity", "memory_class"}) {
        if (!machine.contains(field) || !NonEmptyString(machine.at(field))) {
          Add(result, std::string("/machine/") + field,
              "must be a non-empty string");
        }
      }
      for (const char* field : {"cpu_count", "memory_bytes"}) {
        if (!machine.contains(field) ||
            !NonNegativeInteger(machine.at(field)) ||
            machine.at(field).get<uint64_t>() == 0) {
          Add(result, std::string("/machine/") + field,
              "must be a positive integer");
        }
      }
    }
  }
  if (report.contains("fixture")) {
    const auto& fixture = report.at("fixture");
    ClosedObject(fixture,
                 {"id", "manifest_sha256", "decoded_point_count",
                  "decoded_point_bytes", "config_fingerprint",
                  "pair_fingerprint", "plugin_ids", "parallelism"},
                 "/fixture", result);
    if (fixture.is_object()) {
      if (!fixture.contains("id") || !NonEmptyString(fixture.at("id"))) {
        Add(result, "/fixture/id", "must be a non-empty string");
      }
      if (!fixture.contains("manifest_sha256") ||
          !Sha256(fixture.at("manifest_sha256"))) {
        Add(result, "/fixture/manifest_sha256",
            "must be a lowercase sha256 digest");
      }
      if (!fixture.contains("config_fingerprint") ||
          !Sha256(fixture.at("config_fingerprint"))) {
        Add(result, "/fixture/config_fingerprint",
            "must be a lowercase sha256 digest");
      }
      if (!fixture.contains("pair_fingerprint") ||
          !Sha256(fixture.at("pair_fingerprint"))) {
        Add(result, "/fixture/pair_fingerprint",
            "must be a lowercase sha256 digest");
      }
      if (!fixture.contains("plugin_ids") ||
          !fixture.at("plugin_ids").is_array() ||
          fixture.at("plugin_ids").empty()) {
        Add(result, "/fixture/plugin_ids",
            "must be a non-empty array");
      } else {
        std::string previous;
        for (std::size_t index = 0;
             index < fixture.at("plugin_ids").size(); ++index) {
          const auto& plugin = fixture.at("plugin_ids").at(index);
          if (!NonEmptyString(plugin) ||
              (!previous.empty() && plugin.get<std::string>() <= previous)) {
            Add(result, "/fixture/plugin_ids/" + std::to_string(index),
                "must be a sorted unique non-empty string");
          } else {
            previous = plugin.get<std::string>();
          }
        }
      }
      StringEnum(fixture, "parallelism",
                 {"map-update-disabled", "sequential", "parallel"},
                 "/fixture", result);
      for (const char* field : {"decoded_point_count",
                                "decoded_point_bytes"}) {
        if (!fixture.contains(field) ||
            !NonNegativeInteger(fixture.at(field)) ||
            fixture.at(field).get<uint64_t>() == 0) {
          Add(result, std::string("/fixture/") + field,
              "must be a positive integer");
        }
      }
    }
  }
}

}  // namespace

std::string ValidationResult::Summary() const {
  std::ostringstream output;
  for (std::size_t index = 0; index < issues.size(); ++index) {
    if (index != 0) output << '\n';
    output << issues[index].pointer << ": " << issues[index].message;
  }
  return output.str();
}

ValidationResult ValidatePerformanceReport(const Json& report) {
  ValidationResult result;
  const std::vector<std::string> top_fields = {
      "schema_version", "run_id", "measurement_role", "profile",
      "scenario", "iteration", "git", "build", "machine", "fixture",
      "timing", "stage_timings", "process_memory", "owner_memory",
      "points", "io", "cache", "artifacts", "failures", "result"};
  ClosedObject(report, top_fields, "", result);
  if (!report.is_object()) return result;
  if (!report.contains("schema_version") ||
      !NonNegativeInteger(report.at("schema_version")) ||
      report.at("schema_version").get<uint64_t>() != 1) {
    Add(result, "/schema_version", "must equal 1");
  }
  if (!report.contains("run_id") || !NonEmptyString(report.at("run_id"))) {
    Add(result, "/run_id", "must be a non-empty string");
  }
  StringEnum(report, "measurement_role", {"public", "owner"}, "", result);
  StringEnum(report, "profile",
             {"contract", "pr", "nightly", "external", "gpu"}, "",
             result);
  StringEnum(report, "scenario",
             {"open", "data-load", "alignment", "map-update-sequential",
              "map-update-parallel", "save-fallback", "visualization-cold",
              "visualization-warm", "full-pipeline", "cancellation"},
             "", result);
  if (!report.contains("iteration") ||
      !NonNegativeInteger(report.at("iteration")) ||
      report.at("iteration").get<uint64_t>() == 0) {
    Add(result, "/iteration", "must be a positive integer");
  }
  ValidateMetadata(report, result);

  if (report.contains("timing")) {
    ValidateMetricGroup(report.at("timing"),
                        {"wall_time_ns", "cpu_time_ns", "stage_latency_ns",
                         "command_latency_ns", "cancellation_latency_ns"},
                        {}, {}, "/timing", result);
  }
  if (report.contains("stage_timings")) {
    const auto& timings = report.at("stage_timings");
    if (!timings.is_array()) {
      Add(result, "/stage_timings", "must be an array");
    } else {
      std::set<std::string> observed;
      uint64_t previous_terminal = 0;
      for (std::size_t index = 0; index < timings.size(); ++index) {
        const auto pointer = "/stage_timings/" + std::to_string(index);
        const auto& stage = timings.at(index);
        ClosedObject(stage,
                     {"stage", "latency_ns", "start_sequence",
                      "terminal_sequence"},
                     pointer, result);
        if (!stage.is_object()) continue;
        StringEnum(stage, "stage",
                   {"data-load", "alignment", "map-update", "save"},
                   pointer, result);
        for (const char* field : {"latency_ns", "start_sequence",
                                  "terminal_sequence"}) {
          if (!stage.contains(field) ||
              !NonNegativeInteger(stage.at(field))) {
            Add(result, pointer + "/" + field,
                "must be a non-negative integer");
          }
        }
        if (!stage.contains("stage") || !stage.at("stage").is_string() ||
            !stage.contains("start_sequence") ||
            !NonNegativeInteger(stage.at("start_sequence")) ||
            !stage.contains("terminal_sequence") ||
            !NonNegativeInteger(stage.at("terminal_sequence"))) {
          continue;
        }
        const auto name = stage.at("stage").get<std::string>();
        const auto start = stage.at("start_sequence").get<uint64_t>();
        const auto terminal = stage.at("terminal_sequence").get<uint64_t>();
        if (!observed.insert(name).second) {
          Add(result, pointer + "/stage", "must be unique");
        }
        if (start == 0 || terminal <= start || start <= previous_terminal) {
          Add(result, pointer, "stage event sequences must be ordered");
        }
        previous_terminal = terminal;
      }
      const bool full_pipeline =
          report.contains("scenario") && report.at("scenario").is_string() &&
          report.at("scenario") == "full-pipeline";
      if (full_pipeline) {
        for (const char* required : {"data-load", "alignment", "save"}) {
          if (!observed.contains(required)) {
            Add(result, "/stage_timings",
                std::string("full pipeline is missing ") + required);
          }
        }
        if (report.contains("fixture") && report.at("fixture").is_object() &&
            report.at("fixture").value("parallelism", "") !=
                "map-update-disabled" &&
            !observed.contains("map-update")) {
          Add(result, "/stage_timings",
              "enabled full pipeline is missing map-update");
        }
      }
    }
  }
  if (report.contains("process_memory")) {
    ValidateMetricGroup(
        report.at("process_memory"),
        {"rss_start_bytes", "rss_end_bytes", "sampled_peak_rss_bytes",
         "process_hwm_bytes", "target_peak_delta_bytes", "sample_count",
         "sample_interval_ns"},
        {"retained_rss_delta_bytes"}, {}, "/process_memory", result,
        {"memory_confidence"});
    if (report.at("process_memory").is_object()) {
      StringEnum(report.at("process_memory"), "memory_confidence",
                 {"low", "normal"}, "/process_memory", result);
      for (const char* field : {"sample_count", "sample_interval_ns"}) {
        if (report.at("process_memory").contains(field) &&
            NonNegativeInteger(report.at("process_memory").at(field)) &&
            report.at("process_memory").at(field).get<uint64_t>() == 0) {
          Add(result, std::string("/process_memory/") + field,
              "must be positive when available");
        }
      }
    }
  }
  if (report.contains("owner_memory")) {
    ValidateMetricGroup(
        report.at("owner_memory"),
        {"governor_reserved_total_bytes", "governor_resident_payload_bytes",
         "governor_transient_task_bytes", "governor_heavy_map_bytes",
         "governor_peak_reserved_total_bytes",
         "governor_peak_resident_payload_bytes",
         "governor_peak_transient_task_bytes",
         "governor_peak_heavy_map_bytes",
         "governor_admission_failures", "executor_worker_count",
         "executor_queue_capacity", "executor_max_queued_tasks",
         "executor_max_active_tasks", "executor_max_waiting_submitters"},
        {}, {}, "/owner_memory", result);
  }
  if (report.contains("points")) {
    ValidateMetricGroup(
        report.at("points"),
        {"input_file_bytes", "decoded_source_points", "decoded_source_bytes",
         "retained_filtered_points", "retained_capacity_bytes",
         "optimized_pose_count", "map_output_points",
         "map_output_file_bytes", "visualization_source_points",
         "visualization_displayed_points", "cache_entry_points",
         "public_dto_point_bytes", "gui_staging_bytes"},
        {}, {}, "/points", result);
  }
  if (report.contains("io")) {
    ValidateMetricGroup(report.at("io"),
                        {"rchar", "wchar", "syscr", "syscw", "read_bytes",
                         "write_bytes", "cancelled_write_bytes"},
                        {}, {}, "/io", result);
  }
  if (report.contains("cache")) {
    ValidateMetricGroup(
        report.at("cache"),
        {"entries", "bytes", "hits", "misses", "insertions", "evictions",
         "clears", "returned_snapshot_bytes", "cpu_staging_bytes",
         "gpu_upload_requested_bytes", "gpu_estimated_resident_bytes"},
        {}, {}, "/cache", result, {"gpu_measurement_kind"});
    if (report.at("cache").is_object()) {
      StringEnum(report.at("cache"), "gpu_measurement_kind",
                 {"estimate", "driver_observed", "not_available"},
                 "/cache", result);
    }
  }
  if (report.contains("artifacts")) {
    const auto& artifacts = report.at("artifacts");
    ValidateMetricGroup(
        artifacts,
        {"logical_output_bytes", "retained_owner_bytes",
         "sampled_target_peak_delta_bytes", "process_hwm_bytes"},
        {}, {"peak_to_output_ratio", "peak_to_retained_ratio"},
        "/artifacts", result, {"output_digest", "correctness"});
    if (artifacts.is_object()) {
      if (!artifacts.contains("output_digest") ||
          !Sha256(artifacts.at("output_digest"))) {
        Add(result, "/artifacts/output_digest",
            "must be a lowercase sha256 digest");
      }
      StringEnum(artifacts, "correctness", {"pass", "fail"},
                 "/artifacts", result);
    }
  }
  if (report.contains("failures")) {
    const auto& failures = report.at("failures");
    if (!failures.is_array()) {
      Add(result, "/failures", "must be an array");
    } else {
      for (std::size_t index = 0; index < failures.size(); ++index) {
        const auto pointer = "/failures/" + std::to_string(index);
        ClosedObject(failures.at(index), {"phase", "message"}, pointer,
                     result);
        if (failures.at(index).is_object()) {
          for (const char* field : {"phase", "message"}) {
            if (!failures.at(index).contains(field) ||
                !NonEmptyString(failures.at(index).at(field))) {
              Add(result, pointer + "/" + field,
                  "must be a non-empty string");
            }
          }
        }
      }
    }
  }
  StringEnum(report, "result",
             {"pass", "fail", "uncalibrated", "not_available", "invalid"},
             "", result);
  return result;
}

Json MachineMetadata() {
  utsname system{};
  const bool has_uname = ::uname(&system) == 0;
  const long cpu_count = ::sysconf(_SC_NPROCESSORS_ONLN);
  const long pages = ::sysconf(_SC_PHYS_PAGES);
  const long page_size = ::sysconf(_SC_PAGESIZE);

  std::string cpu_model = "unknown";
  std::ifstream cpuinfo("/proc/cpuinfo");
  for (std::string line; std::getline(cpuinfo, line);) {
    if (!line.starts_with("model name")) continue;
    const auto separator = line.find(':');
    if (separator != std::string::npos) {
      cpu_model = line.substr(separator + 1);
      const auto first = cpu_model.find_first_not_of(" \t");
      if (first != std::string::npos) cpu_model.erase(0, first);
    }
    break;
  }
  std::string affinity = "unknown";
  std::ifstream status("/proc/self/status");
  for (std::string line; std::getline(status, line);) {
    if (!line.starts_with("Cpus_allowed_list:")) continue;
    affinity = line.substr(std::string("Cpus_allowed_list:").size());
    const auto first = affinity.find_first_not_of(" \t");
    if (first != std::string::npos) affinity.erase(0, first);
    break;
  }
  const uint64_t memory_bytes =
      pages > 0 && page_size > 0
          ? static_cast<uint64_t>(pages) * static_cast<uint64_t>(page_size)
          : 1;
  constexpr uint64_t gibibyte = 1024ULL * 1024ULL * 1024ULL;
  uint64_t lower_gib = 1;
  const uint64_t memory_gib = std::max<uint64_t>(1, memory_bytes / gibibyte);
  while (lower_gib <= memory_gib / 2) lower_gib *= 2;
  const std::string memory_class = std::to_string(lower_gib) + "-" +
                                   std::to_string(lower_gib * 2 - 1) +
                                   "GiB";
  return {{"os", has_uname ? system.sysname : "unknown"},
          {"kernel", has_uname ? system.release : "unknown"},
          {"cpu_model", std::move(cpu_model)},
          {"cpu_count", cpu_count > 0 ? cpu_count : 1},
          {"cpu_affinity", std::move(affinity)},
          {"memory_bytes", memory_bytes},
          {"memory_class", memory_class}};
}

void WriteJsonExclusive(const std::filesystem::path& path,
                        const Json& value) {
  const std::string serialized = value.dump(2) + "\n";
  const int descriptor =
      open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
  if (descriptor < 0) {
    throw std::runtime_error("refusing to overwrite benchmark report " +
                             path.string() + ": errno=" +
                             std::to_string(errno));
  }
  std::size_t written = 0;
  while (written < serialized.size()) {
    const ssize_t count =
        write(descriptor, serialized.data() + written,
              serialized.size() - written);
    if (count <= 0) {
      const int error = errno;
      close(descriptor);
      throw std::runtime_error("failed to write benchmark report: errno=" +
                               std::to_string(error));
    }
    written += static_cast<std::size_t>(count);
  }
  if (close(descriptor) != 0) {
    throw std::runtime_error("failed to close benchmark report");
  }
}

}  // namespace open_lmm::test::benchmark
