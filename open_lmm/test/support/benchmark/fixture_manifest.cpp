#include "fixture_manifest.hpp"

#include "tools/replay/replay_sha256.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace open_lmm::test::benchmark {
namespace {

using Json = nlohmann::json;

void Add(ValidationResult& result, std::string pointer,
         std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

bool Unsigned(const Json& value) {
  return value.is_number_unsigned() ||
         (value.is_number_integer() && value.get<int64_t>() >= 0);
}

bool NonEmpty(const Json& value) {
  return value.is_string() && !value.get_ref<const std::string&>().empty();
}

bool Sha256(const Json& value) {
  if (!value.is_string()) return false;
  const auto& text = value.get_ref<const std::string&>();
  return text.size() == 71 && text.starts_with("sha256:") &&
         std::all_of(text.begin() + 7, text.end(),
                     [](unsigned char character) {
                       return (character >= '0' && character <= '9') ||
                              (character >= 'a' && character <= 'f');
                     });
}

void Closed(const Json& value, const std::vector<std::string>& fields,
            std::string pointer, ValidationResult& result) {
  if (!value.is_object()) {
    Add(result, std::move(pointer), "must be an object");
    return;
  }
  for (const auto& field : fields) {
    if (!value.contains(field)) {
      Add(result, pointer + "/" + field, "required field is missing");
    }
  }
  for (auto field = value.begin(); field != value.end(); ++field) {
    if (std::find(fields.begin(), fields.end(), field.key()) == fields.end()) {
      Add(result, pointer + "/" + field.key(), "unknown field");
    }
  }
}

bool SafeRelativePath(const Json& value) {
  if (!NonEmpty(value)) return false;
  const std::filesystem::path path(value.get<std::string>());
  if (path.is_absolute()) return false;
  return std::none_of(path.begin(), path.end(), [](const auto& component) {
    return component == ".." || component == "." || component.empty();
  });
}

bool CheckedProduct(uint64_t left, uint64_t right, uint64_t& product) {
  if (left != 0 && right > std::numeric_limits<uint64_t>::max() / left) {
    return false;
  }
  product = left * right;
  return true;
}

}  // namespace

ValidationResult ValidateFixtureManifest(const Json& manifest) {
  ValidationResult result;
  const std::vector<std::string> fields = {
      "schema_version", "fixture_id", "fixture_version",
      "generator_version", "seed", "source_kind", "agent_count",
      "scans_per_agent", "total_scan_count", "points_per_scan",
      "decoded_point_count", "sizeof_point", "decoded_point_bytes",
      "pose_count", "on_disk_bytes", "config_files", "input_files",
      "plugins", "voxel_sizes", "resource_budget", "license", "provenance",
      "redistribution"};
  Closed(manifest, fields, "", result);
  if (!manifest.is_object()) return result;
  if (!manifest.contains("schema_version") ||
      !Unsigned(manifest.at("schema_version")) ||
      manifest.at("schema_version").get<uint64_t>() != 1) {
    Add(result, "/schema_version", "must equal 1");
  }
  for (const char* field : {"fixture_id", "generator_version", "license",
                            "provenance"}) {
    if (!manifest.contains(field) || !NonEmpty(manifest.at(field))) {
      Add(result, std::string("/") + field, "must be a non-empty string");
    }
  }
  for (const char* field : {"fixture_version", "seed", "agent_count",
                            "scans_per_agent", "total_scan_count",
                            "points_per_scan", "decoded_point_count",
                            "sizeof_point", "decoded_point_bytes",
                            "pose_count", "on_disk_bytes"}) {
    if (!manifest.contains(field) || !Unsigned(manifest.at(field))) {
      Add(result, std::string("/") + field,
          "must be a non-negative integer");
    }
  }
  if (manifest.contains("source_kind") &&
      (!manifest.at("source_kind").is_string() ||
       !std::set<std::string>{"generated", "locked_external"}.contains(
           manifest.at("source_kind").get<std::string>()))) {
    Add(result, "/source_kind", "must be generated or locked_external");
  }
  if (!manifest.contains("redistribution") ||
      !manifest.at("redistribution").is_boolean()) {
    Add(result, "/redistribution", "must be a boolean");
  }

  const auto integer = [&](const char* key) -> uint64_t {
    return manifest.contains(key) && Unsigned(manifest.at(key))
               ? manifest.at(key).get<uint64_t>()
               : 0;
  };
  uint64_t total_scans = 0;
  uint64_t decoded_points = 0;
  uint64_t decoded_bytes = 0;
  const bool arithmetic_ok =
      CheckedProduct(integer("agent_count"), integer("scans_per_agent"),
                     total_scans) &&
      CheckedProduct(total_scans, integer("points_per_scan"),
                     decoded_points) &&
      CheckedProduct(decoded_points, integer("sizeof_point"), decoded_bytes);
  if (!arithmetic_ok || total_scans != integer("total_scan_count")) {
    Add(result, "/total_scan_count", "does not match fixture shape");
  }
  if (!arithmetic_ok || decoded_points != integer("decoded_point_count")) {
    Add(result, "/decoded_point_count", "does not match fixture shape");
  }
  if (!arithmetic_ok || decoded_bytes != integer("decoded_point_bytes")) {
    Add(result, "/decoded_point_bytes",
        "does not match decoded points times sizeof_point");
  }
  if (integer("pose_count") != total_scans) {
    Add(result, "/pose_count", "must equal total scan count");
  }

  for (const char* collection : {"config_files", "input_files"}) {
    if (!manifest.contains(collection) ||
        !manifest.at(collection).is_array() ||
        manifest.at(collection).empty()) {
      Add(result, std::string("/") + collection,
          "must be a non-empty array");
      continue;
    }
    for (std::size_t index = 0; index < manifest.at(collection).size();
         ++index) {
      const auto pointer = std::string("/") + collection + "/" +
                           std::to_string(index);
      const auto& file = manifest.at(collection).at(index);
      Closed(file, {"path", "sha256", "bytes"}, pointer, result);
      if (!file.is_object()) continue;
      if (!file.contains("path") || !SafeRelativePath(file.at("path"))) {
        Add(result, pointer + "/path", "must be a safe relative path");
      }
      if (!file.contains("sha256") || !Sha256(file.at("sha256"))) {
        Add(result, pointer + "/sha256",
            "must be a lowercase sha256 digest");
      }
      if (!file.contains("bytes") || !Unsigned(file.at("bytes"))) {
        Add(result, pointer + "/bytes",
            "must be a non-negative integer");
      }
    }
  }
  if (!manifest.contains("plugins") || !manifest.at("plugins").is_array() ||
      manifest.at("plugins").empty()) {
    Add(result, "/plugins", "must be a non-empty array");
  } else {
    for (std::size_t index = 0; index < manifest.at("plugins").size();
         ++index) {
      const auto pointer = "/plugins/" + std::to_string(index);
      const auto& plugin = manifest.at("plugins").at(index);
      Closed(plugin, {"id", "capability"}, pointer, result);
      if (plugin.is_object()) {
        for (const char* field : {"id", "capability"}) {
          if (!plugin.contains(field) || !NonEmpty(plugin.at(field))) {
            Add(result, pointer + "/" + field,
                "must be a non-empty string");
          }
        }
      }
    }
  }
  if (!manifest.contains("voxel_sizes") ||
      !manifest.at("voxel_sizes").is_object()) {
    Add(result, "/voxel_sizes", "must be an object");
  } else {
    Closed(manifest.at("voxel_sizes"),
           {"input_m", "output_m", "visualization_m"}, "/voxel_sizes",
           result);
    for (const char* field : {"input_m", "output_m", "visualization_m"}) {
      if (!manifest.at("voxel_sizes").contains(field) ||
          !manifest.at("voxel_sizes").at(field).is_number() ||
          !std::isfinite(
              manifest.at("voxel_sizes").at(field).get<double>()) ||
          manifest.at("voxel_sizes").at(field).get<double>() <= 0.0) {
        Add(result, std::string("/voxel_sizes/") + field,
            "must be a positive finite number");
      }
    }
  }
  if (!manifest.contains("resource_budget") ||
      !manifest.at("resource_budget").is_object()) {
    Add(result, "/resource_budget", "must be an object");
  } else {
    Closed(manifest.at("resource_budget"),
           {"max_agent_tasks", "max_cpu_threads", "soft_memory_bytes",
            "enable_map_update", "parallel_map_update"},
           "/resource_budget", result);
    for (const char* field : {"max_agent_tasks", "max_cpu_threads",
                              "soft_memory_bytes"}) {
      if (!manifest.at("resource_budget").contains(field) ||
          !Unsigned(manifest.at("resource_budget").at(field)) ||
          manifest.at("resource_budget").at(field).get<uint64_t>() == 0) {
        Add(result, std::string("/resource_budget/") + field,
            "must be a positive integer");
      }
    }
    for (const char* field : {"enable_map_update", "parallel_map_update"}) {
      if (!manifest.at("resource_budget").contains(field) ||
          !manifest.at("resource_budget").at(field).is_boolean()) {
        Add(result, std::string("/resource_budget/") + field,
            "must be a boolean");
      }
    }
    if (manifest.at("resource_budget").contains("enable_map_update") &&
        manifest.at("resource_budget").contains("parallel_map_update") &&
        manifest.at("resource_budget").at("enable_map_update").is_boolean() &&
        manifest.at("resource_budget").at("parallel_map_update").is_boolean() &&
        manifest.at("resource_budget").at("parallel_map_update").get<bool>() &&
        !manifest.at("resource_budget").at("enable_map_update").get<bool>()) {
      Add(result, "/resource_budget/parallel_map_update",
          "parallel mode requires MapUpdate to be enabled");
    }
  }

  if (manifest.contains("fixture_id") && NonEmpty(manifest.at("fixture_id"))) {
    const auto id = manifest.at("fixture_id").get<std::string>();
    if (id == "small-v1" &&
        (integer("agent_count") != 2 || integer("scans_per_agent") != 8 ||
         integer("points_per_scan") != 4096)) {
      Add(result, "/fixture_id", "small-v1 shape is immutable");
    } else if (id == "medium-v1" &&
               (integer("agent_count") != 2 ||
                integer("scans_per_agent") != 64 ||
                integer("points_per_scan") != 32768)) {
      Add(result, "/fixture_id", "medium-v1 shape is immutable");
    } else if (id == "large-external-v1" &&
               integer("decoded_point_count") < 20'000'000 &&
               integer("decoded_point_bytes") < 2ULL * 1024ULL * 1024ULL *
                                                    1024ULL) {
      Add(result, "/fixture_id",
          "large-external-v1 does not meet its minimum size");
    }
  }
  return result;
}

std::string BenchmarkConfigFingerprint(const Json& manifest) {
  std::string canonical;
  for (const auto& file : manifest.at("config_files")) {
    canonical += file.at("path").get<std::string>() + ":" +
                 file.at("sha256").get<std::string>() + "\n";
  }
  return "sha256:" + replay::Sha256(canonical);
}

std::string BenchmarkPairFingerprint(const std::filesystem::path& fixture_root,
                                     const Json& manifest) {
  std::string canonical;
  for (const auto& file : manifest.at("config_files")) {
    const std::string relative = file.at("path").get<std::string>();
    std::ifstream input(fixture_root / relative);
    if (!input) {
      throw std::runtime_error("failed to read paired config " + relative);
    }
    Json document;
    input >> document;
    if (relative == "config/server/map_server.json") {
      document.at("map_server")["parallel_map_update"] = false;
    }
    canonical += relative + ":" + document.dump() + "\n";
  }
  for (const auto& file : manifest.at("input_files")) {
    canonical += file.at("path").get<std::string>() + ":" +
                 file.at("sha256").get<std::string>() + "\n";
  }
  canonical += manifest.at("plugins").dump();
  canonical += manifest.at("voxel_sizes").dump();
  return "sha256:" + replay::Sha256(canonical);
}

Json BenchmarkPluginIds(const Json& manifest) {
  std::vector<std::string> ids;
  for (const auto& plugin : manifest.at("plugins")) {
    ids.push_back(plugin.at("id").get<std::string>());
  }
  std::sort(ids.begin(), ids.end());
  return ids;
}

}  // namespace open_lmm::test::benchmark
