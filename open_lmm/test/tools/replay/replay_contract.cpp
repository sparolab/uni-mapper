#include "replay_contract.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

namespace open_lmm::test::replay {
namespace {

using Json = nlohmann::json;

constexpr std::string_view kSha256Prefix = "sha256:";

void Add(ValidationResult& result, std::string pointer, std::string message) {
  result.issues.push_back({std::move(pointer), std::move(message)});
}

bool IsNonEmptyString(const Json& value) {
  return value.is_string() && !value.get_ref<const std::string&>().empty();
}

bool IsNonnegativeInteger(const Json& value) {
  return value.is_number_unsigned() ||
         (value.is_number_integer() && value.get<std::int64_t>() >= 0);
}

bool IsSha256(const Json& value) {
  if (!value.is_string()) return false;
  std::string digest = value.get<std::string>();
  if (digest.starts_with(kSha256Prefix)) {
    digest.erase(0, kSha256Prefix.size());
  }
  return digest.size() == 64 &&
         std::all_of(digest.begin(), digest.end(), [](unsigned char character) {
           return (character >= '0' && character <= '9') ||
                  (character >= 'a' && character <= 'f');
         });
}

bool IsCommitId(const Json& value) {
  if (!value.is_string()) return false;
  const std::string& commit = value.get_ref<const std::string&>();
  return commit.size() == 40 &&
         std::all_of(commit.begin(), commit.end(), [](unsigned char character) {
           return (character >= '0' && character <= '9') ||
                  (character >= 'a' && character <= 'f');
         });
}

bool IsSafeRelativePath(const Json& value) {
  if (!IsNonEmptyString(value)) return false;
  const std::filesystem::path path(value.get<std::string>());
  if (path.is_absolute() || path.has_root_name() || path.has_root_directory()) {
    return false;
  }
  for (const auto& component : path) {
    if (component == ".." || component == ".") return false;
  }
  return path.lexically_normal() == path;
}

void RequireObject(const Json& value, std::string_view pointer,
                   ValidationResult& result) {
  if (!value.is_object()) Add(result, std::string(pointer), "must be an object");
}

void RejectUnknown(const Json& object, std::string_view pointer,
                   std::initializer_list<std::string_view> allowed,
                   ValidationResult& result) {
  if (!object.is_object()) return;
  for (const auto& [key, unused] : object.items()) {
    (void)unused;
    if (std::find(allowed.begin(), allowed.end(), key) == allowed.end()) {
      Add(result, std::string(pointer) + "/" + key, "unknown field");
    }
  }
}

const Json* Required(const Json& object, std::string_view key,
                     std::string_view pointer, ValidationResult& result) {
  if (!object.is_object()) return nullptr;
  const auto found = object.find(key);
  if (found == object.end()) {
    Add(result, std::string(pointer) + "/" + std::string(key),
        "required field is missing");
    return nullptr;
  }
  return &*found;
}

void ValidateDigest(const Json* value, std::string pointer,
                    ValidationResult& result) {
  if (value != nullptr && !IsSha256(*value)) {
    Add(result, std::move(pointer),
        "must be 64 lowercase hexadecimal SHA-256 characters, optionally prefixed by sha256:");
  }
}

void ValidateFileLock(const Json& value, std::string pointer,
                      ValidationResult& result) {
  RequireObject(value, pointer, result);
  if (!value.is_object()) return;
  RejectUnknown(value, pointer, {"path", "sha256"}, result);
  const Json* path = Required(value, "path", pointer, result);
  const Json* digest = Required(value, "sha256", pointer, result);
  if (path != nullptr && !IsSafeRelativePath(*path)) {
    Add(result, pointer + "/path", "must be a normalized relative path without dot segments");
  }
  ValidateDigest(digest, pointer + "/sha256", result);
}

bool IsEnum(const Json& value, std::initializer_list<std::string_view> values) {
  if (!value.is_string()) return false;
  const std::string& candidate = value.get_ref<const std::string&>();
  return std::find(values.begin(), values.end(), candidate) != values.end();
}

void ValidateAgent(const Json& value, std::size_t index,
                   ValidationResult& result, std::set<std::string>& agent_ids) {
  const std::string pointer = "/dataset/agents/" + std::to_string(index);
  RequireObject(value, pointer, result);
  if (!value.is_object()) return;
  RejectUnknown(value, pointer,
                {"id", "frames", "pose_file", "scan_index"}, result);
  const Json* id = Required(value, "id", pointer, result);
  const Json* frames = Required(value, "frames", pointer, result);
  const Json* pose_file = Required(value, "pose_file", pointer, result);
  const Json* scan_index = Required(value, "scan_index", pointer, result);
  if (id != nullptr) {
    if (!IsNonEmptyString(*id)) {
      Add(result, pointer + "/id", "must be a non-empty string");
    } else if (!agent_ids.insert(id->get<std::string>()).second) {
      Add(result, pointer + "/id", "duplicate agent id");
    }
  }
  if (frames != nullptr) {
    if (!frames->is_array() || frames->empty()) {
      Add(result, pointer + "/frames", "must be a non-empty array");
    } else {
      std::set<std::uint64_t> seen;
      std::uint64_t previous = 0;
      bool first = true;
      for (std::size_t frame_index = 0; frame_index < frames->size(); ++frame_index) {
        const Json& frame = (*frames)[frame_index];
        const std::string frame_pointer =
            pointer + "/frames/" + std::to_string(frame_index);
        if (!IsNonnegativeInteger(frame)) {
          Add(result, frame_pointer, "must be an unsigned frame id");
          continue;
        }
        const auto number = frame.get<std::uint64_t>();
        if (!seen.insert(number).second) {
          Add(result, frame_pointer, "duplicate frame id");
        }
        if (!first && number <= previous) {
          Add(result, frame_pointer, "frame ids must be strictly increasing");
        }
        previous = number;
        first = false;
      }
    }
  }
  if (pose_file != nullptr) ValidateFileLock(*pose_file, pointer + "/pose_file", result);
  if (scan_index != nullptr) {
    ValidateFileLock(*scan_index, pointer + "/scan_index", result);
  }
}

void ValidateDataset(const Json& value, ValidationResult& result) {
  constexpr std::string_view pointer = "/dataset";
  RequireObject(value, pointer, result);
  if (!value.is_object()) return;
  RejectUnknown(value, pointer,
                {"bundle_id", "bundle_sha256", "source", "license",
                 "redistributable", "agents"}, result);
  for (std::string_view key : {"bundle_id", "source", "license"}) {
    const Json* field = Required(value, key, pointer, result);
    if (field != nullptr && !IsNonEmptyString(*field)) {
      Add(result, std::string(pointer) + "/" + std::string(key),
          "must be a non-empty string");
    }
  }
  ValidateDigest(Required(value, "bundle_sha256", pointer, result),
                 "/dataset/bundle_sha256", result);
  const Json* redistributable = Required(value, "redistributable", pointer, result);
  if (redistributable != nullptr && !redistributable->is_boolean()) {
    Add(result, "/dataset/redistributable", "must be a boolean");
  }
  const Json* agents = Required(value, "agents", pointer, result);
  if (agents == nullptr) return;
  if (!agents->is_array() || agents->empty()) {
    Add(result, "/dataset/agents", "must be a non-empty array");
    return;
  }
  std::set<std::string> agent_ids;
  for (std::size_t index = 0; index < agents->size(); ++index) {
    ValidateAgent((*agents)[index], index, result, agent_ids);
  }
}

void ValidateConfig(const Json& value, ValidationResult& result) {
  constexpr std::string_view pointer = "/config";
  RequireObject(value, pointer, result);
  if (!value.is_object()) return;
  RejectUnknown(value, pointer, {"root", "files", "plugins"}, result);
  const Json* root = Required(value, "root", pointer, result);
  if (root != nullptr && !IsSafeRelativePath(*root)) {
    Add(result, "/config/root",
        "must be a normalized relative path without dot segments");
  }
  const Json* files = Required(value, "files", pointer, result);
  if (files != nullptr) {
    if (!files->is_array() || files->empty()) {
      Add(result, "/config/files", "must be a non-empty array");
    } else {
      std::set<std::string> paths;
      for (std::size_t index = 0; index < files->size(); ++index) {
        const std::string item_pointer = "/config/files/" + std::to_string(index);
        ValidateFileLock((*files)[index], item_pointer, result);
        if ((*files)[index].is_object() && (*files)[index].contains("path") &&
            (*files)[index]["path"].is_string() &&
            !paths.insert((*files)[index]["path"].get<std::string>()).second) {
          Add(result, item_pointer + "/path", "duplicate config path");
        }
      }
      if (root != nullptr && root->is_string() &&
          !paths.contains(root->get<std::string>())) {
        Add(result, "/config/root", "must also appear in config files");
      }
    }
  }
  const Json* plugins = Required(value, "plugins", pointer, result);
  if (plugins != nullptr) {
    if (!plugins->is_array()) {
      Add(result, "/config/plugins", "must be an array");
    } else {
      std::set<std::pair<std::string, std::string>> keys;
      for (std::size_t index = 0; index < plugins->size(); ++index) {
        const Json& plugin = (*plugins)[index];
        const std::string item_pointer = "/config/plugins/" + std::to_string(index);
        RequireObject(plugin, item_pointer, result);
        if (!plugin.is_object()) continue;
        RejectUnknown(plugin, item_pointer, {"kind", "id", "capability"}, result);
        const Json* kind = Required(plugin, "kind", item_pointer, result);
        const Json* id = Required(plugin, "id", item_pointer, result);
        const Json* capability = Required(plugin, "capability", item_pointer, result);
        if (kind != nullptr && !IsNonEmptyString(*kind))
          Add(result, item_pointer + "/kind", "must be a non-empty string");
        if (id != nullptr && !IsNonEmptyString(*id))
          Add(result, item_pointer + "/id", "must be a non-empty string");
        if (capability != nullptr && !IsNonEmptyString(*capability))
          Add(result, item_pointer + "/capability", "must be a non-empty string");
        if (kind != nullptr && id != nullptr && kind->is_string() && id->is_string() &&
            !keys.emplace(kind->get<std::string>(), id->get<std::string>()).second) {
          Add(result, item_pointer, "duplicate plugin kind/id");
        }
      }
    }
  }
}

void ValidateWorkflow(const Json& value, ValidationResult& result) {
  if (!value.is_array() || value.empty()) {
    Add(result, "/workflow", "must be a non-empty array");
    return;
  }
  static constexpr std::string_view kExpectedStages[] = {
      "DataLoad", "Alignment", "MapUpdate", "Save"};
  if (value.size() < std::size(kExpectedStages)) {
    Add(result, "/workflow", "must contain the four required stages in order");
  }
  std::size_t expected_index = 0;
  for (std::size_t index = 0; index < value.size(); ++index) {
    const Json& step = value[index];
    const std::string pointer = "/workflow/" + std::to_string(index);
    RequireObject(step, pointer, result);
    if (!step.is_object()) continue;
    RejectUnknown(step, pointer, {"stage", "agents", "config_change"}, result);
    const Json* stage = Required(step, "stage", pointer, result);
    const Json* agents = Required(step, "agents", pointer, result);
    if (stage != nullptr &&
        !IsEnum(*stage, {"DataLoad", "Alignment", "MapUpdate", "Save",
                        "ApplyConfig"})) {
      Add(result, pointer + "/stage", "unsupported replay stage");
    }
    if (agents != nullptr && !(agents->is_string() || agents->is_array())) {
      Add(result, pointer + "/agents", "must be 'all' or an array of agent ids");
    } else if (agents != nullptr && agents->is_string() && *agents != "all") {
      Add(result, pointer + "/agents", "string value must be 'all'");
    } else if (agents != nullptr && agents->is_array()) {
      std::set<std::string> selected;
      if (agents->empty()) {
        Add(result, pointer + "/agents", "agent array must not be empty");
      }
      for (std::size_t agent_index = 0; agent_index < agents->size();
           ++agent_index) {
        const Json& agent = (*agents)[agent_index];
        const std::string agent_pointer =
            pointer + "/agents/" + std::to_string(agent_index);
        if (!IsNonEmptyString(agent)) {
          Add(result, agent_pointer, "must be a non-empty agent id");
        } else if (!selected.insert(agent.get<std::string>()).second) {
          Add(result, agent_pointer, "duplicate selected agent id");
        }
      }
    }
    if (stage != nullptr && stage->is_string() &&
        expected_index < std::size(kExpectedStages) &&
        *stage == kExpectedStages[expected_index]) {
      ++expected_index;
    }
    if (step.contains("config_change")) {
      if (stage == nullptr || *stage != "ApplyConfig") {
        Add(result, pointer + "/config_change",
            "is allowed only for ApplyConfig");
      }
      const Json& change = step.at("config_change");
      RequireObject(change, pointer + "/config_change", result);
      if (change.is_object()) {
        RejectUnknown(change, pointer + "/config_change",
                      {"domain", "document", "selected_document"}, result);
        const Json* domain = Required(change, "domain",
                                      pointer + "/config_change", result);
        const Json* document = Required(change, "document",
                                        pointer + "/config_change", result);
        if (domain != nullptr &&
            !IsEnum(*domain, {"data_loader", "loop_detector", "optimizer",
                              "dynamic_remover", "map_save"})) {
          Add(result, pointer + "/config_change/domain",
              "unsupported config domain");
        }
        if (document != nullptr && !IsSafeRelativePath(*document)) {
          Add(result, pointer + "/config_change/document",
              "must be a normalized relative path");
        }
        if (change.contains("selected_document") &&
            !IsSafeRelativePath(change.at("selected_document"))) {
          Add(result, pointer + "/config_change/selected_document",
              "must be a normalized relative path");
        }
      }
    } else if (stage != nullptr && *stage == "ApplyConfig") {
      Add(result, pointer + "/config_change",
          "is required for ApplyConfig");
    }
  }
  if (expected_index != std::size(kExpectedStages)) {
    Add(result, "/workflow",
        "required stages must occur in DataLoad, Alignment, MapUpdate, Save order");
  }
}

void ValidateTolerances(const Json& value, ValidationResult& result) {
  RequireObject(value, "/tolerances", result);
  if (!value.is_object()) return;
  if (value.empty()) Add(result, "/tolerances", "must not be empty");
  for (const auto& [name, limit] : value.items()) {
    if (!limit.is_number() || !std::isfinite(limit.get<double>()) ||
        limit.get<double>() < 0.0) {
      Add(result, "/tolerances/" + name, "must be a finite nonnegative number");
    }
  }
}

void ValidateProvenance(const Json& value, ValidationResult& result) {
  RequireObject(value, "/provenance", result);
  if (!value.is_object()) return;
  RejectUnknown(value, "/provenance",
                {"acquired_at", "source_version", "attribution_file",
                 "transformations", "original_sha256"}, result);
  for (std::string_view key : {"acquired_at", "source_version"}) {
    const Json* field = Required(value, key, "/provenance", result);
    if (field != nullptr && !IsNonEmptyString(*field)) {
      Add(result, "/provenance/" + std::string(key), "must be a non-empty string");
    }
  }
  const Json* attribution =
      Required(value, "attribution_file", "/provenance", result);
  if (attribution != nullptr && !IsSafeRelativePath(*attribution)) {
    Add(result, "/provenance/attribution_file", "must be a normalized relative path");
  }
  const Json* transformations =
      Required(value, "transformations", "/provenance", result);
  if (transformations != nullptr) {
    if (!transformations->is_array()) {
      Add(result, "/provenance/transformations", "must be an array");
    } else {
      for (std::size_t index = 0; index < transformations->size(); ++index) {
        if (!IsNonEmptyString((*transformations)[index])) {
          Add(result,
              "/provenance/transformations/" + std::to_string(index),
              "must be a non-empty string");
        }
      }
    }
  }
  ValidateDigest(Required(value, "original_sha256", "/provenance", result),
                 "/provenance/original_sha256", result);
}

void ValidateReportStep(const Json& step, std::size_t index,
                        ValidationResult& result) {
  const std::string pointer = "/steps/" + std::to_string(index);
  RequireObject(step, pointer, result);
  if (!step.is_object()) return;
  RejectUnknown(step, pointer,
                {"stage", "result", "error_code", "revision_before",
                 "revision_after", "artifacts", "events"}, result);
  const Json* stage = Required(step, "stage", pointer, result);
  const Json* stage_result = Required(step, "result", pointer, result);
  const Json* before = Required(step, "revision_before", pointer, result);
  const Json* after = Required(step, "revision_after", pointer, result);
  const Json* artifacts = Required(step, "artifacts", pointer, result);
  const Json* events = Required(step, "events", pointer, result);
  if (stage != nullptr &&
      !IsEnum(*stage, {"Open", "DataLoad", "Alignment", "MapUpdate",
                       "Save", "ApplyConfig"}))
    Add(result, pointer + "/stage", "unsupported replay stage");
  if (stage_result != nullptr &&
      !IsEnum(*stage_result, {"succeeded", "failed", "cancelled"}))
    Add(result, pointer + "/result", "unsupported stage result");
  for (const auto& [value, suffix] : {
           std::pair{before, "/revision_before"},
           std::pair{after, "/revision_after"}}) {
    if (value != nullptr && !IsNonnegativeInteger(*value))
      Add(result, pointer + suffix, "must be unsigned");
  }
  if (artifacts != nullptr && !artifacts->is_array()) {
    Add(result, pointer + "/artifacts", "must be an array");
  }
  if (events != nullptr && !events->is_array()) {
    Add(result, pointer + "/events", "must be an array");
  }
  if (step.contains("error_code") &&
      !(step.at("error_code").is_null() ||
        IsNonEmptyString(step.at("error_code")))) {
    Add(result, pointer + "/error_code",
        "must be null or a non-empty string");
  }
}

bool IsJsonPointer(const Json& value) {
  if (!IsNonEmptyString(value)) return false;
  const std::string& pointer = value.get_ref<const std::string&>();
  if (!pointer.starts_with('/')) return false;
  try {
    (void)Json::json_pointer(pointer);
    return true;
  } catch (const std::exception&) {
    return false;
  }
}

const Json* Resolve(const Json& document, const std::string& pointer) {
  try {
    const Json::json_pointer parsed(pointer);
    if (!document.contains(parsed)) return nullptr;
    return &document.at(parsed);
  } catch (const std::exception&) {
    return nullptr;
  }
}

void Difference(ComparisonResult& result, std::string pointer,
                std::string rule, Json expected, Json actual, Json limit,
                std::string message) {
  result.differences.push_back({std::move(pointer), std::move(rule),
                                std::move(expected), std::move(actual),
                                std::move(limit), std::move(message)});
}

}  // namespace

std::string ValidationResult::Summary() const {
  std::ostringstream output;
  for (const auto& issue : issues) {
    output << issue.pointer << ": " << issue.message << '\n';
  }
  return output.str();
}

Json ComparisonResult::ToJson() const {
  Json output{{"schema_version", 1},
              {"result", Passed() ? "PASS" : "FAIL"},
              {"differences", Json::array()}};
  for (const auto& difference : differences) {
    output["differences"].push_back(
        {{"pointer", difference.pointer},
         {"rule", difference.rule},
         {"expected", difference.expected},
         {"actual", difference.actual},
         {"limit", difference.limit},
         {"message", difference.message}});
  }
  return output;
}

Json LoadJsonFile(const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("failed to open JSON file: " + path.string());
  try {
    Json value;
    input >> value;
    return value;
  } catch (const Json::exception& error) {
    throw std::runtime_error("failed to parse JSON file " + path.string() +
                             ": " + error.what());
  }
}

std::string CanonicalJson(const Json& value) {
  return value.dump(-1, ' ', false, Json::error_handler_t::strict);
}

ValidationResult ValidateCaseManifest(const Json& document) {
  ValidationResult result;
  RequireObject(document, "", result);
  if (!document.is_object()) return result;
  RejectUnknown(document, "",
                {"schema_version", "case_id", "tier", "dataset", "provenance",
                 "config", "workflow", "expected", "tolerances"}, result);
  const Json* version = Required(document, "schema_version", "", result);
  const Json* case_id = Required(document, "case_id", "", result);
  const Json* tier = Required(document, "tier", "", result);
  const Json* dataset = Required(document, "dataset", "", result);
  const Json* provenance = Required(document, "provenance", "", result);
  const Json* config = Required(document, "config", "", result);
  const Json* workflow = Required(document, "workflow", "", result);
  const Json* expected = Required(document, "expected", "", result);
  const Json* tolerances = Required(document, "tolerances", "", result);
  if (version != nullptr && (!IsNonnegativeInteger(*version) || *version != 1))
    Add(result, "/schema_version", "only schema version 1 is supported");
  if (case_id != nullptr && !IsNonEmptyString(*case_id))
    Add(result, "/case_id", "must be a non-empty string");
  if (tier != nullptr && !IsEnum(*tier, {"tiny", "small", "representative", "failure"}))
    Add(result, "/tier", "must be tiny, small, representative, or failure");
  if (dataset != nullptr) ValidateDataset(*dataset, result);
  if (provenance != nullptr) ValidateProvenance(*provenance, result);
  if (config != nullptr) ValidateConfig(*config, result);
  if (workflow != nullptr) {
    ValidateWorkflow(*workflow, result);
    if (config != nullptr && config->is_object() &&
        config->contains("files") && config->at("files").is_array() &&
        workflow->is_array()) {
      std::set<std::string> locked_paths;
      for (const Json& lock : config->at("files")) {
        if (lock.is_object() && lock.contains("path") &&
            lock.at("path").is_string()) {
          locked_paths.insert(lock.at("path").get<std::string>());
        }
      }
      for (std::size_t index = 0; index < workflow->size(); ++index) {
        const Json& step = (*workflow)[index];
        if (!step.is_object() || !step.contains("config_change") ||
            !step.at("config_change").is_object() ||
            !step.at("config_change").contains("document") ||
            !step.at("config_change").at("document").is_string()) {
          continue;
        }
        const std::string document =
            step.at("config_change").at("document").get<std::string>();
        if (!locked_paths.contains(document)) {
          Add(result,
              "/workflow/" + std::to_string(index) +
                  "/config_change/document",
              "must also appear in config files");
        }
      }
    }
  }
  if (expected != nullptr) {
    RequireObject(*expected, "/expected", result);
    if (expected->is_object()) {
      RejectUnknown(*expected, "/expected",
                    {"result", "baseline_id", "failure"}, result);
      const Json* expected_result = Required(*expected, "result", "/expected", result);
      if (expected_result != nullptr && !IsEnum(*expected_result, {"success", "failure"}))
        Add(result, "/expected/result", "must be success or failure");
      if (expected_result != nullptr && *expected_result == "success") {
        const Json* baseline_id =
            Required(*expected, "baseline_id", "/expected", result);
        if (baseline_id != nullptr && !IsNonEmptyString(*baseline_id))
          Add(result, "/expected/baseline_id", "must be a non-empty string");
        if (expected->contains("failure")) {
          Add(result, "/expected/failure",
              "must not be present for a successful case");
        }
      }
      if (expected_result != nullptr && *expected_result == "failure") {
        if (expected->contains("baseline_id")) {
          Add(result, "/expected/baseline_id",
              "must not be present for a failure case");
        }
        const Json* failure = Required(*expected, "failure", "/expected", result);
        if (failure != nullptr) {
          RequireObject(*failure, "/expected/failure", result);
          if (failure->is_object()) {
            RejectUnknown(*failure, "/expected/failure",
                          {"stage", "agent", "error_code",
                           "revision_unchanged", "close_succeeds"}, result);
            const Json* stage =
                Required(*failure, "stage", "/expected/failure", result);
            const Json* error_code =
                Required(*failure, "error_code", "/expected/failure", result);
            const Json* unchanged = Required(
                *failure, "revision_unchanged", "/expected/failure", result);
            const Json* close = Required(
                *failure, "close_succeeds", "/expected/failure", result);
            if (stage != nullptr &&
                !IsEnum(*stage, {"Open", "DataLoad", "Alignment",
                                 "MapUpdate", "Save", "ApplyConfig"})) {
              Add(result, "/expected/failure/stage",
                  "unsupported replay stage");
            }
            if (error_code != nullptr && !IsNonEmptyString(*error_code)) {
              Add(result, "/expected/failure/error_code",
                  "must be a non-empty string");
            }
            if (failure->contains("agent") &&
                !IsNonEmptyString(failure->at("agent"))) {
              Add(result, "/expected/failure/agent",
                  "must be a non-empty string");
            }
            if (unchanged != nullptr && !unchanged->is_boolean()) {
              Add(result, "/expected/failure/revision_unchanged",
                  "must be boolean");
            }
            if (close != nullptr && !close->is_boolean()) {
              Add(result, "/expected/failure/close_succeeds",
                  "must be boolean");
            }
          }
        }
      }
    }
  }
  if (tolerances != nullptr) ValidateTolerances(*tolerances, result);
  return result;
}

ValidationResult ValidateReplayReport(const Json& document) {
  ValidationResult result;
  RequireObject(document, "", result);
  if (!document.is_object()) return result;
  RejectUnknown(document, "",
                {"schema_version", "case_id", "case_manifest_sha256",
                 "dataset_sha256", "config_sha256", "git", "environment",
                 "agents", "steps", "health", "metrics", "artifacts",
                 "diagnostics", "close_result"}, result);
  const Json* version = Required(document, "schema_version", "", result);
  const Json* case_id = Required(document, "case_id", "", result);
  if (version != nullptr && (!IsNonnegativeInteger(*version) || *version != 1))
    Add(result, "/schema_version", "only schema version 1 is supported");
  if (case_id != nullptr && !IsNonEmptyString(*case_id))
    Add(result, "/case_id", "must be a non-empty string");
  for (std::string_view key : {"case_manifest_sha256", "dataset_sha256",
                               "config_sha256"}) {
    ValidateDigest(Required(document, key, "", result),
                   "/" + std::string(key), result);
  }
  const Json* git = Required(document, "git", "", result);
  if (git != nullptr) {
    RequireObject(*git, "/git", result);
    if (git->is_object()) {
      RejectUnknown(*git, "/git", {"commit", "dirty"}, result);
      const Json* commit = Required(*git, "commit", "/git", result);
      const Json* dirty = Required(*git, "dirty", "/git", result);
      if (commit != nullptr && !IsCommitId(*commit)) {
        Add(result, "/git/commit",
            "must be a 40-character lowercase hexadecimal commit id");
      }
      if (dirty != nullptr && !dirty->is_boolean()) {
        Add(result, "/git/dirty", "must be boolean");
      }
    }
  }
  const Json* environment = Required(document, "environment", "", result);
  if (environment != nullptr && !environment->is_object()) {
    Add(result, "/environment", "must be an object");
  }
  const Json* agents = Required(document, "agents", "", result);
  if (agents != nullptr) {
    if (!agents->is_array() || agents->empty()) {
      Add(result, "/agents", "must be a non-empty array");
    } else {
      std::set<std::string> seen;
      for (std::size_t index = 0; index < agents->size(); ++index) {
        const Json& agent = (*agents)[index];
        if (!IsNonEmptyString(agent)) {
          Add(result, "/agents/" + std::to_string(index),
              "must be a non-empty agent id");
        } else if (!seen.insert(agent.get<std::string>()).second) {
          Add(result, "/agents/" + std::to_string(index),
              "duplicate agent id");
        }
      }
    }
  }
  const Json* steps = Required(document, "steps", "", result);
  if (steps != nullptr) {
    if (!steps->is_array() || steps->empty()) {
      Add(result, "/steps", "must be a non-empty array");
    } else {
      for (std::size_t index = 0; index < steps->size(); ++index) {
        ValidateReportStep((*steps)[index], index, result);
      }
    }
  }
  for (std::string_view key : {"health", "metrics", "diagnostics"}) {
    const Json* field = Required(document, key, "", result);
    if (field != nullptr && !field->is_object()) {
      Add(result, "/" + std::string(key), "must be an object");
    }
  }
  const Json* artifacts = Required(document, "artifacts", "", result);
  if (artifacts != nullptr && !artifacts->is_array()) {
    Add(result, "/artifacts", "must be an array");
  }
  const Json* close_result = Required(document, "close_result", "", result);
  if (close_result != nullptr && !IsEnum(*close_result, {"succeeded", "failed"}))
    Add(result, "/close_result", "must be succeeded or failed");
  return result;
}

ValidationResult ValidateReplayBaseline(const Json& document) {
  ValidationResult result;
  RequireObject(document, "", result);
  if (!document.is_object()) return result;
  RejectUnknown(document, "",
                {"schema_version", "baseline_id", "case_id",
                 "case_manifest_sha256", "generator", "exact", "absolute_tolerances",
                 "ranges"}, result);
  const Json* version = Required(document, "schema_version", "", result);
  if (version != nullptr && (!IsNonnegativeInteger(*version) || *version != 1))
    Add(result, "/schema_version", "only schema version 1 is supported");
  for (std::string_view key : {"baseline_id", "case_id"}) {
    const Json* field = Required(document, key, "", result);
    if (field != nullptr && !IsNonEmptyString(*field)) {
      Add(result, "/" + std::string(key), "must be non-empty");
    }
  }
  ValidateDigest(Required(document, "case_manifest_sha256", "", result),
                 "/case_manifest_sha256", result);
  const Json* generator = Required(document, "generator", "", result);
  if (generator != nullptr && !generator->is_object()) {
    Add(result, "/generator", "must be an object");
  }
  std::set<std::string> pointers;
  const auto validate_rules = [&](std::string_view name,
                                  std::initializer_list<std::string_view> allowed) {
    const Json* rules = Required(document, name, "", result);
    if (rules == nullptr) return;
    if (!rules->is_array()) {
      Add(result, "/" + std::string(name), "must be an array");
      return;
    }
    for (std::size_t index = 0; index < rules->size(); ++index) {
      const Json& rule = (*rules)[index];
      std::string pointer;
      pointer.reserve(name.size() + 24);
      pointer.push_back('/');
      pointer.append(name);
      pointer.push_back('/');
      pointer.append(std::to_string(index));
      RequireObject(rule, pointer, result);
      if (!rule.is_object()) continue;
      RejectUnknown(rule, pointer, allowed, result);
      const Json* path = Required(rule, "pointer", pointer, result);
      if (path != nullptr) {
        if (!IsJsonPointer(*path)) {
          Add(result, pointer + "/pointer",
              "must be a valid non-root JSON pointer");
        } else if (!pointers.insert(path->get<std::string>()).second) {
          Add(result, pointer + "/pointer",
              "a report pointer may have only one comparison rule");
        }
      }
    }
  };
  validate_rules("exact", {"pointer", "expected"});
  validate_rules("absolute_tolerances", {"pointer", "expected", "tolerance"});
  validate_rules("ranges", {"pointer", "minimum", "maximum"});
  if (document.contains("exact") && document["exact"].is_array()) {
    for (std::size_t index = 0; index < document["exact"].size(); ++index) {
      const Json& rule = document["exact"][index];
      if (rule.is_object()) {
        Required(rule, "expected",
                 "/exact/" + std::to_string(index), result);
      }
    }
  }
  if (document.contains("absolute_tolerances") && document["absolute_tolerances"].is_array()) {
    for (std::size_t index = 0;
         index < document["absolute_tolerances"].size(); ++index) {
      const Json& rule = document["absolute_tolerances"][index];
      const std::string pointer = "/absolute_tolerances/" + std::to_string(index);
      if (!rule.is_object()) continue;
      const Json* expected = Required(rule, "expected", pointer, result);
      const Json* tolerance = Required(rule, "tolerance", pointer, result);
      if (expected != nullptr &&
          (!expected->is_number() ||
           !std::isfinite(expected->get<double>()))) {
        Add(result, pointer + "/expected", "must be finite and numeric");
      }
      if (tolerance != nullptr &&
          (!tolerance->is_number() ||
           !std::isfinite(tolerance->get<double>()) ||
           tolerance->get<double>() < 0.0)) {
        Add(result, pointer + "/tolerance", "must be finite and nonnegative");
      }
    }
  }
  if (document.contains("ranges") && document["ranges"].is_array()) {
    for (std::size_t index = 0; index < document["ranges"].size(); ++index) {
      const Json& rule = document["ranges"][index];
      const std::string pointer = "/ranges/" + std::to_string(index);
      if (!rule.is_object()) continue;
      const Json* minimum = Required(rule, "minimum", pointer, result);
      const Json* maximum = Required(rule, "maximum", pointer, result);
      if (minimum != nullptr &&
          (!minimum->is_number() ||
           !std::isfinite(minimum->get<double>()))) {
        Add(result, pointer + "/minimum", "must be finite and numeric");
      }
      if (maximum != nullptr &&
          (!maximum->is_number() ||
           !std::isfinite(maximum->get<double>()))) {
        Add(result, pointer + "/maximum", "must be finite and numeric");
      }
      if (minimum != nullptr && maximum != nullptr && minimum->is_number() &&
          maximum->is_number() && std::isfinite(minimum->get<double>()) &&
          std::isfinite(maximum->get<double>()) &&
          minimum->get<double>() > maximum->get<double>()) {
        Add(result, pointer, "minimum must not exceed maximum");
      }
    }
  }
  return result;
}

ComparisonResult CompareReplayReport(const Json& baseline, const Json& report) {
  const ValidationResult baseline_validation = ValidateReplayBaseline(baseline);
  if (!baseline_validation.Ok()) {
    throw std::invalid_argument("invalid baseline:\n" +
                                baseline_validation.Summary());
  }
  const ValidationResult report_validation = ValidateReplayReport(report);
  if (!report_validation.Ok()) {
    throw std::invalid_argument("invalid report:\n" +
                                report_validation.Summary());
  }
  ComparisonResult result;
  if (baseline.at("case_id") != report.at("case_id")) {
    Difference(result, "/case_id", "identity", baseline.at("case_id"),
               report.at("case_id"), nullptr, "case id differs");
  }
  if (baseline.at("case_manifest_sha256") !=
      report.at("case_manifest_sha256")) {
    Difference(result, "/case_manifest_sha256", "identity",
               baseline.at("case_manifest_sha256"),
               report.at("case_manifest_sha256"), nullptr,
               "case manifest digest differs");
  }
  for (const Json& rule : baseline.at("exact")) {
    const std::string pointer = rule.at("pointer");
    const Json* actual = Resolve(report, pointer);
    if (actual == nullptr) {
      Difference(result, pointer, "exact", rule.at("expected"), nullptr,
                 nullptr, "report value is missing");
    } else if (*actual != rule.at("expected")) {
      Difference(result, pointer, "exact", rule.at("expected"), *actual,
                 nullptr, "exact value differs");
    }
  }
  for (const Json& rule : baseline.at("absolute_tolerances")) {
    const std::string pointer = rule.at("pointer");
    const Json* actual = Resolve(report, pointer);
    if (actual == nullptr || !actual->is_number()) {
      Difference(result, pointer, "absolute_tolerance", rule.at("expected"),
                 actual == nullptr ? Json(nullptr) : *actual,
                 rule.at("tolerance"), "numeric report value is missing");
      continue;
    }
    const double expected = rule.at("expected").get<double>();
    const double candidate = actual->get<double>();
    const double tolerance = rule.at("tolerance").get<double>();
    if (!std::isfinite(candidate) || std::abs(candidate - expected) > tolerance)
      Difference(result, pointer, "absolute_tolerance", expected, candidate,
                 tolerance, "absolute error exceeds tolerance");
  }
  for (const Json& rule : baseline.at("ranges")) {
    const std::string pointer = rule.at("pointer");
    const Json* actual = Resolve(report, pointer);
    if (actual == nullptr || !actual->is_number()) {
      Difference(result, pointer, "range",
                 Json::array({rule.at("minimum"), rule.at("maximum")}),
                 actual == nullptr ? Json(nullptr) : *actual, nullptr,
                 "numeric report value is missing");
      continue;
    }
    const double candidate = actual->get<double>();
    const double minimum = rule.at("minimum").get<double>();
    const double maximum = rule.at("maximum").get<double>();
    if (!std::isfinite(candidate) || candidate < minimum || candidate > maximum)
      Difference(result, pointer, "range", Json::array({minimum, maximum}),
                 candidate, nullptr, "value is outside inclusive range");
  }
  return result;
}

}  // namespace open_lmm::test::replay
