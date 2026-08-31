#include "replay_contract.hpp"
#include "replay_input_lock.hpp"
#include "replay_sha256.hpp"

#include <open_lmm/server/runtime_client.hpp>

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
namespace replay = open_lmm::test::replay;
using Json = nlohmann::json;

namespace {

struct Arguments {
  fs::path case_manifest;
  fs::path data_root;
  fs::path config_root;
  fs::path output_root;
  fs::path report;
  std::string git_commit;
  std::string container_digest;
  bool git_dirty = false;
};

struct RunOutcome {
  Json report;
  bool expectations_met = false;
};

Arguments ParseArguments(int argc, char** argv) {
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    auto consume_path = [&](fs::path& destination) {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      destination = argv[index];
    };
    auto consume_text = [&](std::string& destination) {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      destination = argv[index];
    };
    if (option == "--case") consume_path(arguments.case_manifest);
    else if (option == "--data-root") consume_path(arguments.data_root);
    else if (option == "--config-root") consume_path(arguments.config_root);
    else if (option == "--output-root") consume_path(arguments.output_root);
    else if (option == "--report") consume_path(arguments.report);
    else if (option == "--git-commit") consume_text(arguments.git_commit);
    else if (option == "--container-digest")
      consume_text(arguments.container_digest);
    else if (option == "--git-dirty") arguments.git_dirty = true;
    else if (option == "--help") {
      std::cout
          << "usage: open_lmm_replay_runner --case CASE.json "
             "--data-root DATA --config-root CONFIG --output-root OUTPUT "
             "--report REPORT.json --git-commit COMMIT "
             "--container-digest SHA256 [--git-dirty]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.case_manifest.empty() || arguments.data_root.empty() ||
      arguments.config_root.empty() || arguments.output_root.empty() ||
      arguments.report.empty() || arguments.git_commit.empty() ||
      arguments.container_digest.empty()) {
    throw std::invalid_argument("all path and provenance arguments are required");
  }
  return arguments;
}

bool IsSafeRelativePath(const fs::path& path) {
  if (path.empty() || path.is_absolute() || path.has_root_name() ||
      path.has_root_directory() || path.lexically_normal() != path) {
    return false;
  }
  for (const auto& component : path) {
    if (component == "." || component == "..") return false;
  }
  return true;
}

std::string DigestText(const Json& value) {
  std::string digest = value.get<std::string>();
  if (digest.starts_with("sha256:")) digest.erase(0, 7);
  return digest;
}

void PrepareOutputRoot(const fs::path& output_root) {
  std::error_code error;
  if (!fs::exists(output_root, error)) fs::create_directories(output_root, error);
  if (error || !fs::is_directory(output_root, error)) {
    throw std::runtime_error("output root is not a directory: " +
                             output_root.string());
  }
  if (fs::directory_iterator(output_root, error) != fs::directory_iterator() ||
      error) {
    throw std::runtime_error("output root must be empty: " +
                             output_root.string());
  }
}

void Preflight(const Json& manifest, const Arguments& arguments) {
  replay::VerifyReplayInputs(manifest, arguments.data_root,
                             arguments.config_root);
  if (fs::exists(arguments.report)) {
    throw std::runtime_error("replay report path must not already exist: " +
                             arguments.report.string());
  }
  PrepareOutputRoot(arguments.output_root);
}

const char* StageName(open_lmm::StageId stage) {
  switch (stage) {
    case open_lmm::StageId::kDataLoad: return "DataLoad";
    case open_lmm::StageId::kAlignment: return "Alignment";
    case open_lmm::StageId::kMapUpdate: return "MapUpdate";
    case open_lmm::StageId::kSave: return "Save";
  }
  return "Unknown";
}

open_lmm::StageId ParseStage(const std::string& stage) {
  if (stage == "DataLoad") return open_lmm::StageId::kDataLoad;
  if (stage == "Alignment") return open_lmm::StageId::kAlignment;
  if (stage == "MapUpdate") return open_lmm::StageId::kMapUpdate;
  if (stage == "Save") return open_lmm::StageId::kSave;
  throw std::invalid_argument("unsupported runtime stage: " + stage);
}

open_lmm::ConfigDomain ParseConfigDomain(const std::string& domain) {
  using Domain = open_lmm::ConfigDomain;
  if (domain == "data_loader") return Domain::kDataLoader;
  if (domain == "loop_detector") return Domain::kLoopDetector;
  if (domain == "optimizer") return Domain::kOptimizer;
  if (domain == "dynamic_remover") return Domain::kDynamicRemover;
  if (domain == "map_save") return Domain::kMapSave;
  throw std::invalid_argument("unsupported config domain: " + domain);
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) throw std::runtime_error("failed to open config change document");
  std::string text((std::istreambuf_iterator<char>(input)),
                   std::istreambuf_iterator<char>());
  if (input.bad()) throw std::runtime_error("failed to read config change document");
  return text;
}

const char* ArtifactTypeName(open_lmm::ArtifactType type) {
  using Type = open_lmm::ArtifactType;
  switch (type) {
    case Type::kConfigSnapshot: return "config_snapshot";
    case Type::kAgentInput: return "agent_input";
    case Type::kRawData: return "raw_data";
    case Type::kDescriptorState: return "descriptor_state";
    case Type::kLoopCandidates: return "loop_candidates";
    case Type::kMapAlignment: return "map_alignment";
    case Type::kOptimizerState: return "optimizer_state";
    case Type::kOptimizedPoses: return "optimized_poses";
    case Type::kGlobalMap: return "global_map";
    case Type::kPoseFile: return "pose_file";
    case Type::kPcdFile: return "pcd_file";
    case Type::kProfileRecord: return "profile_record";
  }
  return "unknown";
}

const char* ArtifactStateName(open_lmm::ArtifactState state) {
  using State = open_lmm::ArtifactState;
  switch (state) {
    case State::kMissing: return "missing";
    case State::kReady: return "ready";
    case State::kStale: return "stale";
    case State::kFailed: return "failed";
  }
  return "unknown";
}

const char* RuntimeStatusName(open_lmm::RuntimeStatus status) {
  using Status = open_lmm::RuntimeStatus;
  switch (status) {
    case Status::kCreating: return "creating";
    case Status::kReady: return "ready";
    case Status::kRunning: return "running";
    case Status::kCancelling: return "cancelling";
    case Status::kFailedRecoverable: return "failed_recoverable";
    case Status::kFailedFatal: return "failed_fatal";
    case Status::kClosing: return "closing";
    case Status::kClosed: return "closed";
  }
  return "unknown";
}

const char* ErrorCodeName(open_lmm::Error::Code code) {
  using Code = open_lmm::Error::Code;
  switch (code) {
    case Code::kFileNotFound: return "file_not_found";
    case Code::kParseError: return "parse_error";
    case Code::kInvalidArgument: return "invalid_argument";
    case Code::kPluginLoadFailed: return "plugin_load_failed";
    case Code::kRegistrationFailed: return "registration_failed";
    case Code::kOptimizationFailed: return "optimization_failed";
    case Code::kIoError: return "io_error";
    case Code::kCancelled: return "cancelled";
    case Code::kAgentExcluded: return "agent_excluded";
  }
  return "unknown";
}

const char* EventTypeName(open_lmm::EventType type) {
  using Type = open_lmm::EventType;
  switch (type) {
    case Type::kJobQueued: return "job_queued";
    case Type::kJobStarted: return "job_started";
    case Type::kStageStarted: return "stage_started";
    case Type::kNodeStarted: return "node_started";
    case Type::kProgressUpdated: return "progress_updated";
    case Type::kArtifactCommitted: return "artifact_committed";
    case Type::kArtifactInvalidated: return "artifact_invalidated";
    case Type::kNodeFailed: return "node_failed";
    case Type::kStageCompleted: return "stage_completed";
    case Type::kStageFailed: return "stage_failed";
    case Type::kCancellationRequested: return "cancellation_requested";
    case Type::kAlignmentFeedbackRequested:
      return "alignment_feedback_requested";
    case Type::kAlignmentProposalAccepted:
      return "alignment_proposal_accepted";
    case Type::kAlignmentProposalRejected:
      return "alignment_proposal_rejected";
    case Type::kAlignmentFeedbackCancelled:
      return "alignment_feedback_cancelled";
    case Type::kAlignmentAgentExcluded: return "alignment_agent_excluded";
    case Type::kJobCompleted: return "job_completed";
    case Type::kJobCancelled: return "job_cancelled";
  }
  return "unknown";
}

Json EventJson(const open_lmm::ExecutionEvent& event) {
  Json output{{"sequence", event.sequence},
              {"type", EventTypeName(event.type)},
              {"stage", event.stage ? Json(StageName(*event.stage))
                                     : Json(nullptr)},
              {"agent", event.agent ? Json(event.agent->Value())
                                     : Json(nullptr)}};
  if (event.error) output["error_code"] = ErrorCodeName(event.error->code);
  return output;
}

Json ArtifactsJson(const open_lmm::RuntimeSnapshot& snapshot) {
  Json artifacts = Json::array();
  for (const auto& artifact : snapshot.pipeline.artifacts) {
    Json item{{"type", ArtifactTypeName(artifact.key.type)},
              {"agent", artifact.key.agent
                            ? Json(artifact.key.agent->Value())
                            : Json(nullptr)},
              {"state", ArtifactStateName(artifact.state)},
              {"revision", artifact.revision},
              {"producer", artifact.producer}};
    if (!artifact.external_path.empty()) {
      const fs::path path = artifact.external_path;
      const fs::path relative = path.lexically_relative(snapshot.output_directory);
      item["path"] = IsSafeRelativePath(relative) ? Json(relative.string())
                                                   : Json(nullptr);
    }
    artifacts.push_back(std::move(item));
  }
  std::sort(artifacts.begin(), artifacts.end(), [](const Json& lhs,
                                                   const Json& rhs) {
    return std::tie(lhs.at("type"), lhs.at("agent")) <
           std::tie(rhs.at("type"), rhs.at("agent"));
  });
  return artifacts;
}

Json VisualizationJson(open_lmm::RuntimeClient& client,
                       const std::vector<open_lmm::AgentId>& agents) {
  Json output = Json::object();
  for (const auto& agent : agents) {
    open_lmm::VisualizationQuery query;
    query.agent = agent;
    // Real-dataset replay baselines must observe map cardinality and bounds,
    // not only trajectory metadata. This deliberately requests the complete
    // runtime-defined voxel preview; tier size and the replay lane's resource
    // limit control its cost. A voxelized output is not treated as a hard peak
    // memory bound.
    query.include_points = true;
    auto snapshot = client.Visualization(query);
    if (!snapshot) {
      output[agent.Value()] =
          {{"available", false},
           {"error_code", ErrorCodeName(snapshot.GetError().code)}};
      continue;
    }
    const auto& value = snapshot.Value();
    Json poses = Json::array();
    for (const auto& pose : value.poses) {
      Eigen::Quaternionf rotation(pose.transform.linear());
      if (rotation.w() < 0.0F ||
          (rotation.w() == 0.0F &&
           std::tie(rotation.x(), rotation.y(), rotation.z()) <
               std::tuple{0.0F, 0.0F, 0.0F})) {
        rotation.coeffs() *= -1.0F;
      }
      poses.push_back(
          {{"index", pose.index},
           {"translation",
            Json::array({pose.transform.translation().x(),
                         pose.transform.translation().y(),
                         pose.transform.translation().z()})},
           {"quaternion_xyzw",
            Json::array({rotation.x(), rotation.y(), rotation.z(),
                         rotation.w()})}});
    }
    Json edges = Json::array();
    for (const auto& edge : value.edges) {
      edges.push_back(
          {{"from_agent", edge.from_agent.Value()},
           {"from_index", edge.from_index},
           {"to_agent", edge.to_agent.Value()},
           {"to_index", edge.to_index},
           {"type", static_cast<unsigned>(edge.type)}});
    }
    std::sort(edges.begin(), edges.end());
    output[agent.Value()] =
        {{"available", true},
         {"revision", value.revision},
         {"pose_count", value.poses.size()},
         {"poses", std::move(poses)},
         {"edges", std::move(edges)},
         {"has_bounds", value.has_bounds},
         {"min_bound",
          Json::array({value.min_bound.x(), value.min_bound.y(),
                       value.min_bound.z()})},
         {"max_bound",
          Json::array({value.max_bound.x(), value.max_bound.y(),
                       value.max_bound.z()})},
         {"displayed_point_count", value.displayed_point_count},
         {"source_point_count", value.source_point_count}};
  }
  return output;
}

std::vector<open_lmm::AgentId> AgentIds(const Json& manifest) {
  std::vector<open_lmm::AgentId> agents;
  for (const Json& item : manifest.at("dataset").at("agents")) {
    auto parsed = open_lmm::AgentId::Parse(item.at("id").get<std::string>());
    if (!parsed) {
      throw std::runtime_error("invalid runtime agent id in replay manifest");
    }
    agents.push_back(std::move(parsed).Value());
  }
  return agents;
}

Json PatchedRootConfig(const Json& manifest, const Arguments& arguments) {
  const fs::path root_path =
      arguments.config_root /
      manifest.at("config").at("root").get<std::string>();
  Json root = replay::LoadJsonFile(root_path);
  if (!root.contains("directory") || !root.at("directory").is_object()) {
    throw std::runtime_error("root config has no directory object");
  }
  Json agent_names = Json::array();
  for (const Json& agent : manifest.at("dataset").at("agents")) {
    agent_names.push_back(agent.at("id"));
  }
  root["directory"]["root_dir_path"] =
      fs::weakly_canonical(arguments.data_root).string();
  root["directory"]["sub_dir_list"] = std::move(agent_names);
  root["directory"]["root_save_dir"] =
      fs::weakly_canonical(arguments.output_root).string();
  return root;
}

void WriteReport(const fs::path& path, const Json& report) {
  if (!path.parent_path().empty()) fs::create_directories(path.parent_path());
  std::ofstream output(path);
  if (!output) throw std::runtime_error("failed to open replay report output");
  output << report.dump(2) << '\n';
  if (!output) throw std::runtime_error("failed to write replay report");
}

class EventCollector {
 public:
  void Append(const open_lmm::ExecutionEvent& event) {
    {
      std::lock_guard lock(mutex_);
      events_.push_back(event);
    }
    changed_.notify_all();
  }

  std::size_t Mark() const {
    std::lock_guard lock(mutex_);
    return events_.size();
  }

  Json Since(std::size_t mark) {
    std::lock_guard lock(mutex_);
    if (mark > events_.size()) {
      throw std::logic_error("event collector mark is out of range");
    }
    std::sort(events_.begin() + static_cast<std::ptrdiff_t>(mark),
              events_.end(), [](const auto& lhs, const auto& rhs) {
                return lhs.sequence < rhs.sequence;
              });
    Json result = Json::array();
    for (std::size_t index = mark; index < events_.size(); ++index) {
      result.push_back(EventJson(events_[index]));
    }
    return result;
  }

  void WaitForTerminal(open_lmm::JobHandle job) {
    std::unique_lock lock(mutex_);
    const bool completed = changed_.wait_for(
        lock, std::chrono::seconds(30), [&] {
          return std::any_of(events_.begin(), events_.end(), [&](const auto& event) {
            return event.job_id == job.value &&
                   (event.type == open_lmm::EventType::kJobCompleted ||
                    event.type == open_lmm::EventType::kJobCancelled);
          });
        });
    if (!completed) {
      throw std::runtime_error("timed out waiting for terminal replay event");
    }
  }

  std::optional<open_lmm::Error> ErrorFor(open_lmm::JobHandle job) const {
    std::lock_guard lock(mutex_);
    for (auto event = events_.rbegin(); event != events_.rend(); ++event) {
      if (event->job_id == job.value && event->error) return event->error;
    }
    return std::nullopt;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable changed_;
  std::vector<open_lmm::ExecutionEvent> events_;
};

Json InitialReport(const Json& manifest, const Arguments& arguments,
                   const std::vector<open_lmm::AgentId>& agents) {
  Json agent_names = Json::array();
  for (const auto& agent : agents) agent_names.push_back(agent.Value());
  Json report = Json::object();
  report["schema_version"] = 1;
  report["case_id"] = manifest.at("case_id");
  report["case_manifest_sha256"] =
      replay::Sha256File(arguments.case_manifest);
  report["dataset_sha256"] =
      DigestText(manifest.at("dataset").at("bundle_sha256"));
  report["config_sha256"] =
      replay::Sha256(replay::CanonicalJson(manifest.at("config")));
  report["git"] =
      {{"commit", arguments.git_commit}, {"dirty", arguments.git_dirty}};
  report["environment"] =
      {{"compiler", __VERSION__},
       {"container_digest", arguments.container_digest},
       {"max_agent_tasks", 1}};
  report["agents"] = std::move(agent_names);
  report["steps"] = Json::array();
  report["health"] = Json::object();
  report["metrics"] = Json::object();
  report["artifacts"] = Json::array();
  report["diagnostics"] = Json::object();
  report["close_result"] = "failed";
  return report;
}

Json StepReport(const std::string& name,
                const std::optional<open_lmm::Error>& error,
                std::uint64_t before_revision,
                const open_lmm::RuntimeSnapshot& after, Json events) {
  Json report = Json::object();
  report["stage"] = name;
  report["result"] = error ? "failed" : "succeeded";
  report["error_code"] =
      error ? Json(ErrorCodeName(error->code)) : Json(nullptr);
  report["revision_before"] = before_revision;
  report["revision_after"] = after.pipeline.runtime_revision;
  report["artifacts"] = ArtifactsJson(after);
  report["events"] = std::move(events);
  return report;
}

struct OperationOutcome {
  std::optional<open_lmm::Error> error;
  std::optional<open_lmm::JobHandle> job;
};

OperationOutcome ExecuteWorkflowOperation(
    open_lmm::RuntimeClient& client, const Json& step,
    const open_lmm::RuntimeSnapshot& before, const Arguments& arguments) {
  const std::string name = step.at("stage");
  if (name == "ApplyConfig") {
    const Json& change = step.at("config_change");
    open_lmm::ConfigCandidate candidate;
    candidate.domain =
        ParseConfigDomain(change.at("domain").get<std::string>());
    candidate.document_json =
        ReadText(arguments.config_root /
                 change.at("document").get<std::string>());
    if (change.contains("selected_document")) {
      candidate.selected_document =
          change.at("selected_document").get<std::string>();
    }
    const open_lmm::ExpectedRevision expected{
        before.pipeline.runtime_revision, before.pipeline.config_revision};
    auto applied = client.ApplyConfig(candidate, expected);
    if (!applied) return {applied.GetError(), std::nullopt};
    return {};
  }

  open_lmm::ExecutionRequest execution;
  execution.kind = open_lmm::ExecutionRequestKind::kStage;
  execution.stage = ParseStage(name);
  auto submitted = client.Submit(execution);
  if (!submitted) return {submitted.GetError(), std::nullopt};
  const open_lmm::JobHandle job = submitted.Value();
  auto waited = client.Wait(submitted.Value());
  if (!waited) return {waited.GetError(), job};
  return {std::nullopt, job};
}

std::optional<open_lmm::Error> ReconcileOperationError(
    OperationOutcome outcome, const open_lmm::RuntimeSnapshot& snapshot,
    const EventCollector& events) {
  if (!outcome.error || !outcome.job) return outcome.error;
  for (auto event = snapshot.pipeline.recent_events.rbegin();
       event != snapshot.pipeline.recent_events.rend(); ++event) {
    if (event->job_id == outcome.job->value && event->error) {
      return event->error;
    }
  }
  if (auto event_error = events.ErrorFor(*outcome.job)) return event_error;
  return outcome.error;
}

struct FailureObservation {
  bool failed = false;
  std::string stage;
  std::string code;
  std::uint64_t revision_before = 0;
  std::uint64_t revision_after = 0;
  std::optional<std::string> agent;
};

void RecordFailure(const open_lmm::Error& error, const std::string& stage,
                   std::uint64_t before, std::uint64_t after,
                   FailureObservation& observation, Json& diagnostics) {
  observation.failed = true;
  observation.stage = stage;
  observation.code = ErrorCodeName(error.code);
  observation.revision_before = before;
  observation.revision_after = after;
  if (error.context.agent) observation.agent = error.context.agent->Value();
  diagnostics["failure_context"] =
      {{"severity", error.severity == open_lmm::Error::Severity::kFatalRuntime
                        ? "fatal_runtime"
                        : "recoverable"},
       {"stage", error.context.stage},
       {"node", error.context.node},
       {"agent", observation.agent ? Json(*observation.agent)
                                    : Json(nullptr)}};
}

bool ExpectationsMet(const Json& expected,
                     const FailureObservation& observation,
                     bool close_succeeded) {
  if (expected.at("result") == "success") {
    return !observation.failed && close_succeeded;
  }
  const Json& failure = expected.at("failure");
  bool matched =
      observation.failed &&
      observation.stage == failure.at("stage").get<std::string>() &&
      observation.code == failure.at("error_code").get<std::string>() &&
      (!failure.at("revision_unchanged").get<bool>() ||
       observation.revision_before == observation.revision_after) &&
      (!failure.at("close_succeeds").get<bool>() || close_succeeded);
  if (failure.contains("agent")) {
    matched = matched && observation.agent &&
              *observation.agent == failure.at("agent").get<std::string>();
  }
  return matched;
}

RunOutcome RunCase(const Json& manifest, const Arguments& arguments) {
  const std::vector<open_lmm::AgentId> agents = AgentIds(manifest);
  Json report = InitialReport(manifest, arguments, agents);
  EventCollector events;
  open_lmm::RuntimeClient client(1);

  const fs::path root_path =
      arguments.config_root /
      manifest.at("config").at("root").get<std::string>();
  open_lmm::BootstrapRequest request;
  request.config_directory = root_path.parent_path();
  request.label = "replay:" + manifest.at("case_id").get<std::string>();
  request.output_root = arguments.output_root;
  open_lmm::ConfigCandidate root_candidate;
  root_candidate.domain = open_lmm::ConfigDomain::kGlobal;
  root_candidate.document_json = PatchedRootConfig(manifest, arguments).dump();

  const auto opened = client.Open(request, root_candidate);
  if (!opened) {
    FailureObservation failure;
    RecordFailure(opened.GetError(), "Open", 0, 0, failure,
                  report["diagnostics"]);
    report["steps"].push_back(
        {{"stage", "Open"},
         {"result", "failed"},
         {"error_code", ErrorCodeName(opened.GetError().code)},
         {"revision_before", 0},
         {"revision_after", 0},
         {"artifacts", Json::array()},
         {"events", Json::array()}});
    report["health"] = {{"state", "closed"}};
    const auto closed = client.Close();
    report["close_result"] = closed ? "succeeded" : "failed";
    const bool expectations_met = ExpectationsMet(
        manifest.at("expected"), failure, closed.IsOk());
    report["diagnostics"]["expectations_met"] = expectations_met;
    return {std::move(report), expectations_met};
  }

  auto subscribed = client.SubscribeEvents([&](const auto& event) {
    events.Append(event);
  });
  if (!subscribed) {
    throw std::runtime_error("failed to subscribe to runtime events: " +
                             subscribed.GetError().Message());
  }
  auto subscription = std::move(subscribed).Value();

  auto initial = client.Snapshot();
  if (!initial) throw std::runtime_error("snapshot failed after Open");
  report["steps"].push_back(
      {{"stage", "Open"},
       {"result", "succeeded"},
       {"error_code", nullptr},
       {"revision_before", 0},
       {"revision_after", initial.Value().pipeline.runtime_revision},
       {"artifacts", ArtifactsJson(initial.Value())},
       {"events", Json::array()}});

  FailureObservation failure;
  for (const Json& step : manifest.at("workflow")) {
    const std::string name = step.at("stage");
    auto before = client.Snapshot();
    if (!before) throw std::runtime_error("snapshot failed before " + name);
    const std::uint64_t before_revision =
        before.Value().pipeline.runtime_revision;
    const std::size_t event_start = events.Mark();
    auto operation =
        ExecuteWorkflowOperation(client, step, before.Value(), arguments);
    if (operation.job) events.WaitForTerminal(*operation.job);
    auto after = client.Snapshot();
    if (!after) throw std::runtime_error("snapshot failed after " + name);
    const auto error =
        ReconcileOperationError(std::move(operation), after.Value(), events);
    report["steps"].push_back(StepReport(
        name, error, before_revision, after.Value(), events.Since(event_start)));
    if (error) {
      RecordFailure(*error, name, before_revision,
                    after.Value().pipeline.runtime_revision, failure,
                    report["diagnostics"]);
      break;
    }
  }

  auto final_snapshot = client.Snapshot();
  if (!final_snapshot) throw std::runtime_error("final snapshot failed");
  report["health"] = {
      {"state", RuntimeStatusName(final_snapshot.Value().state)}};
  report["artifacts"] = ArtifactsJson(final_snapshot.Value());
  report["metrics"]["visualization"] = VisualizationJson(client, agents);
  report["diagnostics"]["output_directory"] =
      final_snapshot.Value().output_directory.filename().string();

  const auto closed = client.Close();
  report["close_result"] = closed ? "succeeded" : "failed";

  const bool expectations_met =
      ExpectationsMet(manifest.at("expected"), failure, closed.IsOk());
  report["diagnostics"]["expectations_met"] = expectations_met;
  return {std::move(report), expectations_met};
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = ParseArguments(argc, argv);
    const Json manifest = replay::LoadJsonFile(arguments.case_manifest);
    const auto validation = replay::ValidateCaseManifest(manifest);
    if (!validation.Ok()) {
      throw std::invalid_argument("invalid case manifest:\n" +
                                  validation.Summary());
    }
    Preflight(manifest, arguments);
    RunOutcome outcome = RunCase(manifest, arguments);
    const auto report_validation =
        replay::ValidateReplayReport(outcome.report);
    if (!report_validation.Ok()) {
      throw std::runtime_error("runner produced an invalid report:\n" +
                               report_validation.Summary());
    }
    WriteReport(arguments.report, outcome.report);
    std::cout << "replay expectations="
              << (outcome.expectations_met ? "PASS" : "FAIL") << '\n';
    return outcome.expectations_met ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid replay request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "replay failed: " << error.what() << '\n';
    return 2;
  }
}
