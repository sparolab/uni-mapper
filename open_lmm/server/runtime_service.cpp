#include "runtime_service.hpp"

#include <open_lmm/server/map_server.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>

#include <algorithm>
#include <array>
#include <condition_variable>
#include <iomanip>
#include <random>
#include <sstream>

namespace open_lmm {
namespace fs = std::filesystem;
namespace {

std::string RandomHex128() {
  std::random_device source;
  std::array<unsigned char, 16> bytes{};
  for (std::size_t offset = 0; offset < bytes.size(); offset += 4) {
    const uint32_t value = source();
    for (std::size_t byte = 0; byte < 4; ++byte) {
      bytes[offset + byte] = static_cast<unsigned char>(value >> (byte * 8));
    }
  }
  bytes[6] = static_cast<unsigned char>((bytes[6] & 0x0fU) | 0x40U);
  bytes[8] = static_cast<unsigned char>((bytes[8] & 0x3fU) | 0x80U);
  std::ostringstream output;
  output << std::hex << std::setfill('0');
  for (std::size_t index = 0; index < bytes.size(); ++index) {
    if (index == 4 || index == 6 || index == 8 || index == 10) output << '-';
    output << std::setw(2) << static_cast<unsigned int>(bytes[index]);
  }
  return output.str();
}

}  // namespace

struct RuntimeService::RuntimeSession {
  explicit RuntimeSession(SessionId session_id) : id(std::move(session_id)) {}
  SessionId id;
  std::string label;
  std::shared_ptr<const BootstrapConfigSnapshot> bootstrap_config;
  fs::path output_directory;
  std::shared_ptr<StageRuntimePort> port;
  std::shared_ptr<PipelineController> controller;
  mutable std::mutex mutex;
  std::condition_variable idle;
  std::size_t commands_in_progress = 0;
  RuntimeSessionState state = RuntimeSessionState::kCreating;
};

RuntimeService::RuntimeService(std::size_t maximum_sessions,
                               PortFactory port_factory)
    : RuntimeService(ResourceBudget{maximum_sessions, 2, 2,
                                    4ULL * 1024ULL * 1024ULL * 1024ULL},
                     std::move(port_factory)) {}

RuntimeService::RuntimeService(ResourceBudget budget,
                               PortFactory port_factory)
    : governor_(std::make_shared<ResourceGovernor>(budget)),
      port_factory_(std::move(port_factory)) {
  if (!port_factory_) {
    port_factory_ = [governor = governor_](
                          const BootstrapConfigSnapshot& bootstrap,
                          const fs::path& output_directory)
        -> Result<std::shared_ptr<StageRuntimePort>> {
      auto port = std::make_shared<MapServer>(bootstrap,
                                              output_directory, governor);
      auto ready = port->ValidateReady();
      if (!ready) {
        return Result<std::shared_ptr<StageRuntimePort>>::Failure(
            ready.GetError());
      }
      return Result<std::shared_ptr<StageRuntimePort>>::Ok(std::move(port));
    };
  }
}

RuntimeService::~RuntimeService() {
  for (const auto& id : SessionIds()) {
    (void)CloseSession(id, CloseMode::kCancelAndWait);
  }
}

Result<SessionId> RuntimeService::GenerateSessionId() {
  return SessionId::Parse(RandomHex128());
}

std::string RuntimeService::GenerateOutputNamespace() {
  std::string value = RandomHex128();
  value.erase(std::remove(value.begin(), value.end(), '-'), value.end());
  return "runtime-session-" + value;
}

Result<SessionId> RuntimeService::CreateSession(
    const BootstrapRequest& request) {
  if (request.config_directory.empty()) {
    return Result<SessionId>::Failure(
        Error::InvalidArgument("session config directory must be non-empty"));
  }
  if (!governor_->TryAcquireSession()) {
    return Result<SessionId>::Failure(
        Error::InvalidArgument("session resource admission limit reached"));
  }
  struct AdmissionRollback {
    ResourceGovernor& governor;
    bool committed = false;
    ~AdmissionRollback() {
      if (!committed) governor.ReleaseSession();
    }
  } admission{*governor_};

  auto loaded = LoadBootstrapConfig(request.config_directory);
  if (!loaded) return Result<SessionId>::Failure(loaded.GetError());
  auto bootstrap = std::make_shared<const BootstrapConfigSnapshot>(
      std::move(loaded).Value());
  fs::path output_root = request.output_root.value_or(bootstrap->OutputRoot());
  if (output_root.empty()) {
    return Result<SessionId>::Failure(
        Error::InvalidArgument("session output root must be non-empty"));
  }

  for (int attempt = 0; attempt < 8; ++attempt) {
    auto generated = GenerateSessionId();
    if (!generated) return generated;
    const SessionId id = generated.Value();
    const fs::path output_directory = output_root / GenerateOutputNamespace();
    {
      std::lock_guard lock(registry_mutex_);
      if (sessions_.contains(id)) continue;
    }
    auto created = port_factory_(*bootstrap, output_directory);
    if (!created) {
      return Result<SessionId>::Failure(created.GetError());
    }
    auto session = std::make_shared<RuntimeSession>(id);
    session->label = request.label;
    session->bootstrap_config = bootstrap;
    session->output_directory = output_directory;
    session->port = std::move(created).Value();
    session->controller =
        std::make_shared<PipelineController>(session->port);
    session->state = RuntimeSessionState::kReady;
    {
      std::lock_guard lock(registry_mutex_);
      if (sessions_.contains(id)) continue;
      sessions_.emplace(id, session);
    }
    admission.committed = true;
    return Result<SessionId>::Ok(id);
  }
  return Result<SessionId>::Failure(
      Error::InvalidArgument("failed to allocate a unique SessionId"));
}

Result<std::shared_ptr<RuntimeService::RuntimeSession>> RuntimeService::lookup(
    const SessionId& session_id) const {
  std::lock_guard lock(registry_mutex_);
  const auto found = sessions_.find(session_id);
  if (found == sessions_.end()) {
    return Result<std::shared_ptr<RuntimeSession>>::Failure(
        Error::InvalidArgument("unknown SessionId: " + session_id.Value()));
  }
  return Result<std::shared_ptr<RuntimeSession>>::Ok(found->second);
}

bool RuntimeService::IsActive(JobState state) {
  return state == JobState::kQueued ||
         state == JobState::kWaitingForDependency ||
         state == JobState::kRunning || state == JobState::kCancelling ||
         state == JobState::kWaitingForAlignmentFeedback;
}

RuntimeSessionState RuntimeService::DeriveState(
    const RuntimeSession& session, const PipelineSnapshot& pipeline) {
  if (session.state == RuntimeSessionState::kClosing ||
      session.state == RuntimeSessionState::kClosed) {
    return session.state;
  }
  if (!pipeline.job) return RuntimeSessionState::kReady;
  switch (pipeline.job->state) {
    case JobState::kQueued:
    case JobState::kWaitingForDependency:
    case JobState::kRunning:
    case JobState::kWaitingForAlignmentFeedback:
      return RuntimeSessionState::kRunning;
    case JobState::kCancelling: return RuntimeSessionState::kCancelling;
    case JobState::kSucceeded:
    case JobState::kCancelled: return RuntimeSessionState::kReady;
    case JobState::kFailed:
      for (auto event = pipeline.recent_events.rbegin();
           event != pipeline.recent_events.rend(); ++event) {
        if (event->job_id != pipeline.job->id || !event->error) continue;
        return event->error->severity == Error::Severity::kFatalSession
                   ? RuntimeSessionState::kFailedFatal
                   : RuntimeSessionState::kFailedRecoverable;
      }
      return RuntimeSessionState::kFailedRecoverable;
  }
  return RuntimeSessionState::kFailedFatal;
}

Result<JobId> RuntimeService::Submit(const SessionId& session_id,
                                     const ExecutionRequest& request) {
  auto found = lookup(session_id);
  if (!found) return Result<JobId>::Failure(found.GetError());
  auto session = found.Value();
  {
    std::lock_guard lock(session->mutex);
    const auto pipeline = session->controller->Snapshot();
    session->state = DeriveState(*session, pipeline);
    if (session->state != RuntimeSessionState::kReady &&
        session->state != RuntimeSessionState::kFailedRecoverable) {
      return Result<JobId>::Failure(
          Error::InvalidArgument("session is not ready for submission"));
    }
    session->state = RuntimeSessionState::kRunning;
    ++session->commands_in_progress;
  }
  Result<uint64_t> submitted = Result<uint64_t>::Failure(
      Error::InvalidArgument("invalid execution request"));
  switch (request.kind) {
    case ExecutionRequestKind::kRunAll:
      submitted = session->controller->SubmitRunAll();
      break;
    case ExecutionRequestKind::kStage:
      if (request.stage) {
        submitted = session->controller->SubmitStage(*request.stage);
      }
      break;
    case ExecutionRequestKind::kNode:
      if (request.node) {
        submitted = session->controller->SubmitNode(*request.node,
                                                     request.agent);
      }
      break;
    case ExecutionRequestKind::kOptimizeThrough:
      if (request.agent) {
        submitted = session->controller->SubmitOptimizeThrough(*request.agent);
      }
      break;
  }
  {
    std::lock_guard lock(session->mutex);
    --session->commands_in_progress;
    if (!submitted && session->state != RuntimeSessionState::kClosing) {
      session->state = RuntimeSessionState::kReady;
    }
    session->idle.notify_all();
  }
  if (!submitted) return Result<JobId>::Failure(submitted.GetError());
  return Result<JobId>::Ok(submitted.Value());
}

Result<void> RuntimeService::Cancel(const SessionId& session_id,
                                    JobId job_id) {
  auto found = lookup(session_id);
  if (!found) return Result<void>::Failure(found.GetError());
  auto session = found.Value();
  {
    std::lock_guard lock(session->mutex);
    if (session->state == RuntimeSessionState::kClosing ||
        session->state == RuntimeSessionState::kClosed) {
      return Result<void>::Failure(Error::InvalidArgument("session is closing"));
    }
    ++session->commands_in_progress;
  }
  auto cancelled = session->controller->Cancel(job_id);
  {
    std::lock_guard lock(session->mutex);
    --session->commands_in_progress;
    if (cancelled) session->state = RuntimeSessionState::kCancelling;
    session->idle.notify_all();
  }
  return cancelled;
}

Result<RuntimeSessionSnapshot> RuntimeService::Snapshot(
    const SessionId& session_id) const {
  auto found = lookup(session_id);
  if (!found) {
    return Result<RuntimeSessionSnapshot>::Failure(found.GetError());
  }
  auto session = found.Value();
  const auto pipeline = session->controller->Snapshot();
  std::lock_guard lock(session->mutex);
  session->state = DeriveState(*session, pipeline);
  return Result<RuntimeSessionSnapshot>::Ok(
      {session->id, session->label, session->state,
       session->output_directory, pipeline});
}

Result<void> RuntimeService::CloseSession(const SessionId& session_id,
                                          CloseMode mode) {
  auto found = lookup(session_id);
  if (!found) return Result<void>::Failure(found.GetError());
  auto session = found.Value();
  if (session->controller->IsInEventCallback()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "cannot close a session from its event callback"));
  }
  std::optional<JobSnapshot> active_job;
  const auto initial_pipeline = session->controller->Snapshot();
  {
    std::unique_lock lock(session->mutex);
    session->state = DeriveState(*session, initial_pipeline);
    if ((session->state == RuntimeSessionState::kRunning ||
         session->state == RuntimeSessionState::kCancelling ||
         session->commands_in_progress != 0) &&
        mode == CloseMode::kRejectIfRunning) {
      return Result<void>::Failure(
          Error::InvalidArgument("session has an active job"));
    }
    session->state = RuntimeSessionState::kClosing;
    session->idle.wait(lock, [&] { return session->commands_in_progress == 0; });
  }
  const auto pipeline = session->controller->Snapshot();
  if (pipeline.job && IsActive(pipeline.job->state)) active_job = pipeline.job;
  if (active_job) {
    if (active_job->state != JobState::kCancelling) {
      // Completion may win this race. Wait() below remains the authoritative
      // terminal-journal barrier in either case.
      (void)session->controller->Cancel(active_job->id);
    }
    (void)session->controller->Wait(active_job->id);
  }
  // Wait() is intentionally a terminal-journal barrier, not a callback
  // completion barrier. Keep the session queryable until every already
  // committed event callback has left user code.
  auto callbacks_drained = session->controller->WaitForEventCallbacks();
  if (!callbacks_drained) return callbacks_drained;
  {
    std::lock_guard lock(session->mutex);
    session->state = RuntimeSessionState::kClosed;
  }
  {
    std::lock_guard lock(registry_mutex_);
    const auto current = sessions_.find(session_id);
    if (current != sessions_.end() && current->second == session) {
      sessions_.erase(current);
      governor_->ReleaseSession();
    }
  }
  return Result<void>::Ok();
}

Result<ExecutionEventSubscription> RuntimeService::SubscribeEvents(
    const SessionId& session_id,
    std::function<void(const SessionExecutionEvent&)> callback) {
  auto found = lookup(session_id);
  if (!found) {
    return Result<ExecutionEventSubscription>::Failure(found.GetError());
  }
  if (!callback) {
    return Result<ExecutionEventSubscription>::Failure(
        Error::InvalidArgument("event callback must be non-empty"));
  }
  auto subscription = found.Value()->controller->SubscribeEvents(
      [session_id, callback = std::move(callback)](const ExecutionEvent& event) {
        callback({session_id, event});
      });
  return Result<ExecutionEventSubscription>::Ok(std::move(subscription));
}

std::vector<SessionId> RuntimeService::SessionIds() const {
  std::vector<SessionId> result;
  std::lock_guard lock(registry_mutex_);
  result.reserve(sessions_.size());
  for (const auto& [id, session] : sessions_) {
    (void)session;
    result.push_back(id);
  }
  return result;
}

}  // namespace open_lmm
