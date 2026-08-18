#include <open_lmm/gui/gui_controller_bridge.hpp>

#include <map>
#include <mutex>

namespace open_lmm {
namespace {
template <typename T>
Result<T> RuntimeExpired() {
  return Result<T>::Failure(Error::InvalidArgument("runtime client expired"));
}

struct BridgeJobs {
  struct PendingSubmission {
    uint64_t token = 0;
    SessionId expected_session;
    std::optional<BoundJob> observed;
  };

  uint64_t Reserve(const SessionId& session) {
    std::lock_guard lock(mutex);
    const uint64_t token = next_token++;
    pending = PendingSubmission{token, session, std::nullopt};
    return token;
  }

  void RetireSessionLocked(const SessionId& session) {
    for (auto it = jobs.begin(); it != jobs.end();) {
      if (it->second.session_id == session) {
        retired.insert_or_assign(it->first, it->second);
        it = jobs.erase(it);
      } else {
        ++it;
      }
    }
    while (retired.size() > 256) retired.erase(retired.begin());
  }

  std::optional<uint64_t> FindTokenLocked(const BoundJob& job) const {
    for (const auto& [token, candidate] : jobs) {
      if (candidate.session_id == job.session_id &&
          candidate.job_id == job.job_id) return token;
    }
    for (const auto& [token, candidate] : retired) {
      if (candidate.session_id == job.session_id &&
          candidate.job_id == job.job_id) return token;
    }
    return std::nullopt;
  }

  uint64_t EnsureLocked(const BoundJob& job) {
    if (auto token = FindTokenLocked(job)) return *token;
    if (pending && pending->expected_session == job.session_id &&
        !pending->observed) {
      pending->observed = job;
      RetireSessionLocked(job.session_id);
      jobs.insert_or_assign(pending->token, job);
      return pending->token;
    }
    RetireSessionLocked(job.session_id);
    const uint64_t token = next_token++;
    jobs.insert_or_assign(token, job);
    return token;
  }

  template <typename Submit>
  Result<uint64_t> SubmitBound(const SessionId& expected_session,
                               Submit&& submit) {
    std::lock_guard submission_lock(submission_mutex);
    const uint64_t reserved = Reserve(expected_session);
    auto submitted = submit();
    std::lock_guard lock(mutex);
    if (!submitted) {
      jobs.erase(reserved);
      pending.reset();
      return Result<uint64_t>::Failure(submitted.GetError());
    }
    const BoundJob& bound = submitted.Value();
    uint64_t token = reserved;
    if (auto existing = FindTokenLocked(bound)) {
      token = *existing;
    } else {
      RetireSessionLocked(bound.session_id);
      jobs.insert_or_assign(reserved, bound);
    }
    if (token != reserved) jobs.erase(reserved);
    pending.reset();
    return Result<uint64_t>::Ok(token);
  }
  std::optional<BoundJob> Find(JobId id) const {
    std::lock_guard lock(mutex);
    const auto found = jobs.find(id);
    return found == jobs.end() ? std::nullopt
                               : std::optional<BoundJob>(found->second);
  }
  void Clear() {
    std::lock_guard submission_lock(submission_mutex);
    std::lock_guard lock(mutex);
    jobs.clear();
    retired.clear();
    pending.reset();
  }
  ExecutionEvent Translate(const SessionExecutionEvent& source) {
    ExecutionEvent translated = source.event;
    if (source.event.job_id == 0) return translated;
    std::lock_guard lock(mutex);
    translated.job_id = EnsureLocked({source.session_id, source.event.job_id});
    return translated;
  }
  PipelineSnapshot TranslateSnapshot(const SessionId& session,
                                     PipelineSnapshot snapshot) {
    for (auto& event : snapshot.recent_events) {
      event = Translate({session, std::move(event)});
    }
    if (snapshot.job && snapshot.job->id != 0) {
      std::lock_guard lock(mutex);
      snapshot.job->id = EnsureLocked({session, snapshot.job->id});
    }
    return snapshot;
  }
  std::mutex submission_mutex;
  mutable std::mutex mutex;
  uint64_t next_token = 1;
  std::map<uint64_t, BoundJob> jobs;
  std::map<uint64_t, BoundJob> retired;
  std::optional<PendingSubmission> pending;
};
}  // namespace

GuiServices MakeGuiServices(
    const std::shared_ptr<RuntimeSessionClient>& session,
    std::string config_file_path) {
  std::weak_ptr<RuntimeSessionClient> weak_session = session;
  auto jobs = std::make_shared<BridgeJobs>();
  GuiServices services;
  services.config_file_path = std::move(config_file_path);
  services.submit_run_all = [weak_session, jobs] {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<uint64_t>();
    const SessionId session = locked->CurrentSession();
    return jobs->SubmitBound(session,
                             [&] { return locked->Submit(ExecutionRequest{}); });
  };
  services.submit_stage = [weak_session, jobs](StageId stage) {
    auto locked = weak_session.lock();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kStage;
    request.stage = stage;
    if (!locked) return RuntimeExpired<uint64_t>();
    const SessionId session = locked->CurrentSession();
    return jobs->SubmitBound(session, [&] { return locked->Submit(request); });
  };
  services.submit_node = [weak_session, jobs](NodeId node,
                                        std::optional<AgentId> agent) {
    auto locked = weak_session.lock();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kNode;
    request.node = node;
    request.agent = std::move(agent);
    if (!locked) return RuntimeExpired<uint64_t>();
    const SessionId session = locked->CurrentSession();
    return jobs->SubmitBound(session, [&] { return locked->Submit(request); });
  };
  services.submit_optimize_through = [weak_session, jobs](AgentId agent) {
    auto locked = weak_session.lock();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kOptimizeThrough;
    request.agent = std::move(agent);
    if (!locked) return RuntimeExpired<uint64_t>();
    const SessionId session = locked->CurrentSession();
    return jobs->SubmitBound(session, [&] { return locked->Submit(request); });
  };
  services.cancel_job = [weak_session, jobs](uint64_t job_id) {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<void>();
    auto bound = jobs->Find(job_id);
    return bound ? locked->Cancel(*bound)
                 : Result<void>::Failure(
                       Error::InvalidArgument("GUI job token is unknown"));
  };
  services.apply_config = [weak_session](SessionId expected_session,
                                         ConfigCandidate candidate,
                                         ExpectedRevision expected) {
    auto locked = weak_session.lock();
    return locked
               ? locked->ApplyConfig(expected_session, candidate, expected)
               : RuntimeExpired<ConfigApplyReceipt>();
  };
  services.replace_session = [weak_session, jobs,
                              config_file_path = services.config_file_path](
                                 ConfigCandidate root_candidate) {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<void>();
    auto replaced = locked->ReplaceSession(
        BootstrapRequest{std::filesystem::path(config_file_path).parent_path(),
                         "gui"},
        root_candidate);
    if (replaced) jobs->Clear();
    return replaced;
  };
  services.node_descriptors = [weak_session] {
    auto locked = weak_session.lock();
    if (!locked) return std::vector<NodeDescriptor>{};
    auto descriptors = locked->NodeDescriptors();
    return descriptors ? std::move(descriptors).Value()
                       : std::vector<NodeDescriptor>{};
  };
  services.runtime_snapshot = [weak_session, jobs] {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<RuntimeSessionSnapshot>();
    auto snapshot = locked->Snapshot();
    if (!snapshot) {
      return Result<RuntimeSessionSnapshot>::Failure(snapshot.GetError());
    }
    auto translated = std::move(snapshot).Value();
    translated.pipeline = jobs->TranslateSnapshot(
        translated.id, std::move(translated.pipeline));
    return Result<RuntimeSessionSnapshot>::Ok(std::move(translated));
  };
  services.snapshot = [weak_session, jobs] {
    auto locked = weak_session.lock();
    if (!locked) return PipelineSnapshot{};
    auto snapshot = locked->Snapshot();
    if (!snapshot) return PipelineSnapshot{};
    auto value = std::move(snapshot).Value();
    return jobs->TranslateSnapshot(value.id, std::move(value.pipeline));
  };
  services.visualization_snapshot = [weak_session](const AgentId& agent) {
    auto locked = weak_session.lock();
    return locked ? locked->VisualizationSnapshotFor(agent)
                  : RuntimeExpired<VisualizationSnapshot>();
  };
  services.alignment_feedback_snapshot = [weak_session] {
    auto locked = weak_session.lock();
    if (!locked) return std::optional<AlignmentFeedbackSnapshot>{};
    auto snapshot = locked->AlignmentFeedbackSnapshotFor();
    return snapshot ? std::move(snapshot).Value()
                    : std::optional<AlignmentFeedbackSnapshot>{};
  };
  services.respond_to_alignment = [weak_session, jobs](
                                       uint64_t job_id,
                                       AlignmentResponse response) {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<void>();
    auto bound = jobs->Find(job_id);
    if (!bound) {
      return Result<void>::Failure(
          Error::InvalidArgument("GUI job token is unknown"));
    }
    if (bound->session_id != locked->CurrentSession()) {
      return Result<void>::Failure(
          Error::InvalidArgument("GUI job belongs to a retired session"));
    }
    return locked->RespondToAlignment(*bound, std::move(response));
  };
  services.subscribe_events = [weak_session, jobs](auto callback) {
    auto locked = weak_session.lock();
    if (!locked) return RuntimeExpired<ExecutionEventSubscription>();
    return locked->Subscribe(
        [callback = std::move(callback), jobs](
            const SessionExecutionEvent& event) {
          callback(jobs->Translate(event));
        });
  };
  return services;
}

}  // namespace open_lmm
