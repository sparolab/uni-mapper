#include "runtime_client.hpp"

#include <open_lmm/server/runtime_service.hpp>

#include <utility>
#include <thread>

namespace open_lmm {

struct RuntimeClient::Impl {
  explicit Impl(std::size_t maximum_sessions) : service(maximum_sessions) {}
  RuntimeService service;
};

RuntimeClient::RuntimeClient(std::size_t maximum_sessions)
    : impl_(std::make_unique<Impl>(maximum_sessions)) {}
RuntimeClient::~RuntimeClient() {
  if (!impl_ || !impl_->service.IsInEventCallback()) return;
  // RuntimeService teardown waits for controller workers. If the last client
  // owner is released by one of those workers' callbacks, destroy the Impl on
  // a neutral cleanup thread so PipelineController never joins itself.
  Impl* deferred = impl_.release();
  try {
    std::thread([deferred] { delete deferred; }).detach();
  } catch (...) {
    // Destructing on this callback thread is unsafe. In the exceptional case
    // that a cleanup thread cannot be created, leak rather than self-join or
    // access a controller after destruction.
  }
}
RuntimeClient::RuntimeClient(RuntimeClient&&) noexcept = default;
RuntimeClient& RuntimeClient::operator=(RuntimeClient&&) noexcept = default;

Result<SessionId> RuntimeClient::CreateSession(const BootstrapRequest& request) {
  return impl_->service.CreateSession(request);
}

Result<SessionId> RuntimeClient::CreateSession(
    const BootstrapRequest& request,
    const ConfigCandidate& root_candidate) {
  return impl_->service.CreateSession(request, root_candidate);
}

Result<RuntimeSessionReplacement> RuntimeClient::ReplaceSession(
    const SessionId& previous_session, const BootstrapRequest& request,
    const ConfigCandidate& root_candidate,
    std::function<void(const SessionExecutionEvent&)> callback) {
  return impl_->service.ReplaceSession(previous_session, request,
                                       root_candidate, std::move(callback));
}

Result<JobId> RuntimeClient::Submit(const SessionId& session_id,
                                    const ExecutionRequest& request) {
  return impl_->service.Submit(session_id, request);
}

Result<JobId> RuntimeClient::SubmitRunAll(const SessionId& session_id) {
  return impl_->service.Submit(session_id, ExecutionRequest{});
}

Result<void> RuntimeClient::Cancel(const SessionId& session_id, JobId job_id) {
  return impl_->service.Cancel(session_id, job_id);
}

Result<void> RuntimeClient::Wait(const SessionId& session_id, JobId job_id) {
  return impl_->service.Wait(session_id, job_id);
}

Result<ClientSessionSnapshot> RuntimeClient::Snapshot(
    const SessionId& session_id) const {
  auto snapshot = impl_->service.Snapshot(session_id);
  if (!snapshot) {
    return Result<ClientSessionSnapshot>::Failure(snapshot.GetError());
  }
  const auto& value = snapshot.Value();
  return Result<ClientSessionSnapshot>::Ok(
      {value.id, value.label, value.state, value.output_directory});
}

Result<RuntimeSessionSnapshot> RuntimeClient::RuntimeSnapshot(
    const SessionId& session_id) const {
  return impl_->service.Snapshot(session_id);
}

Result<std::vector<NodeDescriptor>> RuntimeClient::NodeDescriptors(
    const SessionId& session_id) const {
  return impl_->service.NodeDescriptors(session_id);
}

Result<open_lmm::VisualizationSnapshot> RuntimeClient::VisualizationSnapshot(
    const SessionId& session_id, const AgentId& agent) const {
  return impl_->service.VisualizationSnapshot(session_id, agent);
}

Result<std::optional<open_lmm::AlignmentFeedbackSnapshot>>
RuntimeClient::AlignmentFeedbackSnapshot(const SessionId& session_id) const {
  return impl_->service.AlignmentFeedbackSnapshot(session_id);
}

Result<void> RuntimeClient::RespondToAlignment(
    const SessionId& session_id, JobId job_id, AlignmentResponse response) {
  return impl_->service.RespondToAlignment(session_id, job_id,
                                           std::move(response));
}

Result<void> RuntimeClient::SetAlignmentFeedbackEnabled(
    const SessionId& session_id, bool enabled) {
  return impl_->service.SetAlignmentFeedbackEnabled(session_id, enabled);
}

Result<ConfigApplyReceipt> RuntimeClient::ApplyConfig(
    const SessionId& session_id, const ConfigCandidate& candidate,
    const ExpectedRevision& expected) {
  return impl_->service.ApplyConfig(session_id, candidate, expected);
}

Result<ExecutionEventSubscription> RuntimeClient::SubscribeEvents(
    const SessionId& session_id,
    std::function<void(const SessionExecutionEvent&)> callback) {
  return impl_->service.SubscribeEvents(session_id, std::move(callback));
}

Result<void> RuntimeClient::CloseSession(const SessionId& session_id,
                                         CloseMode mode) {
  return impl_->service.CloseSession(session_id, mode);
}

std::vector<SessionId> RuntimeClient::SessionIds() const {
  return impl_->service.SessionIds();
}

}  // namespace open_lmm
