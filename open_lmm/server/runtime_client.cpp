#include "runtime_client.hpp"

#include <open_lmm/server/runtime_service.hpp>

#include <utility>

namespace open_lmm {

struct RuntimeClient::Impl {
  explicit Impl(std::size_t maximum_sessions) : service(maximum_sessions) {}
  RuntimeService service;
};

RuntimeClient::RuntimeClient(std::size_t maximum_sessions)
    : impl_(std::make_unique<Impl>(maximum_sessions)) {}
RuntimeClient::~RuntimeClient() = default;
RuntimeClient::RuntimeClient(RuntimeClient&&) noexcept = default;
RuntimeClient& RuntimeClient::operator=(RuntimeClient&&) noexcept = default;

Result<SessionId> RuntimeClient::CreateSession(const BootstrapRequest& request) {
  return impl_->service.CreateSession(request);
}

Result<JobId> RuntimeClient::SubmitRunAll(const SessionId& session_id) {
  return impl_->service.Submit(session_id, ExecutionRequest{});
}

Result<void> RuntimeClient::Cancel(const SessionId& session_id, JobId job_id) {
  return impl_->service.Cancel(session_id, job_id);
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

Result<void> RuntimeClient::CloseSession(const SessionId& session_id,
                                         CloseMode mode) {
  return impl_->service.CloseSession(session_id, mode);
}

std::vector<SessionId> RuntimeClient::SessionIds() const {
  return impl_->service.SessionIds();
}

}  // namespace open_lmm
