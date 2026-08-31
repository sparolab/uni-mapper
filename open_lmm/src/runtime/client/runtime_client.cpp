#include <open_lmm/server/runtime_client.hpp>

#include <runtime/service/runtime_service.hpp>

#include <thread>
#include <utility>

namespace open_lmm {

struct RuntimeClient::Impl {
  explicit Impl(std::size_t max_agent_tasks) : service(max_agent_tasks) {}
  RuntimeService service;
};

RuntimeClient::RuntimeClient(std::size_t max_agent_tasks)
    : impl_(std::make_unique<Impl>(max_agent_tasks)) {}

RuntimeClient::~RuntimeClient() {
  if (!impl_ || !impl_->service.IsInEventCallback()) return;
  // Controller workers must never join themselves.  Move assignment below
  // shares this same destruction path by routing the old Impl through this
  // destructor when necessary.
  Impl* deferred = impl_.release();
  try {
    std::thread([deferred] { delete deferred; }).detach();
  } catch (...) {
    // A leak is preferable to self-join during process teardown failure.
  }
}

RuntimeClient::RuntimeClient(RuntimeClient&&) noexcept = default;
RuntimeClient& RuntimeClient::operator=(RuntimeClient&& other) noexcept {
  if (this == &other) return *this;
  auto retiring = std::move(impl_);
  impl_ = std::move(other.impl_);
  if (!retiring || !retiring->service.IsInEventCallback()) return *this;
  Impl* deferred = retiring.release();
  try {
    std::thread([deferred] { delete deferred; }).detach();
  } catch (...) {
    // A leak is preferable to joining the emitting controller worker.
  }
  return *this;
}

Result<void> RuntimeClient::Open(const BootstrapRequest& request) {
  return impl_->service.Open(request);
}
Result<void> RuntimeClient::Open(const BootstrapRequest& request,
                                 const ConfigCandidate& candidate) {
  return impl_->service.Open(request, candidate);
}
Result<RuntimeReplaceReceipt> RuntimeClient::ReplaceRootConfig(
    const BootstrapRequest& request, const ConfigCandidate& candidate,
    const ExpectedRevision& expected) {
  return impl_->service.ReplaceRootConfig(request, candidate, expected);
}
Result<JobHandle> RuntimeClient::Submit(const ExecutionRequest& request) {
  return impl_->service.Submit(request);
}
Result<JobHandle> RuntimeClient::SubmitRunAll() {
  return impl_->service.Submit(ExecutionRequest{});
}
Result<void> RuntimeClient::Cancel(JobHandle job) { return impl_->service.Cancel(job); }
Result<void> RuntimeClient::Wait(JobHandle job) { return impl_->service.Wait(job); }
Result<RuntimeSnapshot> RuntimeClient::Snapshot() const {
  return impl_->service.Snapshot();
}
Result<std::vector<NodeDescriptor>> RuntimeClient::NodeDescriptors() const {
  return impl_->service.NodeDescriptors();
}
Result<VisualizationSnapshot> RuntimeClient::Visualization(
    const AgentId& agent) const {
  return Visualization(VisualizationQuery{agent});
}
Result<VisualizationSnapshot> RuntimeClient::Visualization(
    const VisualizationQuery& query) const {
  return impl_->service.Visualization(query);
}
Result<std::optional<AlignmentFeedbackSnapshot>>
RuntimeClient::AlignmentFeedback() const { return impl_->service.AlignmentFeedback(); }
Result<void> RuntimeClient::RespondToAlignment(JobHandle job,
                                                AlignmentResponse response) {
  return impl_->service.RespondToAlignment(job, std::move(response));
}
Result<void> RuntimeClient::SetAlignmentFeedbackEnabled(bool enabled) {
  return impl_->service.SetAlignmentFeedbackEnabled(enabled);
}
Result<ConfigApplyReceipt> RuntimeClient::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected) {
  return impl_->service.ApplyConfig(candidate, expected);
}
Result<ExecutionEventSubscription> RuntimeClient::SubscribeEvents(
    std::function<void(const ExecutionEvent&)> callback) {
  return impl_->service.SubscribeEvents(std::move(callback));
}
Result<void> RuntimeClient::Close(CloseMode mode) { return impl_->service.Close(mode); }
bool RuntimeClient::IsOpen() const { return impl_->service.IsOpen(); }

}  // namespace open_lmm
