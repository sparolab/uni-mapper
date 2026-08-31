#include <open_lmm/server/runtime_client.hpp>

#include "runtime_retirement_coordinator.hpp"

#include <runtime/service/runtime_service.hpp>

#include <utility>

namespace open_lmm {

struct RuntimeClient::Impl {
  explicit Impl(std::size_t max_agent_tasks) : service(max_agent_tasks) {
    retirement.owner = this;
    retirement.destroy = [](void* owner) noexcept {
      delete static_cast<Impl*>(owner);
    };
  }

  RuntimeRetirementNode retirement;
  RuntimeService service;
};

namespace {

template <typename ImplType>
void RetireOrDelete(ImplType* retiring) noexcept {
  if (!retiring) return;
  if (!retiring->service.IsInEventCallback()) {
    delete retiring;
    return;
  }
  GlobalRuntimeRetirementCoordinator().Retire(retiring->retirement);
}

}  // namespace

RuntimeClient::RuntimeClient(std::size_t max_agent_tasks) {
  // Construct the process coordinator first. Reverse static destruction order
  // then keeps it alive for every RuntimeClient::Impl created afterwards.
  (void)GlobalRuntimeRetirementCoordinator();
  impl_ = std::make_unique<Impl>(max_agent_tasks);
}

RuntimeClient::~RuntimeClient() {
  RetireOrDelete(impl_.release());
}

RuntimeClient::RuntimeClient(RuntimeClient&&) noexcept = default;
RuntimeClient& RuntimeClient::operator=(RuntimeClient&& other) noexcept {
  if (this == &other) return *this;
  Impl* retiring = impl_.release();
  impl_ = std::move(other.impl_);
  RetireOrDelete(retiring);
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
Result<CommittedConfigDocuments> RuntimeClient::ConfigDocuments() const {
  return impl_->service.ConfigDocuments();
}

Result<ConfigCandidateCatalog> RuntimeClient::ConfigCandidates() const {
  return impl_->service.ConfigCandidates();
}
Result<std::vector<std::string>> RuntimeClient::RecentLogs(
    std::size_t max_lines) const {
  return impl_->service.RecentLogs(max_lines);
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
