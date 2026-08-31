#include "host/gui_controller_bridge.hpp"

#include <filesystem>

namespace open_lmm {
namespace {
template <typename T>
Result<T> RuntimeExpired() {
  return Result<T>::Failure(Error::InvalidArgument("runtime client expired"));
}
}  // namespace

GuiServices MakeGuiServices(const std::shared_ptr<RuntimeClient>& runtime,
                            std::string config_file_path) {
  std::weak_ptr<RuntimeClient> weak_runtime = runtime;
  GuiServices services;
  services.config_file_path = std::move(config_file_path);
  services.submit_run_all = [weak_runtime] {
    auto locked = weak_runtime.lock();
    if (!locked) return RuntimeExpired<uint64_t>();
    auto submitted = locked->SubmitRunAll();
    return submitted ? Result<uint64_t>::Ok(submitted.Value().value)
                     : Result<uint64_t>::Failure(submitted.GetError());
  };
  services.submit_stage = [weak_runtime](StageId stage) {
    auto locked = weak_runtime.lock();
    if (!locked) return RuntimeExpired<uint64_t>();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kStage;
    request.stage = stage;
    auto submitted = locked->Submit(request);
    return submitted ? Result<uint64_t>::Ok(submitted.Value().value)
                     : Result<uint64_t>::Failure(submitted.GetError());
  };
  services.submit_node = [weak_runtime](NodeId node,
                                        std::optional<AgentId> agent) {
    auto locked = weak_runtime.lock();
    if (!locked) return RuntimeExpired<uint64_t>();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kNode;
    request.node = node;
    request.agent = std::move(agent);
    auto submitted = locked->Submit(request);
    return submitted ? Result<uint64_t>::Ok(submitted.Value().value)
                     : Result<uint64_t>::Failure(submitted.GetError());
  };
  services.submit_optimize_through = [weak_runtime](AgentId agent) {
    auto locked = weak_runtime.lock();
    if (!locked) return RuntimeExpired<uint64_t>();
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kOptimizeThrough;
    request.agent = std::move(agent);
    auto submitted = locked->Submit(request);
    return submitted ? Result<uint64_t>::Ok(submitted.Value().value)
                     : Result<uint64_t>::Failure(submitted.GetError());
  };
  services.cancel_job = [weak_runtime](uint64_t job_id) {
    auto locked = weak_runtime.lock();
    return locked ? locked->Cancel({job_id}) : RuntimeExpired<void>();
  };
  services.apply_config = [weak_runtime](ConfigCandidate candidate,
                                         ExpectedRevision expected) {
    auto locked = weak_runtime.lock();
    return locked ? locked->ApplyConfig(candidate, expected)
                  : RuntimeExpired<ConfigApplyReceipt>();
  };
  services.replace_root_config = [weak_runtime,
                                  config_file_path = services.config_file_path](
                                     ConfigCandidate root_candidate,
                                     ExpectedRevision expected) {
    auto locked = weak_runtime.lock();
    if (!locked) return RuntimeExpired<RuntimeReplaceReceipt>();
    return locked->ReplaceRootConfig(
        {std::filesystem::path(config_file_path).parent_path(), "gui",
         std::nullopt},
        root_candidate, expected);
  };
  services.node_descriptors = [weak_runtime] {
    auto locked = weak_runtime.lock();
    if (!locked) return std::vector<NodeDescriptor>{};
    auto descriptors = locked->NodeDescriptors();
    return descriptors ? std::move(descriptors).Value()
                       : std::vector<NodeDescriptor>{};
  };
  services.runtime_snapshot = [weak_runtime] {
    auto locked = weak_runtime.lock();
    return locked ? locked->Snapshot() : RuntimeExpired<RuntimeSnapshot>();
  };
  services.snapshot = [weak_runtime] {
    auto locked = weak_runtime.lock();
    if (!locked) return PipelineSnapshot{};
    auto snapshot = locked->Snapshot();
    return snapshot ? std::move(snapshot).Value().pipeline : PipelineSnapshot{};
  };
  services.visualization_snapshot = [weak_runtime](
                                        const VisualizationQuery& query) {
    auto locked = weak_runtime.lock();
    return locked ? locked->Visualization(query)
                  : RuntimeExpired<VisualizationSnapshot>();
  };
  services.alignment_feedback_snapshot = [weak_runtime] {
    auto locked = weak_runtime.lock();
    if (!locked) return std::optional<AlignmentFeedbackSnapshot>{};
    auto feedback = locked->AlignmentFeedback();
    return feedback ? std::move(feedback).Value()
                    : std::optional<AlignmentFeedbackSnapshot>{};
  };
  services.respond_to_alignment = [weak_runtime](uint64_t job_id,
                                                  AlignmentResponse response) {
    auto locked = weak_runtime.lock();
    return locked ? locked->RespondToAlignment({job_id}, std::move(response))
                  : RuntimeExpired<void>();
  };
  services.subscribe_events = [weak_runtime](auto callback) {
    auto locked = weak_runtime.lock();
    return locked ? locked->SubscribeEvents(std::move(callback))
                  : RuntimeExpired<ExecutionEventSubscription>();
  };
  return services;
}

}  // namespace open_lmm
