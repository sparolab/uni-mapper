#include <open_lmm/gui/gui_controller_bridge.hpp>

namespace open_lmm {
namespace {
template <typename T>
Result<T> ControllerExpired() {
  return Result<T>::Failure(Error::InvalidArgument("pipeline controller expired"));
}
}  // namespace

GuiServices MakeGuiServices(const std::shared_ptr<PipelineController>& controller,
                            std::string config_file_path) {
  if (controller) controller->SetAlignmentFeedbackEnabled(true);
  std::weak_ptr<PipelineController> weak = controller;
  GuiServices services;
  services.config_file_path = std::move(config_file_path);
  services.submit_run_all = [weak] {
    auto locked = weak.lock();
    return locked ? locked->SubmitRunAll() : ControllerExpired<uint64_t>();
  };
  services.submit_stage = [weak](StageId stage) {
    auto locked = weak.lock();
    return locked ? locked->SubmitStage(stage) : ControllerExpired<uint64_t>();
  };
  services.submit_node = [weak](NodeId node, std::optional<AgentId> agent) {
    auto locked = weak.lock();
    return locked ? locked->SubmitNode(node, agent) : ControllerExpired<uint64_t>();
  };
  services.submit_optimize_through = [weak](AgentId agent) {
    auto locked = weak.lock();
    return locked ? locked->SubmitOptimizeThrough(agent) : ControllerExpired<uint64_t>();
  };
  services.cancel_job = [weak](uint64_t job_id) {
    auto locked = weak.lock();
    return locked ? locked->Cancel(job_id) : ControllerExpired<void>();
  };
  services.apply_config = [weak](ConfigDomain domain, uint64_t revision) {
    auto locked = weak.lock();
    return locked ? locked->ApplyConfig(domain, revision)
                  : ControllerExpired<void>();
  };
  services.node_descriptors = [weak] {
    auto locked = weak.lock();
    return locked ? locked->NodeDescriptors() : std::vector<NodeDescriptor>{};
  };
  services.snapshot = [weak] {
    auto locked = weak.lock();
    return locked ? locked->Snapshot() : PipelineSnapshot{};
  };
  services.visualization_snapshot = [weak](const AgentId& agent) {
    auto locked = weak.lock();
    return locked ? locked->GetVisualizationSnapshot(agent)
                  : ControllerExpired<VisualizationSnapshot>();
  };
  services.alignment_feedback_snapshot = [weak] {
    auto locked = weak.lock();
    return locked ? locked->GetAlignmentFeedbackSnapshot()
                  : std::optional<AlignmentFeedbackSnapshot>{};
  };
  services.respond_to_alignment = [weak](uint64_t job_id,
                                          AlignmentResponse response) {
    auto locked = weak.lock();
    return locked ? locked->RespondToAlignment(job_id, std::move(response))
                  : ControllerExpired<void>();
  };
  services.subscribe_events = [weak](auto callback) {
    auto locked = weak.lock();
    return locked ? locked->SubscribeEvents(std::move(callback))
                  : ExecutionEventSubscription{};
  };
  return services;
}

}  // namespace open_lmm
