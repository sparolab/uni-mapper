#include "map_server.hpp"

#include "stage_executor.hpp"

namespace open_lmm {

MapServer::MapServer(
    BootstrapConfigSnapshot bootstrap_config,
    std::optional<std::filesystem::path> output_directory,
    std::shared_ptr<ResourceGovernor> resource_governor)
    : executor_(std::make_unique<StageExecutor>(
          std::move(bootstrap_config), std::move(output_directory),
          std::move(resource_governor))) {}
MapServer::~MapServer() = default;

Result<void> MapServer::process() {
  auto cancellation = std::make_shared<CancellationToken>(
      CancellationMetadata());
  auto feedback = std::make_shared<AlignmentFeedbackBroker>();
  for (StageId stage : PipelineStages()) {
    const uint64_t base_revision = Snapshot().revision;
    auto executed = Execute(ExecutionCommand::Stage(stage),
                            {cancellation, feedback, base_revision});
    if (!executed) return Result<void>::Failure(executed.GetError());
    const auto& receipt = executed.Value();
    const uint64_t query_revision = Snapshot().revision;
    if (receipt.base_revision != base_revision ||
        receipt.committed_revision <= base_revision ||
        query_revision != receipt.committed_revision) {
      return Result<void>::Failure(Error::InvalidArgument(
          "batch execution receipt does not match committed runtime: base=" +
          std::to_string(base_revision) + " receipt_base=" +
          std::to_string(receipt.base_revision) + " receipt_committed=" +
          std::to_string(receipt.committed_revision) + " query=" +
          std::to_string(query_revision)));
    }
  }
  return Result<void>::Ok();
}

CancellationCapability MapServer::CancellationMetadata() const {
  return executor_->CancellationMetadata();
}

Result<ExecutionReceipt> MapServer::Execute(
    const ExecutionCommand& command, const ExecutionContext& context) {
  return executor_->Execute(command, context);
}

Result<ConfigApplyReceipt> MapServer::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected,
    const ExecutionContext& context) {
  return executor_->ApplyConfig(candidate, expected, context);
}

CommittedRuntimeSnapshot MapServer::Snapshot() const {
  return executor_->Snapshot();
}

Result<VisualizationSnapshot> MapServer::Visualization(
    const AgentId& agent) const {
  return executor_->Visualization(agent);
}

Result<void> MapServer::InitializeRuntimeRevisions(
    uint64_t runtime_revision, uint64_t config_revision) {
  return executor_->InitializeRuntimeRevisions(runtime_revision, config_revision);
}

Result<void> MapServer::ValidateReady() { return executor_->ValidateReady(); }

}  // namespace open_lmm
