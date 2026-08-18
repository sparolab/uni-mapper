#include "map_server.hpp"

#include "stage_executor.hpp"
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

MapServer::MapServer()
    : MapServer(GlobalConfig::config_directory(), std::nullopt) {}
MapServer::MapServer(
    std::filesystem::path config_directory,
    std::optional<std::filesystem::path> output_directory,
    std::shared_ptr<ResourceGovernor> resource_governor)
    : executor_(std::make_unique<StageExecutor>(
          std::move(config_directory), std::move(output_directory),
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
          "batch execution receipt does not match committed session: base=" +
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

CommittedSessionSnapshot MapServer::Snapshot() const {
  return executor_->Snapshot();
}

Result<VisualizationSnapshot> MapServer::Visualization(
    const AgentId& agent) const {
  return executor_->Visualization(agent);
}

Result<void> MapServer::ValidateReady() { return executor_->ValidateReady(); }

}  // namespace open_lmm
