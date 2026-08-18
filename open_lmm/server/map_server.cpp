#include "map_server.hpp"

#include "stage_executor.hpp"

namespace open_lmm {

MapServer::MapServer() : executor_(std::make_unique<StageExecutor>()) {}
MapServer::~MapServer() = default;

Result<void> MapServer::process() { return executor_->process(); }

void MapServer::SetCancellationToken(
    std::shared_ptr<CancellationToken> token) {
  executor_->SetCancellationToken(std::move(token));
}

void MapServer::SetAlignmentFeedbackBroker(
    std::shared_ptr<AlignmentFeedbackBroker> broker) {
  executor_->SetAlignmentFeedbackBroker(std::move(broker));
}

Result<void> MapServer::RunStage(StageId stage) {
  return executor_->RunStage(stage);
}

Result<void> MapServer::RunNode(NodeId node, std::optional<char> agent) {
  return executor_->RunNode(node, agent);
}

Result<void> MapServer::RunOptimizeThrough(char target_agent) {
  return executor_->RunOptimizeThrough(target_agent);
}

Result<void> MapServer::Reconfigure(ConfigDomain domain, uint64_t revision) {
  return executor_->Reconfigure(domain, revision);
}

std::vector<char> MapServer::AgentIds() const {
  return executor_->AgentIds();
}

std::optional<CommittedSessionSnapshot> MapServer::SessionSnapshot() const {
  return executor_->SessionSnapshot();
}

Result<VisualizationSnapshot> MapServer::CreateVisualizationSnapshot(
    char agent) const {
  return executor_->CreateVisualizationSnapshot(agent);
}

Result<void> MapServer::ValidateReady() { return executor_->ValidateReady(); }

}  // namespace open_lmm
