#include "session_state.hpp"

#include <algorithm>

namespace open_lmm {
namespace {

const AgentPipelineCtx* FindContext(const SessionState& state, char agent) {
  if (!state.payload) return nullptr;
  const auto found = std::find_if(
      state.payload->contexts.begin(), state.payload->contexts.end(),
      [agent](const AgentPipelineCtx& context) {
        return context.agent.id == agent;
      });
  return found == state.payload->contexts.end() ? nullptr : &*found;
}

Result<void> MissingPayload(const ArtifactMetadata& artifact) {
  return Result<void>::Failure(Error::InvalidArgument(
      "ready artifact has no committed payload: type=" +
      std::to_string(static_cast<int>(artifact.key.type))));
}

}  // namespace

SessionTransaction::SessionTransaction(
    std::shared_ptr<const SessionState> base)
    : base_(std::move(base)), working_(std::make_unique<SessionState>(*base_)) {}

uint64_t SessionTransaction::BaseRevision() const { return base_->revision; }

const std::shared_ptr<const SessionState>& SessionTransaction::Base() const {
  return base_;
}

SessionState& SessionTransaction::Working() { return *working_; }

void SessionTransaction::SetPayload(
    std::shared_ptr<const SessionPayload> payload) {
  working_->payload = std::move(payload);
}

Result<void> SessionTransaction::Validate() const {
  if (!working_->config || !working_->payload ||
      !working_->payload->database || !working_->payload->optimizer) {
    return Result<void>::Failure(
        Error::InvalidArgument("session transaction has incomplete ownership"));
  }
  const auto& database = *working_->payload->database;
  for (const auto& artifact : working_->artifacts) {
    if (artifact.state != ArtifactState::kReady) continue;
    const char agent = artifact.key.agent.value_or(0);
    const AgentPipelineCtx* context =
        artifact.key.agent ? FindContext(*working_, agent) : nullptr;
    switch (artifact.key.type) {
      case ArtifactType::kRawData:
        if (!context || !context->raw_data ||
            !database.raw_data.contains(agent)) {
          return MissingPayload(artifact);
        }
        break;
      case ArtifactType::kLoopCandidates:
      case ArtifactType::kMapAlignment:
        if (!context || !context->loop_output) return MissingPayload(artifact);
        break;
      case ArtifactType::kDescriptorState: {
        const bool has_output = std::any_of(
            working_->payload->contexts.begin(),
            working_->payload->contexts.end(),
            [](const AgentPipelineCtx& item) {
              return static_cast<bool>(item.loop_output);
            });
        if (!has_output) return MissingPayload(artifact);
        break;
      }
      case ArtifactType::kOptimizerState:
        if (working_->optimizer.processed_agents.empty()) {
          return MissingPayload(artifact);
        }
        break;
      case ArtifactType::kOptimizedPoses:
        if (!database.optimized_data.contains(agent)) {
          return MissingPayload(artifact);
        }
        break;
      case ArtifactType::kGlobalMap:
      case ArtifactType::kPoseFile:
      case ArtifactType::kPcdFile:
        if (artifact.external_path.empty()) return MissingPayload(artifact);
        break;
      case ArtifactType::kConfigSnapshot:
      case ArtifactType::kAgentInput:
      case ArtifactType::kProfileRecord:
        break;
    }
  }
  return Result<void>::Ok();
}

Result<std::shared_ptr<const SessionState>> SessionTransaction::Finalize(
    const std::shared_ptr<CancellationToken>& cancellation) && {
  if (cancellation && cancellation->IsCancellationRequested()) {
    return Result<std::shared_ptr<const SessionState>>::Failure(
        Error::Cancelled("before session commit"));
  }
  working_->revision = base_->revision + 1;
  working_->optimizer.processed_agents.clear();
  for (char agent : working_->ordered_agents) {
    if (working_->payload && working_->payload->optimizer &&
        working_->payload->optimizer->HasProcessedAgent(agent)) {
      working_->optimizer.processed_agents.push_back(agent);
    }
  }
  auto valid = Validate();
  if (!valid) {
    return Result<std::shared_ptr<const SessionState>>::Failure(
        valid.GetError());
  }
  return Result<std::shared_ptr<const SessionState>>::Ok(
      std::shared_ptr<const SessionState>(std::move(working_)));
}

}  // namespace open_lmm
