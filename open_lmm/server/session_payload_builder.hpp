#pragma once

#include <map>
#include <memory>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/session_state.hpp>

namespace open_lmm {

// Builds immutable payload candidates while keeping resident-memory
// reservations coupled to the raw point-cloud handles they account for.
class SessionPayloadBuilder {
 public:
  explicit SessionPayloadBuilder(
      std::shared_ptr<const SessionPayload> base_payload);

  SessionPayloadBuilder& SetContexts(std::vector<AgentPipelineCtx> contexts);
  SessionPayloadBuilder& SetDatabase(
      std::shared_ptr<const SharedDatabase> database);
  SessionPayloadBuilder& SetOptimizer(
      std::shared_ptr<BackendOptimizerBase> optimizer);
  SessionPayloadBuilder& ReplaceResidentReservations(
      std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations);
  SessionPayloadBuilder& SetResidentReservation(
      const AgentId& agent, std::shared_ptr<MemoryReservation> reservation);

  [[nodiscard]] Result<std::shared_ptr<const SessionPayload>> Build();

 private:
  std::shared_ptr<const SessionPayload> base_;
  std::vector<AgentPipelineCtx> contexts_;
  std::shared_ptr<const SharedDatabase> database_;
  std::shared_ptr<BackendOptimizerBase> optimizer_;
  std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations_;
};

[[nodiscard]] Result<void> ValidateResidentMemoryOwnership(
    const SessionPayload& payload,
    const SessionPayload* base_payload = nullptr);

}  // namespace open_lmm
