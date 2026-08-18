#pragma once

#include <map>
#include <memory>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/runtime_state.hpp>

namespace open_lmm {

// Builds immutable payload candidates while keeping resident-memory
// reservations coupled to the raw point-cloud handles they account for.
class RuntimePayloadBuilder {
 public:
  explicit RuntimePayloadBuilder(
      std::shared_ptr<const RuntimePayload> base_payload);

  RuntimePayloadBuilder& SetContexts(std::vector<AgentPipelineCtx> contexts);
  RuntimePayloadBuilder& SetDatabase(
      std::shared_ptr<const SharedDatabase> database);
  RuntimePayloadBuilder& SetOptimizer(
      std::shared_ptr<BackendOptimizerBase> optimizer);
  RuntimePayloadBuilder& ReplaceResidentReservations(
      std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations);
  RuntimePayloadBuilder& SetResidentReservation(
      const AgentId& agent, std::shared_ptr<MemoryReservation> reservation);

  [[nodiscard]] Result<std::shared_ptr<const RuntimePayload>> Build();

 private:
  std::shared_ptr<const RuntimePayload> base_;
  std::vector<AgentPipelineCtx> contexts_;
  std::shared_ptr<const SharedDatabase> database_;
  std::shared_ptr<BackendOptimizerBase> optimizer_;
  std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations_;
};

[[nodiscard]] Result<void> ValidateResidentMemoryOwnership(
    const RuntimePayload& payload,
    const RuntimePayload* base_payload = nullptr);

}  // namespace open_lmm
