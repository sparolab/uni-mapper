#include "runtime_payload_builder.hpp"

#include <algorithm>
#include <utility>

namespace open_lmm {

Result<void> ValidateResidentMemoryOwnership(
    const RuntimePayload& payload, const RuntimePayload* base_payload) {
  if (!payload.database) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime payload database is unavailable"));
  }

  for (const auto& [agent, raw] : payload.database->raw_data) {
    if (!raw) {
      return Result<void>::Failure(Error::InvalidArgument(
          "resident raw payload is null for agent " + agent.Value()));
    }
    const auto reservation = payload.resident_memory_reservations.find(agent);
    if (reservation == payload.resident_memory_reservations.end() ||
        !reservation->second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "resident raw payload has no memory reservation for agent " +
          agent.Value()));
    }

    if (base_payload && base_payload->database) {
      const auto old_raw = base_payload->database->raw_data.find(agent);
      const auto old_reservation =
          base_payload->resident_memory_reservations.find(agent);
      if (old_raw != base_payload->database->raw_data.end() &&
          old_reservation !=
              base_payload->resident_memory_reservations.end()) {
        const bool shares_raw = old_raw->second.get() == raw.get();
        const bool shares_reservation =
            old_reservation->second.get() == reservation->second.get();
        if (shares_raw != shares_reservation) {
          return Result<void>::Failure(Error::InvalidArgument(
              "raw payload and resident reservation lifetime diverged for "
              "agent " + agent.Value()));
        }
      }
    }
  }

  for (const auto& [agent, reservation] :
       payload.resident_memory_reservations) {
    if (!reservation || !payload.database->raw_data.contains(agent)) {
      return Result<void>::Failure(Error::InvalidArgument(
          "resident memory reservation has no raw payload for agent " +
          agent.Value()));
    }
  }

  for (const auto& context : payload.contexts) {
    const auto raw = payload.database->raw_data.find(context.agent.id);
    if (context.raw_data) {
      if (raw == payload.database->raw_data.end() ||
          raw->second.get() != context.raw_data.get()) {
        return Result<void>::Failure(Error::InvalidArgument(
            "context and database raw payload differ for agent " +
            context.agent.id.Value()));
      }
    } else if (raw != payload.database->raw_data.end()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "database raw payload has no matching context for agent " +
          context.agent.id.Value()));
    }
  }

  return Result<void>::Ok();
}

RuntimePayloadBuilder::RuntimePayloadBuilder(
    std::shared_ptr<const RuntimePayload> base_payload)
    : base_(std::move(base_payload)) {
  if (!base_) return;
  contexts_ = base_->contexts;
  database_ = base_->database;
  optimizer_ = base_->optimizer;
  reservations_ = base_->resident_memory_reservations;
}

RuntimePayloadBuilder& RuntimePayloadBuilder::SetContexts(
    std::vector<AgentPipelineCtx> contexts) {
  contexts_ = std::move(contexts);
  return *this;
}

RuntimePayloadBuilder& RuntimePayloadBuilder::SetDatabase(
    std::shared_ptr<const SharedDatabase> database) {
  database_ = std::move(database);
  return *this;
}

RuntimePayloadBuilder& RuntimePayloadBuilder::SetOptimizer(
    std::shared_ptr<BackendOptimizerBase> optimizer) {
  optimizer_ = std::move(optimizer);
  return *this;
}

RuntimePayloadBuilder& RuntimePayloadBuilder::ReplaceResidentReservations(
    std::map<AgentId, std::shared_ptr<MemoryReservation>> reservations) {
  reservations_ = std::move(reservations);
  return *this;
}

RuntimePayloadBuilder& RuntimePayloadBuilder::SetResidentReservation(
    const AgentId& agent, std::shared_ptr<MemoryReservation> reservation) {
  reservations_[agent] = std::move(reservation);
  return *this;
}

Result<std::shared_ptr<const RuntimePayload>> RuntimePayloadBuilder::Build() {
  if (!database_ || !optimizer_) {
    return Result<std::shared_ptr<const RuntimePayload>>::Failure(
        Error::InvalidArgument("runtime payload candidate is incomplete"));
  }
  auto candidate = std::make_shared<RuntimePayload>();
  candidate->contexts = std::move(contexts_);
  candidate->database = std::move(database_);
  candidate->optimizer = std::move(optimizer_);
  candidate->resident_memory_reservations = std::move(reservations_);
  auto valid = ValidateResidentMemoryOwnership(*candidate, base_.get());
  if (!valid) {
    return Result<std::shared_ptr<const RuntimePayload>>::Failure(
        valid.GetError());
  }
  return Result<std::shared_ptr<const RuntimePayload>>::Ok(
      std::shared_ptr<const RuntimePayload>(std::move(candidate)));
}

}  // namespace open_lmm
