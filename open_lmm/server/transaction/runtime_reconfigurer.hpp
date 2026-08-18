#pragma once

#include <memory>

#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/runtime_state.hpp>

namespace open_lmm {

struct RuntimeReconfigureCandidate {
  std::shared_ptr<const RuntimeConfig> config;
  std::shared_ptr<const RuntimePayload> payload;
};

// Builds a reconfiguration candidate from one committed config snapshot and
// one explicitly selected candidate file. It owns no mutable config mirror.
class RuntimeReconfigurer {
 public:
  explicit RuntimeReconfigurer(
      std::shared_ptr<const AlgorithmFactory> algorithms = {});

  Result<RuntimeReconfigureCandidate> Prepare(
      const std::shared_ptr<const RuntimeState>& base, ConfigDomain domain,
      uint64_t revision) const;
  Result<RuntimeReconfigureCandidate> Prepare(
      const std::shared_ptr<const RuntimeState>& base,
      const ConfigCandidate& candidate, uint64_t revision) const;

 private:
  Result<RuntimeReconfigureCandidate> PrepareImpl(
      const std::shared_ptr<const RuntimeState>& base, ConfigDomain domain,
      uint64_t revision, const ConfigCandidate* candidate) const;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
};

}  // namespace open_lmm
