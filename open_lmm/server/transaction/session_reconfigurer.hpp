#pragma once

#include <memory>

#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/session_state.hpp>

namespace open_lmm {

struct SessionReconfigureCandidate {
  std::shared_ptr<const SessionConfig> config;
  std::shared_ptr<const SessionPayload> payload;
};

// Builds a reconfiguration candidate from one committed config snapshot and
// one explicitly selected candidate file. It owns no mutable config mirror.
class SessionReconfigurer {
 public:
  explicit SessionReconfigurer(
      std::shared_ptr<const AlgorithmFactory> algorithms = {});

  Result<SessionReconfigureCandidate> Prepare(
      const std::shared_ptr<const SessionState>& base, ConfigDomain domain,
      uint64_t revision) const;
  Result<SessionReconfigureCandidate> Prepare(
      const std::shared_ptr<const SessionState>& base,
      const ConfigCandidate& candidate, uint64_t revision) const;

 private:
  Result<SessionReconfigureCandidate> PrepareImpl(
      const std::shared_ptr<const SessionState>& base, ConfigDomain domain,
      uint64_t revision, const ConfigCandidate* candidate) const;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
};

}  // namespace open_lmm
