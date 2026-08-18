#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>
#include <open_lmm/server/resource_governor.hpp>
#include <open_lmm/server/session_state.hpp>

namespace open_lmm {

class AlgorithmFactory;

struct SessionBootstrapRequest {
  BootstrapConfigSnapshot bootstrap_config;
  std::optional<std::filesystem::path> output_directory;
  std::shared_ptr<CancellationToken> cancellation;
};

struct SessionBootstrapResult {
  std::shared_ptr<const SessionState> initial_state;
  ResourceBudget suggested_resource_budget;
};

class SessionBootstrapper {
 public:
  explicit SessionBootstrapper(
      std::shared_ptr<const AlgorithmFactory> algorithms = {});

  Result<SessionBootstrapResult> Bootstrap(
      const SessionBootstrapRequest& request) const;

 private:
  std::shared_ptr<const AlgorithmFactory> algorithms_;
};

}  // namespace open_lmm
