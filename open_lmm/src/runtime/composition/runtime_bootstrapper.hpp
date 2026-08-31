#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>
#include <config/bootstrap/bootstrap_config.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/resources/resource_governor.hpp>
#include <runtime/state/runtime_state.hpp>

namespace open_lmm {

struct RuntimeBootstrapRequest {
  BootstrapConfigSnapshot bootstrap_config;
  std::optional<std::filesystem::path> output_directory;
  std::shared_ptr<CancellationToken> cancellation;
};

struct RuntimeBootstrapResult {
  std::shared_ptr<const RuntimeState> initial_state;
  ResourceBudget suggested_resource_budget;
  std::shared_ptr<const AlgorithmProvider> algorithms;
};

class RuntimeBootstrapper {
 public:
  explicit RuntimeBootstrapper(
      std::shared_ptr<const AlgorithmProvider> algorithms = {});

  Result<RuntimeBootstrapResult> Bootstrap(
      const RuntimeBootstrapRequest& request) const;

 private:
  std::shared_ptr<const AlgorithmProvider> algorithms_;
};

}  // namespace open_lmm
