#pragma once

#include <Eigen/Geometry>
#include <map>
#include <cstddef>
#include <mutex>
#include <memory>
#include <vector>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/result.hpp>
#include <config/domain/algorithm_config.hpp>

namespace open_lmm {

struct BackendOptimizerInput {
  const AgentRawData& raw_data;
  const LoopPairVec& intra_loops;
  const LoopPairVec& inter_loops;
  const AgentRawDataMap& all_raw_data;
};

using BackendOptimizerOutput = std::map<AgentId, AgentOptimizedData>;

class BackendOptimizerBase {
 public:
  BackendOptimizerBase() = default;
  virtual ~BackendOptimizerBase() = default;

  // Command-scoped authority is explicit. Implementations translate failures
  // to Result; a candidate that fails after solver mutation must report
  // IsUsable()==false so runtime composition cannot publish it.
  virtual Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput& input) = 0;

  virtual void Reset() = 0;
  [[nodiscard]] virtual bool HasProcessedAgent(const AgentId& agent_id) const = 0;
  [[nodiscard]] virtual std::size_t ProcessedAgentCount() const = 0;
  [[nodiscard]] virtual bool IsUsable() const { return true; }
  [[nodiscard]] virtual Result<std::shared_ptr<BackendOptimizerBase>>
  ForkCandidate() const {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        Error::InvalidArgument("optimizer does not support candidate forks"));
  }
  virtual Result<BackendOptimizerOutput> OptimizePrefix(
      const AlgorithmExecutionContext&, const std::vector<AgentId>&,
      const AgentRawDataMap&) {
    return Result<BackendOptimizerOutput>::Failure(
        Error::InvalidArgument("optimizer does not support prefix refinement"));
  }
 protected:
  mutable std::mutex execution_mutex_;
};

}  // namespace open_lmm
