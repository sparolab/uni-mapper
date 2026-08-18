#pragma once

#include <Eigen/Geometry>
#include <map>
#include <cstddef>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/core/algorithm_config.hpp>

namespace open_lmm {

class BackendOptimizerBase {
 public:
  BackendOptimizerBase() = default;
  virtual ~BackendOptimizerBase() = default;

  // Agent별 graph/value를 누적해 매 호출 시 새 ISAM2로 joint batch 최적화한다.
  // lifecycle은 Reset과 processed-agent metadata로 명시적으로 관리한다.
  virtual std::map<AgentId, AgentOptimizedData> Process(
      const AgentContext&                 ctx,
      const AgentRawData&                 raw_data,
      const LoopPairVec&                  intra_loops,
      const LoopPairVec&                  inter_loops,
      const AgentRawDataMap&              all_raw_data) = 0;

  void SetCancellationToken(std::shared_ptr<CancellationToken> token) {
    cancellation_ = std::move(token);
  }
  virtual void Reset() = 0;
  [[nodiscard]] virtual bool HasProcessedAgent(const AgentId& agent_id) const = 0;
  [[nodiscard]] virtual std::size_t ProcessedAgentCount() const = 0;
  static Result<std::unique_ptr<BackendOptimizerBase>> createInstance(
      const OptimizerConfig& config);

 protected:
  std::shared_ptr<CancellationToken> cancellation_;
};

}  // namespace open_lmm
