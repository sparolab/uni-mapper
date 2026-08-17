#pragma once

#include <Eigen/Geometry>
#include <map>
#include <cstddef>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

class BackendOptimizerBase {
 public:
  BackendOptimizerBase() = default;
  explicit BackendOptimizerBase(Config config);
  virtual ~BackendOptimizerBase() = default;

  // Agent별 graph/value를 누적해 매 호출 시 새 ISAM2로 joint batch 최적화한다.
  // lifecycle은 Reset과 processed-agent metadata로 명시적으로 관리한다.
  virtual std::map<char, AgentOptimizedData> Process(
      const AgentContext&                 ctx,
      const AgentRawData&                 raw_data,
      const LoopPairVec&                  intra_loops,
      const LoopPairVec&                  inter_loops,
      const std::map<char, AgentRawData>& all_raw_data) = 0;

  virtual void parseConfig(Config config) = 0;
  void SetCancellationToken(std::shared_ptr<CancellationToken> token) {
    cancellation_ = std::move(token);
  }
  virtual void Reset() = 0;
  [[nodiscard]] virtual bool HasProcessedAgent(char agent_id) const = 0;
  [[nodiscard]] virtual std::size_t ProcessedAgentCount() const = 0;
  static Result<std::unique_ptr<BackendOptimizerBase>> createInstance(Config config);

 protected:
  std::shared_ptr<CancellationToken> cancellation_;
};

}  // namespace open_lmm
