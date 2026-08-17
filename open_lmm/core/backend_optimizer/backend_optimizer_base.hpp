#pragma once

#include <Eigen/Geometry>
#include <map>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

class BackendOptimizerBase {
 public:
  BackendOptimizerBase() = default;
  explicit BackendOptimizerBase(Config config);
  virtual ~BackendOptimizerBase() = default;

  // ISAM2가 클래스 멤버로 상태를 관리하므로 GraphStore 인자 불필요.
  // 매 호출 후 joint optimization 결과를 모든 에이전트에 반영.
  virtual std::map<char, AgentOptimizedData> Process(
      const AgentContext&                 ctx,
      const AgentRawData&                 raw_data,
      const LoopPairVec&                  intra_loops,
      const LoopPairVec&                  inter_loops,
      const std::map<char, AgentRawData>& all_raw_data) = 0;

  virtual void parseConfig(Config config) = 0;
  static std::unique_ptr<BackendOptimizerBase> createInstance(Config config);

 protected:
};

}  // namespace open_lmm