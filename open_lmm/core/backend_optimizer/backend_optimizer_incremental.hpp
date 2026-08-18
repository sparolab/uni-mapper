#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "backend_optimizer_base.hpp"

namespace open_lmm {

using BackendOptimizerIncrementalParam = OptimizerConfig;

class BackendOptimizerIncremental : public BackendOptimizerBase {
 public:
  explicit BackendOptimizerIncremental(OptimizerConfig config);
  ~BackendOptimizerIncremental() override;

  // ISAM2 자체는 Process 호출마다 생성한다. committed graph/values는 agent 간
  // 누적하며, 각 호출은 working copy에서 처리한 후 성공 시에만 commit한다.
  std::map<AgentId, AgentOptimizedData> Process(
      const AgentContext&                 ctx,
      const AgentRawData&                 raw_data,
      const LoopPairVec&                  intra_loops,
      const LoopPairVec&                  inter_loops,
      const AgentRawDataMap&              all_raw_data) override;

  void initNoise();
  void Reset() override;
  [[nodiscard]] bool HasProcessedAgent(const AgentId& agent_id) const override;
  [[nodiscard]] std::size_t ProcessedAgentCount() const override;

 private:
  BackendOptimizerIncrementalParam param_;

  // 에이전트 간 공유 팩터 그래프 — 클래스 멤버로 누적 (GraphStore 대체)
  gtsam::NonlinearFactorGraph accumulated_graph_;
  gtsam::Values               accumulated_values_;
  std::set<AgentId>           processed_agents_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Base::shared_ptr     robust_loop_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr large_noise_;
};

}  // namespace open_lmm
