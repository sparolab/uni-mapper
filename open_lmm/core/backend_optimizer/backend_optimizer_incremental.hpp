#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <memory>
#include <set>

#include "backend_optimizer_base.hpp"

namespace open_lmm {

using BackendOptimizerIncrementalParam = OptimizerConfig;

class BackendOptimizerIncremental : public BackendOptimizerBase {
 public:
  explicit BackendOptimizerIncremental(OptimizerConfig config);
  ~BackendOptimizerIncremental() override;

  // ISAM2 자체는 Process 호출마다 생성한다. committed graph/values는 agent 간
  // 누적하며, 각 호출은 working copy에서 처리한 후 성공 시에만 commit한다.
  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput& input) override;

  void initNoise();
  void Reset() override;
  [[nodiscard]] bool HasProcessedAgent(const AgentId& agent_id) const override;
  [[nodiscard]] std::size_t ProcessedAgentCount() const override;

 private:
  struct Lifecycle {
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values values;
    std::set<AgentId> processed_agents;
  };

  BackendOptimizerOutput processTransactional(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput& input);

  BackendOptimizerIncrementalParam param_;

  // Published as one pointer so graph, values, and lifecycle metadata cannot
  // be partially replaced if preparation/allocation fails.
  std::unique_ptr<Lifecycle> lifecycle_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Base::shared_ptr     robust_loop_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr large_noise_;
};

}  // namespace open_lmm
