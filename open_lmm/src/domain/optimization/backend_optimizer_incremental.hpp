#pragma once
#include <gtsam/nonlinear/ISAM2.h>

#include <memory>
#include <map>
#include <vector>

#include "backend_optimizer_base.hpp"

namespace open_lmm {

using BackendOptimizerIncrementalParam = OptimizerConfig;

class BackendOptimizerIncremental : public BackendOptimizerBase {
 public:
  explicit BackendOptimizerIncremental(OptimizerConfig config);
  ~BackendOptimizerIncremental() override;

  // One command-private ISAM2 instance receives only agent-local deltas.
  // A failure after solver mutation poisons the unpublished candidate.
  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput& input) override;

  void initNoise();
  void Reset() override;
  [[nodiscard]] bool HasProcessedAgent(const AgentId& agent_id) const override;
  [[nodiscard]] std::size_t ProcessedAgentCount() const override;
  [[nodiscard]] bool IsUsable() const override;
  [[nodiscard]] Result<std::shared_ptr<BackendOptimizerBase>> ForkCandidate()
      const override;
  Result<BackendOptimizerOutput> OptimizePrefix(
      const AlgorithmExecutionContext& context,
      const std::vector<AgentId>& retained_agents,
      const AgentRawDataMap& all_raw_data) override;

  struct Diagnostics {
    std::size_t solver_constructions = 0;
    std::size_t nonempty_updates = 0;
    std::size_t submitted_factors = 0;
    std::size_t factor_count = 0;
    std::size_t value_count = 0;
    std::size_t variables_relinearized = 0;
    std::size_t variables_reeliminated = 0;
    std::size_t candidate_forks = 0;
    std::size_t removed_factors = 0;
    bool poisoned = false;
  };
  [[nodiscard]] Diagnostics GetDiagnostics() const;

 private:
  struct Lifecycle {
    explicit Lifecycle(const gtsam::ISAM2Params& params) : solver(params) {}
    Lifecycle(const Lifecycle& other)
        : solver(other.solver),
          processed_agents(other.processed_agents),
          factor_indices(other.factor_indices),
          agent_keys(other.agent_keys),
          diagnostics(other.diagnostics),
          poisoned(other.poisoned) {}

    gtsam::ISAM2 solver;
    std::vector<AgentId> processed_agents;
    std::map<AgentId, gtsam::FactorIndices> factor_indices;
    std::map<AgentId, gtsam::KeyVector> agent_keys;
    Diagnostics diagnostics;
    bool poisoned = false;
  };

  BackendOptimizerOutput processTransactional(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput& input, bool& mutation_started);

  BackendOptimizerOutput BuildOutput(
      const AlgorithmExecutionContext& context, const gtsam::Values& values,
      const AgentRawDataMap& all_raw_data,
      const AgentRawData* current_raw_data) const;

  [[nodiscard]] gtsam::ISAM2Params IsamParams() const;

  BackendOptimizerIncrementalParam param_;

  // Command-private until RuntimeTransaction publishes the containing payload.
  std::unique_ptr<Lifecycle> lifecycle_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Base::shared_ptr     robust_loop_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr large_noise_;
};

}  // namespace open_lmm
