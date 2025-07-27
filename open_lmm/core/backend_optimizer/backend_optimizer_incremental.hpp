#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include "backend_optimizer_base.hpp"

namespace open_lmm {

struct BackendOptimizerIncrementalParam {
  std::string backend_optimizer_type;
  double relinearize_threshold;
  int relinearize_skip;
};

class BackendOptimizerIncremental : public BackendOptimizerBase {
 public:
  explicit BackendOptimizerIncremental(Config config);
  ~BackendOptimizerIncremental() override;
  void parseConfig(Config config) override;
  std::vector<std::pair<int, Eigen::Isometry3d>> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      std::vector<Eigen::Isometry3d> poses, LoopPairVec intra_loops,
      LoopPairVec inter_loops) override;
  void initNoise();

 private:
  BackendOptimizerIncrementalParam param_;
  gtsam::NonlinearFactorGraph pose_graph_;
  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr large_noise_;
  // tqdm bar_;
};

}  // namespace open_lmm