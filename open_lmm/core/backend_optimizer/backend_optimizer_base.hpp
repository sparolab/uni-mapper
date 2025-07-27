#pragma once

// #include <string>
// #include <memory>
// #include <Eigen/Core>
#include <Eigen/Geometry>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

class BackendOptimizerBase {
 public:
  BackendOptimizerBase() = default;
  explicit BackendOptimizerBase(Config config);
  virtual ~BackendOptimizerBase() = default;
  virtual std::vector<std::pair<int, Eigen::Isometry3d>> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      std::vector<Eigen::Isometry3d> poses, LoopPairVec intra_loops,
      LoopPairVec inter_loops) = 0;
  virtual void parseConfig(Config config) = 0;
  static std::unique_ptr<BackendOptimizerBase> createInstance(Config config);

 protected:
};

}  // namespace open_lmm