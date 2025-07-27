#pragma once

#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp>

#include <kiss_matcher/FasterPFH.hpp>
#include <kiss_matcher/GncSolver.hpp>
#include <kiss_matcher/KISSMatcher.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/load_module.hpp>

namespace open_lmm {

class LoopDetectorBase {
 public:
  LoopDetectorBase() = default;
  explicit LoopDetectorBase(Config config);
  virtual ~LoopDetectorBase() = default;
  virtual std::tuple<LoopPairVec, LoopPairVec> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      ScanVec scans) = 0;
  static std::unique_ptr<LoopDetectorBase> createInstance(Config config);
  bool TryKissMatcher(const std::vector<Eigen::Vector3f> tgt_map_vec,
                      const std::vector<Eigen::Vector3f> src_map_vec,
                      const float leaf_size, const bool use_quatro,
                      Eigen::Matrix4f& output);
};

}  // namespace open_lmm