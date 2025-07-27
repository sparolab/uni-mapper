#pragma once
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>
#include <tqdmcpp/tqdmcpp.hpp>


#include <filesystem>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp>
#include <type_traits>

#include "loop_detector_base.hpp"

namespace open_lmm {

struct KdtreeParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit KdtreeParams();
  ~KdtreeParams() = default;

 public:
  // IDescriptorKdtree::Params descriptor_params;
  size_t num_candidates{5};
  double distance_threshold{0.2};
  size_t kdtree_rebuild_threshold{50};
  std::string model;
};

/**
 * @brief A templated loop detector implementation that can work with different
 * descriptor types
 * @tparam DescriptorType The type of descriptor used for loop detection (e.g.,
 * ScanContext, SOLID)
 */
class LoopDetectorKdtree : public LoopDetectorBase {
 public:
  explicit LoopDetectorKdtree(const KdtreeParams& params);
  explicit LoopDetectorKdtree(Config config);
  ~LoopDetectorKdtree() override = default;

  std::tuple<LoopPairVec, LoopPairVec> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      ScanVec scans) override;

 private:
  // Helper methods for loop detection and map manipulation

  LoopPair createLoopPair(char agent_id, size_t current_idx,
                          const LoopCandidateInfo& candidate_info);

  std::vector<LoopPair> detectIntraLoops(const ScanVec& scans, char agent_id);

  std::vector<LoopPair> detectInterLoops(
      const ScanVec& scans, std::shared_ptr<SharedDatabase>& shared_data,
      char agent_id);

  std::vector<LoopPair> detectKissMatcherLoops(
      std::shared_ptr<SharedDatabase>& shared_data, char agent_id);

  std::vector<LoopPair> findLoopPairsFromKdTree(
      std::shared_ptr<SharedDatabase>& shared_data,
      const std::vector<Eigen::Isometry3f>& transformed_poses, char agent_id,
      float distance_threshold);

  KdtreeParams params_;
  std::optional<DatabaseKdtree> database_;
  std::shared_ptr<IDescriptorKdtree> model_descriptor_;

  /**
   * @brief Load an dynamic removal module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded dynamic removal module
   */
  static std::shared_ptr<IDescriptorKdtree> loadModule(
      const std::string& so_name);
};

}  // namespace open_lmm