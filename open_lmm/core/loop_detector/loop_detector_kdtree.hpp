#pragma once
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>
#include <filesystem>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/descriptor_index.hpp>
#include <type_traits>

#include "loop_detector_base.hpp"

namespace open_lmm {

using KdtreeParams = LoopDetectorConfig;

/**
 * @brief A templated loop detector implementation that can work with different
 * descriptor types
 * @tparam DescriptorType The type of descriptor used for loop detection (e.g.,
 * ScanContext, SOLID)
 */
class LoopDetectorKdtree : public LoopDetectorBase {
 public:
  LoopDetectorKdtree(const KdtreeParams& params,
                     std::shared_ptr<IDescriptorKdtree> model_descriptor);
  ~LoopDetectorKdtree() override = default;

  LoopDetectorOutput Process(const LoopDetectorInput& input) override;

  static Result<std::shared_ptr<IDescriptorKdtree>> loadModule(
      const std::string& so_name, const std::string& config_json);

 private:
  LoopPair createLoopPair(AgentId agent_id, size_t current_idx,
                          const LoopCandidateInfo& candidate_info);

  std::vector<LoopPair> detectIntraLoops(const ScanVec& scans,
                                         const AgentContext& agent_ctx);

  std::vector<LoopPair> detectInterLoops(const ScanVec& scans,
                                         const DescriptorStore& descriptor_store,
                                         const AgentContext& agent_ctx);

  // transformed_map_points를 out 파라미터로 반환
  std::vector<LoopPair> detectKissMatcherLoops(
      const LoopDetectorInput& input,
      std::vector<Eigen::Vector3f>& out_transformed_map_points,
      std::optional<MapAlignmentProposal>& out_proposal);

  std::vector<LoopPair> loopsFromGlobalTransform(
      const LoopDetectorInput& input,
      const Eigen::Isometry3d& target_T_source);

  std::vector<LoopPair> findLoopPairsFromKdTree(
      const AgentOptimizedDataMap& all_optimized,
      const AgentRawDataMap& all_raw_data,
      const std::vector<Eigen::Isometry3f>& transformed_poses,
      const AgentContext& agent_ctx,
      float distance_threshold);

  KdtreeParams params_;
  std::optional<DatabaseKdtree> database_;
  std::shared_ptr<IDescriptorKdtree> model_descriptor_;

  /**
   * @brief Load an dynamic removal module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded dynamic removal module
   */
};

}  // namespace open_lmm
