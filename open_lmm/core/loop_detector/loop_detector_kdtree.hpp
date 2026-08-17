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
  explicit KdtreeParams(const Config& config);
  ~KdtreeParams() = default;

 public:
  // IDescriptorKdtree::Params descriptor_params;
  size_t num_candidates{5};
  double distance_threshold{0.2};
  size_t kdtree_rebuild_threshold{50};
  std::string model;
  double pcm_translation_threshold{10.0};
  double pcm_rotation_threshold_deg{20.0};
  std::string pcm_solver{"heuristic"};
  int pcm_threads{1};
  size_t pcm_max_candidates{0};
  float kiss_voxel_size{2.0F};
  bool kiss_use_quatro{false};
  float pose_nn_distance_threshold{10.0F};
  // adaptive is the backward-compatible implicit default: interactive when a
  // GUI broker is enabled, automatic otherwise. Explicit interactive/manual
  // modes must never silently fall back to headless execution.
  std::string feedback_mode{"adaptive"};
  std::string headless_policy{"legacy_combined"};
  int feedback_timeout_sec{0};
};

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
      const std::string& so_name);

 private:
  LoopPair createLoopPair(char agent_id, size_t current_idx,
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
      const std::map<char, AgentOptimizedData>& all_optimized,
      const std::map<char, AgentRawData>& all_raw_data,
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
