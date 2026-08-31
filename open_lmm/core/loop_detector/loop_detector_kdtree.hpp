#pragma once
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>
#include <filesystem>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/core/descriptor/descriptor_engine.hpp>
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
                     std::shared_ptr<const DescriptorEngine> descriptor_engine);
  ~LoopDetectorKdtree() override = default;

  Result<LoopDetectorOutput> Process(
      const AlgorithmExecutionContext& context,
      const LoopDetectorProcessInput& input) override;

  static Result<std::shared_ptr<IDescriptorKdtree>> loadModule(
      const std::string& so_name, const std::string& config_json);

 private:
  LoopPair createLoopPair(AgentId agent_id, size_t current_idx,
                          const LoopCandidateInfo& candidate_info);

  Result<std::vector<LoopPair>> detectIntraLoops(
      const AlgorithmExecutionContext& context, const ScanVec& scans,
      const AgentContext& agent_ctx);

  Result<std::vector<LoopPair>> detectInterLoops(
      const AlgorithmExecutionContext& context, const ScanVec& scans,
      const DescriptorStore& descriptor_store,
      const AgentContext& agent_ctx);

  std::optional<MapAlignmentProposal> proposeKissAlignment(
      const AlgorithmExecutionContext& context,
      const LoopDetectorProcessInput& input,
      const VoxelizedAgentMap& current_map);

  KdtreeParams params_;
  std::optional<DatabaseKdtree> database_;
  std::shared_ptr<const DescriptorEngine> descriptor_engine_;

  /**
   * @brief Load an dynamic removal module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded dynamic removal module
   */
};

}  // namespace open_lmm
