#pragma once

#include <Eigen/Geometry>
#include <cstddef>
#include <filesystem>
#include <map>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/descriptor_index.hpp>

#include <config/domain/algorithm_config.hpp>

namespace open_lmm {

struct LoopDetectorProcessInput {
  const AgentRawData& current;
  const DescriptorStore& descriptor_store;
  const AgentRawDataMap& all_raw_data;
  const AgentOptimizedDataMap& all_optimized;
  const StoredAlignment* stored_alignment = nullptr;
  std::size_t min_intra_loop_frame_gap = 0;
};

class LoopDetectorBase {
 public:
  LoopDetectorBase() = default;
  virtual ~LoopDetectorBase() = default;
  virtual Result<LoopDetectorOutput> Process(
      const AlgorithmExecutionContext& context,
      const LoopDetectorProcessInput& input) = 0;
};

}  // namespace open_lmm
