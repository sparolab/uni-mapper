#pragma once

#include <Eigen/Geometry>
#include <filesystem>
#include <map>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/descriptor_index.hpp>

#include <open_lmm/core/algorithm_config.hpp>
#include <open_lmm/utils/load_module.hpp>

namespace open_lmm {

struct LoopDetectorProcessInput {
  const AgentRawData& current;
  const DescriptorStore& descriptor_store;
  const AgentRawDataMap& all_raw_data;
  const AgentOptimizedDataMap& all_optimized;
  const StoredAlignment* stored_alignment = nullptr;
};

class LoopDetectorBase {
 public:
  LoopDetectorBase() = default;
  virtual ~LoopDetectorBase() = default;
  virtual Result<LoopDetectorOutput> Process(
      const AlgorithmExecutionContext& context,
      const LoopDetectorProcessInput& input) = 0;
};

Result<void> InspectDescriptorPlugin(const LoopDetectorConfig& config);

}  // namespace open_lmm
