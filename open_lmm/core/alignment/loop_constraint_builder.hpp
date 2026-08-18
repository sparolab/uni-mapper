#pragma once

#include <cstdint>

#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/agent_data.hpp>

namespace open_lmm {

enum class AlignmentAcceptanceSource : uint8_t {
  kAutomatic,
  kInteractive,
  kStored,
};

struct LoopConstraintBuildInput {
  const StoredAlignment& accepted;
  AlignmentAcceptanceSource acceptance_source =
      AlignmentAcceptanceSource::kAutomatic;
  const AgentRawData& source;
  const AgentRawDataMap& all_raw_data;
  const AgentOptimizedDataMap& optimized_agents;
  double pose_nn_distance_threshold = 10.0;
  double minimum_source_separation = 10.0;
};

struct ValidatedLoopConstraints {
  StoredAlignment accepted;
  AlignmentAcceptanceSource acceptance_source =
      AlignmentAcceptanceSource::kAutomatic;
  LoopPairVec loops;
};

// Converts every accepted transform (automatic, interactive, or restored)
// through one validation and pose-nearest-neighbor path.
class LoopConstraintBuilder {
 public:
  Result<ValidatedLoopConstraints> Build(
      const AlgorithmExecutionContext& context,
      const LoopConstraintBuildInput& input) const;
};

}  // namespace open_lmm
