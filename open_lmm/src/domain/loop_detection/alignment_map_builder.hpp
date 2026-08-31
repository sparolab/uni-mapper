#pragma once

#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/result.hpp>

namespace open_lmm {

// Builds the map consumed by map-alignment proposers directly at the selected
// alignment resolution. DataLoader deliberately does not own this artifact:
// changing alignment.kiss_voxel_size must rebuild it without reinterpreting a
// coarser, already lossy DataLoad result.
Result<VoxelizedAgentMap> BuildAlignmentMap(
    const AlgorithmExecutionContext& context, const AgentRawData& raw,
    float voxel_size_m);

}  // namespace open_lmm
