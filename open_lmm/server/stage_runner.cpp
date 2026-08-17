#include "stage_runner.hpp"

#include <array>
#include <stdexcept>

namespace open_lmm {

const NodeDescriptor& DescribeNode(NodeId node) {
  static const std::array<NodeDescriptor, 5> descriptors{{
      {NodeId::kDataLoad, "data_load", StageId::kDataLoad,
       {ArtifactType::kConfigSnapshot, ArtifactType::kAgentInput},
       {ArtifactType::kRawData}, false, true},
      {NodeId::kLoopDetect, "loop_detect", StageId::kAlignment,
       {ArtifactType::kRawData},
       {ArtifactType::kLoopCandidates, ArtifactType::kDescriptorState},
       true, true},
      {NodeId::kOptimize, "optimize", StageId::kAlignment,
       {ArtifactType::kRawData, ArtifactType::kLoopCandidates},
       {ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses},
       true, true},
      {NodeId::kMapUpdate, "map_update", StageId::kMapUpdate,
       {ArtifactType::kOptimizedPoses},
       {ArtifactType::kGlobalMap, ArtifactType::kPcdFile}, false, true},
      {NodeId::kPoseSave, "pose_save", StageId::kSave,
       {ArtifactType::kOptimizedPoses}, {ArtifactType::kPoseFile}, false, true},
  }};
  const auto index = static_cast<std::size_t>(node);
  if (index >= descriptors.size()) throw std::out_of_range("unknown node id");
  return descriptors[index];
}

}  // namespace open_lmm
