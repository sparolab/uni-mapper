#include "stage_runner.hpp"

#include <array>
#include <algorithm>
#include <set>
#include <stdexcept>

namespace open_lmm {

const std::vector<NodeExecutionSpec>& ExecutionSpecs() {
  static const std::vector<NodeExecutionSpec> specs{
      {NodeId::kDataLoad, "data_load", StageId::kDataLoad,
       ExecutionScope::kPerAgent, false, true,
       {ArtifactType::kConfigSnapshot, ArtifactType::kAgentInput},
       {ArtifactType::kRawData},
       {ArtifactType::kRawData, ArtifactType::kDescriptorState,
        ArtifactType::kLoopCandidates, ArtifactType::kMapAlignment,
        ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses,
        ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
        ArtifactType::kPcdFile}},
      {NodeId::kLoopDetect, "loop_detect", StageId::kAlignment,
       ExecutionScope::kPerAgent, true, true,
       {ArtifactType::kRawData},
       {ArtifactType::kLoopCandidates, ArtifactType::kMapAlignment,
        ArtifactType::kDescriptorState},
       {ArtifactType::kDescriptorState, ArtifactType::kLoopCandidates,
        ArtifactType::kMapAlignment, ArtifactType::kOptimizerState,
        ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
        ArtifactType::kPoseFile, ArtifactType::kPcdFile}},
      {NodeId::kOptimize, "optimize", StageId::kAlignment,
       ExecutionScope::kPerAgent, true, true,
       {ArtifactType::kRawData, ArtifactType::kLoopCandidates},
       {ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses},
       {ArtifactType::kDescriptorState, ArtifactType::kLoopCandidates,
        ArtifactType::kMapAlignment, ArtifactType::kOptimizerState,
        ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
        ArtifactType::kPoseFile, ArtifactType::kPcdFile}},
      {NodeId::kMapUpdate, "map_update", StageId::kMapUpdate,
       ExecutionScope::kPerAgent, false, true,
       {ArtifactType::kOptimizedPoses},
       {ArtifactType::kGlobalMap, ArtifactType::kPcdFile},
       {ArtifactType::kGlobalMap, ArtifactType::kPcdFile}},
      {NodeId::kPoseSave, "pose_save", StageId::kSave,
       ExecutionScope::kPerAgent, false, true,
       {ArtifactType::kOptimizedPoses}, {ArtifactType::kPoseFile},
       {ArtifactType::kPoseFile}},
  };
  return specs;
}

const NodeExecutionSpec& ExecutionSpecFor(NodeId node) {
  const auto& specs = ExecutionSpecs();
  const auto found = std::find_if(specs.begin(), specs.end(),
                                  [node](const auto& spec) {
                                    return spec.id == node;
                                  });
  if (found == specs.end()) throw std::out_of_range("unknown node id");
  return *found;
}

const NodeDescriptor& DescribeNode(NodeId node) {
  static const std::vector<NodeDescriptor> descriptors = [] {
    std::vector<NodeDescriptor> result;
    result.reserve(ExecutionSpecs().size());
    for (const auto& spec : ExecutionSpecs()) {
      result.push_back({spec.id, spec.name, spec.stage, spec.required_artifacts,
                        spec.produces, spec.ordered,
                        spec.supports_cancellation});
    }
    return result;
  }();
  const auto found = std::find_if(descriptors.begin(), descriptors.end(),
                                  [node](const auto& descriptor) {
                                    return descriptor.id == node;
                                  });
  if (found == descriptors.end()) throw std::out_of_range("unknown node id");
  return *found;
}

const std::vector<NodeId>& PipelineNodes() {
  static const std::vector<NodeId> nodes = [] {
    std::vector<NodeId> result;
    result.reserve(ExecutionSpecs().size());
    for (const auto& spec : ExecutionSpecs()) result.push_back(spec.id);
    return result;
  }();
  return nodes;
}

const std::vector<StageId>& PipelineStages() {
  static const std::vector<StageId> stages = [] {
    std::vector<StageId> result;
    for (const auto& spec : ExecutionSpecs()) {
      if (std::find(result.begin(), result.end(), spec.stage) == result.end()) {
        result.push_back(spec.stage);
      }
    }
    return result;
  }();
  return stages;
}

std::vector<NodeId> StageNodes(StageId stage) {
  std::vector<NodeId> result;
  for (const auto& spec : ExecutionSpecs()) {
    if (spec.stage == stage) result.push_back(spec.id);
  }
  return result;
}

Result<std::vector<char>> OrderedAgentPrefix(
    const std::vector<char>& ordered_agents, char target_agent) {
  const auto target = std::find(
      ordered_agents.begin(), ordered_agents.end(), target_agent);
  if (target == ordered_agents.end()) {
    return Result<std::vector<char>>::Failure(
        Error::InvalidArgument("unknown ordered replay target agent"));
  }
  return Result<std::vector<char>>::Ok(
      std::vector<char>(ordered_agents.begin(), std::next(target)));
}

std::size_t ProgressTotal(NodeId node,
                          const std::vector<char>& ordered_agents,
                          std::optional<char> target_agent) {
  const auto& spec = ExecutionSpecFor(node);
  if (!spec.ordered || !target_agent) return 1;
  auto prefix = OrderedAgentPrefix(ordered_agents, *target_agent);
  return prefix ? prefix.Value().size() : 0;
}

namespace {
std::vector<ArtifactType> TransitiveArtifacts(
    std::vector<ArtifactType> seeds, bool include_seeds) {
  std::set<ArtifactType> affected(seeds.begin(), seeds.end());
  bool changed = true;
  while (changed) {
    changed = false;
    for (const auto& descriptor : ExecutionSpecs()) {
      const bool depends = std::any_of(
          descriptor.required_artifacts.begin(),
          descriptor.required_artifacts.end(),
          [&](ArtifactType type) { return affected.contains(type); });
      if (!depends) continue;
      for (ArtifactType type : descriptor.produces) {
        changed = affected.insert(type).second || changed;
      }
    }
  }
  if (!include_seeds) {
    for (ArtifactType seed : seeds) affected.erase(seed);
  }
  return {affected.begin(), affected.end()};
}
}  // namespace

std::vector<ArtifactType> ProducedArtifacts(StageId stage) {
  std::set<ArtifactType> result;
  for (const auto& descriptor : ExecutionSpecs()) {
    if (descriptor.stage == stage) {
      result.insert(descriptor.produces.begin(), descriptor.produces.end());
    }
  }
  return {result.begin(), result.end()};
}

std::vector<ArtifactType> AffectedArtifacts(NodeId node) {
  return ExecutionSpecFor(node).invalidates;
}

std::vector<ArtifactType> AffectedArtifacts(StageId stage) {
  std::set<ArtifactType> affected;
  for (NodeId node : StageNodes(stage)) {
    const auto& invalidates = ExecutionSpecFor(node).invalidates;
    affected.insert(invalidates.begin(), invalidates.end());
  }
  for (ArtifactType produced : ProducedArtifacts(stage)) {
    affected.erase(produced);
  }
  return {affected.begin(), affected.end()};
}

std::vector<ArtifactType> AffectedArtifacts(ConfigDomain domain) {
  switch (domain) {
    case ConfigDomain::kGlobal:
    case ConfigDomain::kDataLoader:
      return TransitiveArtifacts({ArtifactType::kRawData}, true);
    case ConfigDomain::kLoopDetector:
      return TransitiveArtifacts(
          {ArtifactType::kDescriptorState, ArtifactType::kLoopCandidates,
           ArtifactType::kMapAlignment}, true);
    case ConfigDomain::kOptimizer:
      return TransitiveArtifacts(
          {ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses}, true);
    case ConfigDomain::kDynamicRemover:
    case ConfigDomain::kMapSave:
      return TransitiveArtifacts(
          {ArtifactType::kGlobalMap, ArtifactType::kPcdFile}, true);
  }
  return {};
}

}  // namespace open_lmm
