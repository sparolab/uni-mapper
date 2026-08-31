#include "execution_spec.hpp"

#include <array>
#include <algorithm>
#include <iterator>
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
       ExecutionScope::kRuntime, false, true,
       {ArtifactType::kOptimizedPoses}, {ArtifactType::kPoseFile},
       {ArtifactType::kPoseFile}},
      {NodeId::kFallbackMapSave, "fallback_map_save", StageId::kSave,
       ExecutionScope::kRuntime, false, true,
       {ArtifactType::kRawData, ArtifactType::kOptimizedPoses},
       {ArtifactType::kGlobalMap, ArtifactType::kPcdFile},
       {ArtifactType::kGlobalMap, ArtifactType::kPcdFile}},
  };
  return specs;
}

const std::vector<ArtifactExecutionSpec>& ArtifactExecutionSpecs() {
  static const std::vector<ArtifactExecutionSpec> specs{
      {ArtifactType::kConfigSnapshot, ExecutionScope::kRuntime},
      {ArtifactType::kAgentInput, ExecutionScope::kPerAgent},
      {ArtifactType::kRawData, ExecutionScope::kPerAgent},
      {ArtifactType::kDescriptorState, ExecutionScope::kRuntime},
      {ArtifactType::kLoopCandidates, ExecutionScope::kPerAgent},
      {ArtifactType::kMapAlignment, ExecutionScope::kPerAgent},
      {ArtifactType::kOptimizerState, ExecutionScope::kRuntime},
      {ArtifactType::kOptimizedPoses, ExecutionScope::kPerAgent},
      {ArtifactType::kGlobalMap, ExecutionScope::kPerAgent},
      {ArtifactType::kPoseFile, ExecutionScope::kPerAgent},
      {ArtifactType::kPcdFile, ExecutionScope::kPerAgent},
      {ArtifactType::kProfileRecord, ExecutionScope::kPerAgent},
  };
  return specs;
}

ExecutionScope ArtifactOwnership(ArtifactType artifact) {
  const auto& specs = ArtifactExecutionSpecs();
  const auto found = std::find_if(
      specs.begin(), specs.end(), [artifact](const auto& spec) {
        return spec.type == artifact;
      });
  if (found == specs.end()) throw std::out_of_range("unknown artifact type");
  return found->ownership;
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
      result.push_back({spec.id, spec.name, spec.stage, spec.scope,
                        spec.required_artifacts, spec.produces, spec.ordered,
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

Result<std::vector<AgentId>> OrderedAgentPrefix(
    const std::vector<AgentId>& ordered_agents, const AgentId& target_agent) {
  const auto target = std::find(
      ordered_agents.begin(), ordered_agents.end(), target_agent);
  if (target == ordered_agents.end()) {
    return Result<std::vector<AgentId>>::Failure(
        Error::InvalidArgument("unknown ordered replay target agent"));
  }
  return Result<std::vector<AgentId>>::Ok(
      std::vector<AgentId>(ordered_agents.begin(), std::next(target)));
}

std::size_t ProgressTotal(NodeId node,
                          const std::vector<AgentId>& ordered_agents,
                          std::optional<AgentId> target_agent) {
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
          {ArtifactType::kLoopCandidates, ArtifactType::kOptimizerState,
           ArtifactType::kOptimizedPoses}, true);
    case ConfigDomain::kDynamicRemover:
    case ConfigDomain::kMapSave:
      return TransitiveArtifacts(
          {ArtifactType::kGlobalMap, ArtifactType::kPcdFile}, true);
  }
  return {};
}

std::vector<AgentId> ArtifactRevisionAffectedAgents(
    const CommittedRuntimeSnapshot& before,
    const CommittedRuntimeSnapshot& after) {
  std::map<ArtifactKey, uint64_t> before_revisions;
  std::map<ArtifactKey, uint64_t> after_revisions;
  for (const auto& artifact : before.artifacts) {
    before_revisions[artifact.key] = artifact.revision;
  }
  for (const auto& artifact : after.artifacts) {
    after_revisions[artifact.key] = artifact.revision;
  }

  bool runtime_artifact_changed = false;
  std::set<AgentId> changed_agents;
  const auto record_change = [&](const ArtifactKey& key) {
    if (key.agent) {
      changed_agents.insert(*key.agent);
    } else {
      runtime_artifact_changed = true;
    }
  };
  for (const auto& [key, revision] : after_revisions) {
    const auto previous = before_revisions.find(key);
    if (previous == before_revisions.end() || previous->second != revision) {
      record_change(key);
    }
  }
  for (const auto& [key, revision] : before_revisions) {
    (void)revision;
    if (!after_revisions.contains(key)) record_change(key);
  }

  if (runtime_artifact_changed) return after.ordered_agents;
  std::vector<AgentId> result;
  result.reserve(changed_agents.size());
  for (const AgentId& agent : after.ordered_agents) {
    if (changed_agents.erase(agent) != 0) result.push_back(agent);
  }
  result.insert(result.end(), changed_agents.begin(), changed_agents.end());
  return result;
}

}  // namespace open_lmm
