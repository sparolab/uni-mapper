#include "artifact_repository.hpp"

#include <algorithm>

namespace open_lmm {
namespace {
const char* StageName(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad: return "data_load";
    case StageId::kAlignment: return "alignment";
    case StageId::kMapUpdate: return "map_update";
    case StageId::kSave: return "save";
  }
  return "unknown";
}
}  // namespace

bool ArtifactRepository::IsPerAgent(ArtifactType type) {
  return type != ArtifactType::kOptimizerState &&
         type != ArtifactType::kDescriptorState &&
         type != ArtifactType::kConfigSnapshot;
}

std::vector<ArtifactType> ArtifactRepository::ProducedBy(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad: return {ArtifactType::kRawData};
    case StageId::kAlignment:
      return {ArtifactType::kDescriptorState, ArtifactType::kLoopCandidates, ArtifactType::kOptimizerState,
              ArtifactType::kOptimizedPoses};
    case StageId::kMapUpdate:
      return {ArtifactType::kGlobalMap, ArtifactType::kPcdFile};
    case StageId::kSave: return {ArtifactType::kPoseFile};
  }
  return {};
}

void ArtifactRepository::RegisterAgents(const std::vector<char>& agents) {
  std::lock_guard lock(mutex_);
  agents_ = agents;
  for (auto type : {ArtifactType::kRawData, ArtifactType::kLoopCandidates,
                    ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
                    ArtifactType::kPoseFile, ArtifactType::kPcdFile}) {
    for (char agent : agents_) {
      ArtifactKey key{type, agent};
      artifacts_.try_emplace(key, ArtifactMetadata{key});
    }
  }
  ArtifactKey optimizer{ArtifactType::kOptimizerState, std::nullopt};
  artifacts_.try_emplace(optimizer, ArtifactMetadata{optimizer});
  ArtifactKey descriptor{ArtifactType::kDescriptorState, std::nullopt};
  artifacts_.try_emplace(descriptor, ArtifactMetadata{descriptor});
  ArtifactKey config{ArtifactType::kConfigSnapshot, std::nullopt};
  artifacts_[config] = ArtifactMetadata{config, ArtifactState::kReady,
                                        next_revision_++, "session"};
  for (char agent : agents_) {
    ArtifactKey input{ArtifactType::kAgentInput, agent};
    artifacts_[input] = ArtifactMetadata{input, ArtifactState::kReady,
                                         next_revision_++, "session"};
  }
}

void ArtifactRepository::setTypes(const std::vector<ArtifactType>& types,
                                  ArtifactState state,
                                  const std::string& producer,
                                  const std::string& detail) {
  for (auto type : types) {
    if (IsPerAgent(type)) {
      for (char agent : agents_) {
        auto& item = artifacts_[ArtifactKey{type, agent}];
        item.key = {type, agent};
        item.state = state;
        item.revision = next_revision_++;
        item.producer = producer;
        item.detail = detail;
      }
    } else {
      auto& item = artifacts_[ArtifactKey{type, std::nullopt}];
      item.key = {type, std::nullopt};
      item.state = state;
      item.revision = next_revision_++;
      item.producer = producer;
      item.detail = detail;
    }
  }
}

void ArtifactRepository::invalidateDownstream(StageId stage) {
  std::vector<ArtifactType> types;
  if (stage == StageId::kDataLoad) {
    types = {ArtifactType::kLoopCandidates, ArtifactType::kOptimizerState,
             ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
             ArtifactType::kPoseFile, ArtifactType::kPcdFile};
  } else if (stage == StageId::kAlignment) {
    types = {ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
             ArtifactType::kPcdFile};
  } else if (stage == StageId::kMapUpdate) {
    types = {ArtifactType::kPcdFile};
  }
  setTypes(types, ArtifactState::kStale, StageName(stage));
}

void ArtifactRepository::BeginStage(StageId stage) {
  std::lock_guard lock(mutex_);
  invalidateDownstream(stage);
}
void ArtifactRepository::CompleteStage(StageId stage) {
  std::lock_guard lock(mutex_);
  setTypes(ProducedBy(stage), ArtifactState::kReady, StageName(stage));
}
void ArtifactRepository::FailStage(StageId stage, std::string detail) {
  std::lock_guard lock(mutex_);
  setTypes(ProducedBy(stage), ArtifactState::kFailed, StageName(stage), detail);
}

void ArtifactRepository::CompleteOptimizeThrough(
    char target_agent, const std::vector<char>& ordered_agents) {
  std::lock_guard lock(mutex_);
  setTypes({ArtifactType::kOptimizerState}, ArtifactState::kReady,
           "optimizer_replay");
  bool completed = true;
  for (char agent : ordered_agents) {
    auto& item = artifacts_[ArtifactKey{ArtifactType::kOptimizedPoses, agent}];
    item.key = {ArtifactType::kOptimizedPoses, agent};
    item.state = completed ? ArtifactState::kReady : ArtifactState::kStale;
    item.revision = next_revision_++;
    item.producer = "optimizer_replay";
    if (agent == target_agent) completed = false;
  }
  setTypes({ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
            ArtifactType::kPcdFile}, ArtifactState::kStale,
           "optimizer_replay");
}

ArtifactMetadata ArtifactRepository::Get(const ArtifactKey& key) const {
  std::lock_guard lock(mutex_);
  auto it = artifacts_.find(key);
  return it == artifacts_.end() ? ArtifactMetadata{key} : it->second;
}

std::vector<ArtifactMetadata> ArtifactRepository::Snapshot() const {
  std::lock_guard lock(mutex_);
  std::vector<ArtifactMetadata> result;
  result.reserve(artifacts_.size());
  for (const auto& [key, value] : artifacts_) result.push_back(value);
  return result;
}

void ArtifactRepository::Restore(
    const std::vector<ArtifactMetadata>& snapshot) {
  std::lock_guard lock(mutex_);
  artifacts_.clear();
  for (const auto& item : snapshot) artifacts_[item.key] = item;
}

Result<void> ArtifactRepository::ValidateNode(
    NodeId node, std::optional<char> agent) const {
  std::lock_guard lock(mutex_);
  if (!agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("node command required_artifacts an agent"));
  }
  if (std::find(agents_.begin(), agents_.end(), *agent) == agents_.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }
  const auto& descriptor = DescribeNode(node);
  for (auto type : descriptor.required_artifacts) {
    std::optional<char> key_agent;
    if (IsPerAgent(type)) key_agent = agent;
    ArtifactKey key{type, key_agent};
    auto it = artifacts_.find(key);
    if (it == artifacts_.end() || it->second.state != ArtifactState::kReady) {
      return Result<void>::Failure(Error::InvalidArgument(
          "required artifact is not ready for node " +
          std::string(descriptor.name)));
    }
  }
  const auto position = std::find(agents_.begin(), agents_.end(), *agent);
  if (position != agents_.begin() &&
      (node == NodeId::kLoopDetect || node == NodeId::kOptimize)) {
    const char previous = *std::prev(position);
    ArtifactType previous_type = node == NodeId::kLoopDetect
        ? ArtifactType::kOptimizedPoses : ArtifactType::kOptimizerState;
    ArtifactKey key{previous_type,
                    IsPerAgent(previous_type) ? std::optional<char>(previous)
                                              : std::nullopt};
    auto it = artifacts_.find(key);
    if (it == artifacts_.end() || it->second.state != ArtifactState::kReady) {
      return Result<void>::Failure(Error::InvalidArgument(
          "ordered predecessor artifact is not ready"));
    }
  }
  return Result<void>::Ok();
}

void ArtifactRepository::BeginNode(NodeId node, std::optional<char> agent) {
  std::lock_guard lock(mutex_);
  if (!agent) return;
  const auto start = std::find(agents_.begin(), agents_.end(), *agent);
  const auto stale = [&](ArtifactType type, bool from_agent) {
    if (!IsPerAgent(type)) {
      auto& item = artifacts_[ArtifactKey{type, std::nullopt}];
      item.state = ArtifactState::kStale;
      item.revision = next_revision_++;
      item.producer = std::string(DescribeNode(node).name);
      return;
    }
    auto begin = from_agent ? start : agents_.begin();
    for (auto it = begin; it != agents_.end(); ++it) {
      auto& item = artifacts_[ArtifactKey{type, *it}];
      item.state = ArtifactState::kStale;
      item.revision = next_revision_++;
      item.producer = std::string(DescribeNode(node).name);
    }
  };
  if (node == NodeId::kDataLoad || node == NodeId::kLoopDetect) {
    stale(ArtifactType::kDescriptorState, false);
    stale(ArtifactType::kLoopCandidates, true);
    stale(ArtifactType::kOptimizerState, false);
    stale(ArtifactType::kOptimizedPoses, true);
    stale(ArtifactType::kGlobalMap, true);
    stale(ArtifactType::kPoseFile, true);
    stale(ArtifactType::kPcdFile, true);
  } else if (node == NodeId::kOptimize) {
    stale(ArtifactType::kOptimizerState, false);
    stale(ArtifactType::kOptimizedPoses, true);
    stale(ArtifactType::kGlobalMap, true);
    stale(ArtifactType::kPoseFile, true);
    stale(ArtifactType::kPcdFile, true);
  } else if (node == NodeId::kMapUpdate) {
    stale(ArtifactType::kGlobalMap, true);
    stale(ArtifactType::kPcdFile, true);
  }
}

void ArtifactRepository::CompleteNode(NodeId node, std::optional<char> agent) {
  std::lock_guard lock(mutex_);
  if (!agent) return;
  for (auto type : DescribeNode(node).produced_artifacts) {
    std::optional<char> key_agent;
    if (IsPerAgent(type)) key_agent = agent;
    ArtifactKey key{type, key_agent};
    auto& item = artifacts_[key];
    item.key = key;
    item.state = ArtifactState::kReady;
    item.revision = next_revision_++;
    item.producer = std::string(DescribeNode(node).name);
    item.detail.clear();
  }
}

void ArtifactRepository::FailNode(NodeId node, std::optional<char> agent,
                                  std::string detail) {
  std::lock_guard lock(mutex_);
  if (!agent) return;
  for (auto type : DescribeNode(node).produced_artifacts) {
    std::optional<char> key_agent;
    if (IsPerAgent(type)) key_agent = agent;
    ArtifactKey key{type, key_agent};
    auto& item = artifacts_[key];
    item.key = key;
    item.state = ArtifactState::kFailed;
    item.revision = next_revision_++;
    item.producer = std::string(DescribeNode(node).name);
    item.detail = detail;
  }
}

void ArtifactRepository::ApplyConfig(ConfigDomain domain,
                                     uint64_t config_revision) {
  std::lock_guard lock(mutex_);
  auto& config = artifacts_[ArtifactKey{ArtifactType::kConfigSnapshot,
                                        std::nullopt}];
  config.state = ArtifactState::kReady;
  config.revision = config_revision ? config_revision : next_revision_++;
  config.producer = "config_apply";
  std::vector<ArtifactType> stale;
  if (domain == ConfigDomain::kGlobal || domain == ConfigDomain::kDataLoader) {
    stale = {ArtifactType::kRawData, ArtifactType::kDescriptorState,
             ArtifactType::kLoopCandidates, ArtifactType::kOptimizerState,
             ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
             ArtifactType::kPoseFile, ArtifactType::kPcdFile};
  } else if (domain == ConfigDomain::kLoopDetector) {
    stale = {ArtifactType::kDescriptorState, ArtifactType::kLoopCandidates,
             ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses,
             ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
             ArtifactType::kPcdFile};
  } else if (domain == ConfigDomain::kOptimizer) {
    stale = {ArtifactType::kOptimizerState, ArtifactType::kOptimizedPoses,
             ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
             ArtifactType::kPcdFile};
  } else {
    stale = {ArtifactType::kGlobalMap, ArtifactType::kPcdFile};
  }
  setTypes(stale, ArtifactState::kStale, "config_apply");
}


}  // namespace open_lmm
