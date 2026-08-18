#include "artifact_repository.hpp"

#include <algorithm>

namespace open_lmm {

void ArtifactRepository::Reset(const std::vector<AgentId>& agents) {
  {
    std::lock_guard lock(mutex_);
    artifacts_.clear();
    agents_.clear();
    next_revision_ = 1;
  }
  RegisterAgents(agents);
}
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
  return ArtifactOwnership(type) == ExecutionScope::kPerAgent;
}

void ArtifactRepository::RegisterAgents(const std::vector<AgentId>& agents) {
  std::lock_guard lock(mutex_);
  agents_ = agents;
  for (auto type : {ArtifactType::kRawData, ArtifactType::kLoopCandidates,
                    ArtifactType::kMapAlignment,
                    ArtifactType::kOptimizedPoses, ArtifactType::kGlobalMap,
                    ArtifactType::kPoseFile, ArtifactType::kPcdFile}) {
    for (const AgentId& agent : agents_) {
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
  for (const AgentId& agent : agents_) {
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
      for (const AgentId& agent : agents_) {
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
  setTypes(AffectedArtifacts(stage), ArtifactState::kStale, StageName(stage));
}

void ArtifactRepository::BeginStage(StageId stage) {
  std::lock_guard lock(mutex_);
  invalidateDownstream(stage);
}
void ArtifactRepository::CompleteStage(StageId stage) {
  std::lock_guard lock(mutex_);
  setTypes(ProducedArtifacts(stage), ArtifactState::kReady, StageName(stage));
}
void ArtifactRepository::FailStage(StageId stage, std::string detail) {
  std::lock_guard lock(mutex_);
  setTypes(ProducedArtifacts(stage), ArtifactState::kFailed, StageName(stage), detail);
}

void ArtifactRepository::CompleteOptimizeThrough(
    const AgentId& target_agent, const std::vector<AgentId>& ordered_agents) {
  std::lock_guard lock(mutex_);
  const auto target = std::find(
      ordered_agents.begin(), ordered_agents.end(), target_agent);
  if (target == ordered_agents.end()) return;
  setTypes({ArtifactType::kOptimizerState}, ArtifactState::kReady,
           "optimizer_replay");
  const auto set = [this](ArtifactType type, std::optional<AgentId> agent,
                          ArtifactState state) {
    ArtifactKey key{type, agent};
    auto& item = artifacts_[key];
    item.key = key;
    item.state = state;
    item.revision = next_revision_++;
    item.producer = "optimizer_replay";
    item.detail.clear();
  };
  set(ArtifactType::kDescriptorState, std::nullopt, ArtifactState::kReady);
  for (auto current = ordered_agents.begin(); current != ordered_agents.end();
       ++current) {
    const bool in_prefix = current <= target;
    for (ArtifactType type : {ArtifactType::kLoopCandidates,
                              ArtifactType::kMapAlignment,
                              ArtifactType::kOptimizedPoses}) {
      set(type, *current,
          in_prefix ? ArtifactState::kReady : ArtifactState::kStale);
    }
  }
  setTypes({ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
            ArtifactType::kPcdFile}, ArtifactState::kStale,
           "optimizer_replay");
}

void ArtifactRepository::CompleteLoopDetectThrough(
    const AgentId& target_agent, const std::vector<AgentId>& ordered_agents) {
  std::lock_guard lock(mutex_);
  const auto target = std::find(
      ordered_agents.begin(), ordered_agents.end(), target_agent);
  if (target == ordered_agents.end()) return;
  const auto set = [this](ArtifactType type, std::optional<AgentId> agent,
                          ArtifactState state) {
    ArtifactKey key{type, agent};
    auto& item = artifacts_[key];
    item.key = key;
    item.state = state;
    item.revision = next_revision_++;
    item.producer = "loop_detect_replay";
    item.detail.clear();
  };
  set(ArtifactType::kDescriptorState, std::nullopt, ArtifactState::kReady);
  for (auto current = ordered_agents.begin(); current != ordered_agents.end();
       ++current) {
    const bool in_loop_prefix = current <= target;
    const bool in_optimizer_prefix = current < target;
    for (ArtifactType type : {ArtifactType::kLoopCandidates,
                              ArtifactType::kMapAlignment}) {
      set(type, *current, in_loop_prefix ? ArtifactState::kReady
                                         : ArtifactState::kStale);
    }
    set(ArtifactType::kOptimizedPoses, *current,
        in_optimizer_prefix ? ArtifactState::kReady : ArtifactState::kStale);
  }
  set(ArtifactType::kOptimizerState, std::nullopt,
      target == ordered_agents.begin() ? ArtifactState::kStale
                                       : ArtifactState::kReady);
  setTypes({ArtifactType::kGlobalMap, ArtifactType::kPoseFile,
            ArtifactType::kPcdFile}, ArtifactState::kStale,
           "loop_detect_replay");
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
  uint64_t next_revision = 1;
  for (const auto& item : snapshot) {
    artifacts_[item.key] = item;
    next_revision = std::max(next_revision, item.revision + 1);
  }
  next_revision_ = next_revision;
}

void ArtifactRepository::RecordExternalFile(
    ArtifactType type, AgentId agent, std::string path,
    std::string fingerprint) {
  std::lock_guard lock(mutex_);
  ArtifactKey key{type, std::nullopt};
  if (IsPerAgent(type)) key.agent = agent;
  auto& item = artifacts_[key];
  item.key = key;
  item.state = ArtifactState::kReady;
  item.revision = next_revision_++;
  item.producer = "output_repository";
  item.detail.clear();
  item.external_path = std::move(path);
  item.fingerprint = std::move(fingerprint);
}

Result<void> ArtifactRepository::ValidateNode(
    NodeId node, std::optional<AgentId> agent) const {
  std::lock_guard lock(mutex_);
  auto execution_agents = executionAgentsLocked(node, agent);
  if (!execution_agents) {
    return Result<void>::Failure(execution_agents.GetError());
  }
  return Result<void>::Ok();
}

Result<std::vector<AgentId>> ArtifactRepository::ExecutionAgents(
    NodeId node, std::optional<AgentId> agent) const {
  std::lock_guard lock(mutex_);
  return executionAgentsLocked(node, agent);
}

Result<std::vector<AgentId>> ArtifactRepository::executionAgentsLocked(
    NodeId node, std::optional<AgentId> agent) const {
  const auto& spec = ExecutionSpecFor(node);
  std::vector<AgentId> execution_agents;
  if (spec.scope == ExecutionScope::kSession) {
    for (ArtifactType type : spec.required_artifacts) {
      if (IsPerAgent(type)) continue;
      const auto found = artifacts_.find({type, std::nullopt});
      if (found == artifacts_.end() ||
          found->second.state != ArtifactState::kReady) {
        return Result<std::vector<AgentId>>::Failure(Error::InvalidArgument(
            "required session artifact is not ready for node " +
            std::string(spec.name)));
      }
    }
    for (const AgentId& candidate : agents_) {
      const bool ready = std::all_of(
          spec.required_artifacts.begin(), spec.required_artifacts.end(),
          [&](ArtifactType type) {
            if (!IsPerAgent(type)) return true;
            const auto found = artifacts_.find({type, candidate});
            return found != artifacts_.end() &&
                   found->second.state == ArtifactState::kReady;
          });
      if (ready) execution_agents.push_back(candidate);
    }
    if (execution_agents.empty()) {
      return Result<std::vector<AgentId>>::Failure(Error::InvalidArgument(
          "no ready agent artifacts for session node " +
          std::string(spec.name)));
    }
    return Result<std::vector<AgentId>>::Ok(std::move(execution_agents));
  }
  if (!agent) {
    return Result<std::vector<AgentId>>::Failure(Error::InvalidArgument(
        "per-agent node command requires an agent"));
  }
  if (std::find(agents_.begin(), agents_.end(), *agent) == agents_.end()) {
    return Result<std::vector<AgentId>>::Failure(
        Error::InvalidArgument("unknown agent"));
  }
  execution_agents = {*agent};
  if (spec.ordered) {
    auto prefix = OrderedAgentPrefix(agents_, *agent);
    if (!prefix) {
      return Result<std::vector<AgentId>>::Failure(prefix.GetError());
    }
    execution_agents = std::move(prefix).Value();
  }
  for (const AgentId& execution_agent : execution_agents) {
    for (auto type : spec.required_artifacts) {
      std::optional<AgentId> key_agent;
      if (IsPerAgent(type)) key_agent = execution_agent;
      ArtifactKey key{type, key_agent};
      auto it = artifacts_.find(key);
      if (it == artifacts_.end() || it->second.state != ArtifactState::kReady) {
        return Result<std::vector<AgentId>>::Failure(Error::InvalidArgument(
            "required artifact is not ready for node " +
            std::string(spec.name)));
      }
    }
  }
  return Result<std::vector<AgentId>>::Ok(std::move(execution_agents));
}

void ArtifactRepository::BeginNode(NodeId node, std::optional<AgentId> agent) {
  std::lock_guard lock(mutex_);
  auto affected_agents = executionAgentsLocked(node, agent);
  if (!affected_agents) return;
  beginNodeLocked(node, affected_agents.Value());
}

void ArtifactRepository::BeginNode(
    NodeId node, const std::vector<AgentId>& affected_agents) {
  std::lock_guard lock(mutex_);
  beginNodeLocked(node, affected_agents);
}

void ArtifactRepository::beginNodeLocked(
    NodeId node, const std::vector<AgentId>& affected_agents) {
  if (affected_agents.empty()) return;
  const auto& spec = ExecutionSpecFor(node);
  const auto start = spec.scope == ExecutionScope::kSession
      ? agents_.begin()
      : std::find(agents_.begin(), agents_.end(), affected_agents.back());
  const auto stale = [&](ArtifactType type, bool from_agent) {
    if (!IsPerAgent(type)) {
      auto& item = artifacts_[ArtifactKey{type, std::nullopt}];
      item.state = ArtifactState::kStale;
      item.revision = next_revision_++;
      item.producer = std::string(DescribeNode(node).name);
      return;
    }
    auto begin = from_agent && spec.ordered ? start : agents_.begin();
    for (auto it = begin; it != agents_.end(); ++it) {
      const bool explicitly_affected =
          std::find(affected_agents.begin(), affected_agents.end(), *it) !=
          affected_agents.end();
      if ((spec.scope == ExecutionScope::kSession || !spec.ordered) &&
          !explicitly_affected) continue;
      auto& item = artifacts_[ArtifactKey{type, *it}];
      item.state = ArtifactState::kStale;
      item.revision = next_revision_++;
      item.producer = std::string(DescribeNode(node).name);
    }
  };
  for (ArtifactType type : AffectedArtifacts(node)) {
    const bool shared = !IsPerAgent(type);
    stale(type, !shared);
  }
}

void ArtifactRepository::CompleteNode(NodeId node, std::optional<AgentId> agent) {
  std::lock_guard lock(mutex_);
  if (ExecutionSpecFor(node).scope == ExecutionScope::kSession) {
    auto affected_agents = executionAgentsLocked(node, agent);
    if (!affected_agents) return;
    completeNodeLocked(node, affected_agents.Value(), ArtifactState::kReady,
                       {});
    return;
  }
  if (!agent) return;
  completeNodeLocked(node, std::vector<AgentId>{*agent},
                     ArtifactState::kReady, {});
}

void ArtifactRepository::CompleteNode(
    NodeId node, const std::vector<AgentId>& affected_agents) {
  std::lock_guard lock(mutex_);
  completeNodeLocked(node, affected_agents, ArtifactState::kReady, {});
}

void ArtifactRepository::completeNodeLocked(
    NodeId node, const std::vector<AgentId>& affected_agents,
    ArtifactState state, const std::string& detail) {
  for (auto type : DescribeNode(node).produced_artifacts) {
    const std::vector<std::optional<AgentId>> owners = IsPerAgent(type)
        ? [&] {
            std::vector<std::optional<AgentId>> result;
            result.reserve(affected_agents.size());
            for (const auto& item : affected_agents) result.emplace_back(item);
            return result;
          }()
        : std::vector<std::optional<AgentId>>{std::nullopt};
    for (const auto& owner : owners) {
      ArtifactKey key{type, owner};
      auto& item = artifacts_[key];
      item.key = key;
      item.state = state;
      item.revision = next_revision_++;
      item.producer = std::string(DescribeNode(node).name);
      item.detail = detail;
    }
  }
}

void ArtifactRepository::FailNode(NodeId node, std::optional<AgentId> agent,
                                  std::string detail) {
  std::lock_guard lock(mutex_);
  if (ExecutionSpecFor(node).scope == ExecutionScope::kSession) {
    auto affected_agents = executionAgentsLocked(node, agent);
    if (!affected_agents) return;
    completeNodeLocked(node, affected_agents.Value(), ArtifactState::kFailed,
                       detail);
    return;
  }
  if (!agent) return;
  completeNodeLocked(node, std::vector<AgentId>{*agent},
                     ArtifactState::kFailed, detail);
}

void ArtifactRepository::FailNode(
    NodeId node, const std::vector<AgentId>& affected_agents,
    std::string detail) {
  std::lock_guard lock(mutex_);
  completeNodeLocked(node, affected_agents, ArtifactState::kFailed, detail);
}

void ArtifactRepository::ApplyConfig(ConfigDomain domain,
                                     uint64_t config_revision) {
  std::lock_guard lock(mutex_);
  auto& config = artifacts_[ArtifactKey{ArtifactType::kConfigSnapshot,
                                        std::nullopt}];
  config.state = ArtifactState::kReady;
  config.revision = config_revision ? config_revision : next_revision_++;
  config.producer = "config_apply";
  setTypes(AffectedArtifacts(domain), ArtifactState::kStale, "config_apply");
}


}  // namespace open_lmm
