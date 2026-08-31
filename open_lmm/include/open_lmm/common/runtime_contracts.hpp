#pragma once

#include <open_lmm/common/result.hpp>

#include <cstdint>
#include <compare>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace open_lmm {

using JobId = uint64_t;

// Public job identity for the single-runtime API.  It is intentionally
// independent of PipelineController's per-instance JobId: a root-config
// replacement may create a new controller whose local IDs start again at 1.
struct JobHandle {
  uint64_t value = 0;
  auto operator<=>(const JobHandle&) const = default;
};

enum class StageId : uint8_t { kDataLoad, kAlignment, kMapUpdate, kSave };
enum class NodeId : uint8_t {
  kDataLoad, kLoopDetect, kOptimize, kMapUpdate, kPoseSave, kFallbackMapSave
};
enum class ArtifactType : uint8_t {
  kConfigSnapshot, kAgentInput, kRawData, kDescriptorState,
  kLoopCandidates, kMapAlignment, kOptimizerState, kOptimizedPoses, kGlobalMap,
  kPoseFile, kPcdFile, kProfileRecord,
};
enum class ArtifactState : uint8_t { kMissing, kReady, kStale, kFailed };
enum class ExecutionScope : uint8_t { kPerAgent, kRuntime };

struct ArtifactKey {
  ArtifactType type;
  std::optional<AgentId> agent;
  auto operator<=>(const ArtifactKey&) const = default;
};

struct ArtifactMetadata {
  ArtifactKey key;
  ArtifactState state = ArtifactState::kMissing;
  uint64_t revision = 0;
  std::string producer;
  std::string detail;
  std::string external_path;
  std::string fingerprint;
};

struct NodeDescriptor {
  NodeId id;
  std::string_view name;
  StageId stage;
  ExecutionScope scope;
  std::vector<ArtifactType> required_artifacts;
  std::vector<ArtifactType> produced_artifacts;
  bool ordered = false;
  bool supports_cancellation = false;
};

enum class ExecutionRequestKind : uint8_t {
  kRunAll,
  kStage,
  kNode,
  kOptimizeThrough,
};

struct ExecutionRequest {
  ExecutionRequestKind kind = ExecutionRequestKind::kRunAll;
  std::optional<StageId> stage;
  std::optional<NodeId> node;
  std::optional<AgentId> agent;
};

struct BootstrapRequest {
  std::filesystem::path config_directory;
  std::string label;
  std::optional<std::filesystem::path> output_root;
};

enum class RuntimeStatus : uint8_t {
  kCreating,
  kReady,
  kRunning,
  kCancelling,
  kFailedRecoverable,
  kFailedFatal,
  kClosing,
  kClosed,
};

enum class CloseMode : uint8_t { kRejectIfRunning, kCancelAndWait };

}  // namespace open_lmm
