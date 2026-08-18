#pragma once

#include <open_lmm/common/result.hpp>

#include <compare>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace open_lmm {

class SessionId {
 public:
  [[nodiscard]] static Result<SessionId> Parse(std::string_view value);
  [[nodiscard]] const std::string& Value() const noexcept { return value_; }
  friend bool operator==(const SessionId&, const SessionId&) = default;
  friend auto operator<=>(const SessionId&, const SessionId&) = default;

 private:
  explicit SessionId(std::string value) : value_(std::move(value)) {}
  std::string value_;
  friend class RuntimeService;
};

using JobId = uint64_t;

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
enum class ExecutionScope : uint8_t { kPerAgent, kSession };

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

enum class RuntimeSessionState : uint8_t {
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
