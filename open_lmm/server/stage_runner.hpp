#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <memory>

#include <cstdint>
#include <optional>
#include <string_view>
#include <vector>

namespace open_lmm {

enum class StageId : uint8_t { kDataLoad, kAlignment, kMapUpdate, kSave };
enum class NodeId : uint8_t {
  kDataLoad, kLoopDetect, kOptimize, kMapUpdate, kPoseSave
};
enum class ArtifactType : uint8_t {
  kConfigSnapshot, kAgentInput, kRawData, kDescriptorState,
  kLoopCandidates, kMapAlignment, kOptimizerState, kOptimizedPoses, kGlobalMap,
  kPoseFile, kPcdFile, kProfileRecord,
};
enum class ConfigDomain : uint8_t {
  kGlobal, kDataLoader, kLoopDetector, kOptimizer, kDynamicRemover,
  kMapSave,
};

struct NodeDescriptor {
  NodeId id;
  std::string_view name;
  StageId stage;
  std::vector<ArtifactType> required_artifacts;
  std::vector<ArtifactType> produced_artifacts;
  bool ordered = false;
  bool supports_cancellation = false;
};

[[nodiscard]] const NodeDescriptor& DescribeNode(NodeId node);

class StageRunner {
 public:
  virtual ~StageRunner() = default;
  virtual void SetCancellationToken(std::shared_ptr<CancellationToken> token) = 0;
  virtual void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker>) {}
  virtual Result<void> RunStage(StageId stage) = 0;
  virtual Result<void> RunNode(NodeId node, std::optional<char> agent) = 0;
  virtual Result<void> RunOptimizeThrough(char target_agent) = 0;
  virtual Result<void> Reconfigure(ConfigDomain) { return Result<void>::Ok(); }
  [[nodiscard]] virtual std::vector<char> AgentIds() const = 0;
  [[nodiscard]] virtual Result<VisualizationSnapshot>
  CreateVisualizationSnapshot(char) const {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("visualization snapshots are not supported"));
  }
};

}  // namespace open_lmm
