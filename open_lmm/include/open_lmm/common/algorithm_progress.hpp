#pragma once

#include <open_lmm/common/agent_id.hpp>

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <string_view>

namespace open_lmm {

enum class AlgorithmProgressPhase : uint8_t {
  kEnumerate,
  kReadAndFilter,
  kBuildPreview,
  kLoadRemoverInput,
  kReadAndRunRemover,
  kBuildRawMap,
  kInitializeRemover,
  kRunRemover,
  kBuildStaticMap,
  kDetectLoops,
  kOptimizeGraph,
  kWriteOutput,
  kBuildAlignmentMap,
  kDetectIntraLoops,
  kRunKissMatcher,
  kDetectInterLoops,
  kBuildDescriptorConsensus,
  kWaitAlignmentReview,
  kBuildLoopConstraints,
  kRegisterIntraLoops,
  kRegisterInterLoops,
  kSolveGraph,
};

struct AlgorithmProgress {
  AgentId agent;
  std::string operation;
  AlgorithmProgressPhase phase = AlgorithmProgressPhase::kEnumerate;
  uint64_t current = 0;
  std::optional<uint64_t> total;
};

using AlgorithmProgressCallback = std::function<void(const AlgorithmProgress&)>;

[[nodiscard]] inline std::string_view DescribeAlgorithmProgressPhase(
    AlgorithmProgressPhase phase) {
  switch (phase) {
    case AlgorithmProgressPhase::kEnumerate: return "enumerate input scans";
    case AlgorithmProgressPhase::kReadAndFilter: return "read and filter scans";
    case AlgorithmProgressPhase::kBuildPreview: return "build preview";
    case AlgorithmProgressPhase::kLoadRemoverInput: return "load remover input";
    case AlgorithmProgressPhase::kReadAndRunRemover:
      return "read and run remover";
    case AlgorithmProgressPhase::kBuildRawMap: return "build remover raw map";
    case AlgorithmProgressPhase::kInitializeRemover: return "initialize remover";
    case AlgorithmProgressPhase::kRunRemover:
      return "run remover frame calls";
    case AlgorithmProgressPhase::kBuildStaticMap: return "build static map";
    case AlgorithmProgressPhase::kDetectLoops: return "detect loops";
    case AlgorithmProgressPhase::kOptimizeGraph: return "optimize graph";
    case AlgorithmProgressPhase::kWriteOutput: return "write output";
    case AlgorithmProgressPhase::kBuildAlignmentMap:
      return "build alignment map";
    case AlgorithmProgressPhase::kDetectIntraLoops:
      return "detect intra loops";
    case AlgorithmProgressPhase::kRunKissMatcher: return "run KISS Matcher";
    case AlgorithmProgressPhase::kDetectInterLoops:
      return "detect inter loops";
    case AlgorithmProgressPhase::kBuildDescriptorConsensus:
      return "build descriptor consensus";
    case AlgorithmProgressPhase::kWaitAlignmentReview:
      return "wait for alignment review";
    case AlgorithmProgressPhase::kBuildLoopConstraints:
      return "build loop constraints";
    case AlgorithmProgressPhase::kRegisterIntraLoops:
      return "register intra loops";
    case AlgorithmProgressPhase::kRegisterInterLoops:
      return "register inter loops";
    case AlgorithmProgressPhase::kSolveGraph: return "solve graph";
  }
  return "unknown";
}

[[nodiscard]] inline std::string FormatAlgorithmProgressStatus(
    const AlgorithmProgress& progress) {
  std::string status(DescribeAlgorithmProgressPhase(progress.phase));
  if (progress.total) {
    status += ' ';
    status += std::to_string(progress.current);
    status += '/';
    status += std::to_string(*progress.total);
  } else {
    status += " (in progress; total unavailable)";
  }
  return status;
}

}  // namespace open_lmm
