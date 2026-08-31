#pragma once

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>

#include <cstddef>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <string_view>

namespace open_lmm {

// Immutable, algorithm-facing representation of the configuration selected at
// the command's base revision.  Core algorithms must not reopen config files.
struct AlgorithmConfigSnapshot {
  std::string schema_id;
  uint32_t schema_version = 0;
  std::string canonical_json;
  std::string fingerprint;
};

struct AlgorithmResourceBudget {
  std::size_t maximum_cpu_threads = 1;
  uint64_t maximum_transient_bytes = 0;
  uint64_t maximum_result_bytes = 0;
};

enum class AlgorithmLogLevel : uint8_t { kDebug, kInfo, kWarning, kError };

using AlgorithmLogger =
    std::function<void(AlgorithmLogLevel, std::string_view)>;
using AlgorithmProfiler =
    std::function<void(std::string_view operation, double elapsed_ms)>;

// Command-scoped services and authority. All pointed-to state is either
// immutable or has an explicitly thread-safe interface.
struct AlgorithmExecutionContext {
  AgentContext agent;
  std::shared_ptr<const AlgorithmConfigSnapshot> config;
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  AlgorithmLogger logger;
  AlgorithmProfiler profiler;
  AlgorithmProgressCallback progress;
  AlgorithmResourceBudget resource_budget;
  uint64_t base_revision = 0;
  std::string operation;
  std::string plugin_id;
};

inline void ReportAlgorithmProgress(
    const AlgorithmExecutionContext& context, AlgorithmProgressPhase phase,
    uint64_t current, std::optional<uint64_t> total = std::nullopt) noexcept {
  if (!context.progress) return;
  try {
    context.progress(
        AlgorithmProgress{context.agent.id, context.operation, phase,
                          current, total});
  } catch (...) {
    // Progress is observational and must not change algorithm semantics.
  }
}

inline Error WithAlgorithmContext(Error error,
                                  const AlgorithmExecutionContext& context) {
  error.context.runtime_revision = context.base_revision;
  if (error.context.stage.empty()) error.context.stage = "algorithm";
  if (error.context.node.empty()) {
    error.context.node =
        context.operation.empty() ? "unspecified" : context.operation;
  }
  if (!error.context.agent) error.context.agent = context.agent.id;
  if (error.context.plugin.empty()) {
    error.context.plugin =
        context.plugin_id.empty() ? "builtin" : context.plugin_id;
  }
  if (error.context.config.empty() && context.config) {
    error.context.config = context.config->schema_id;
  }
  if (context.logger) {
    try {
      context.logger(AlgorithmLogLevel::kError, error.message);
    } catch (...) {
      // Diagnostics are observers; they may not change algorithm semantics.
    }
  }
  return error;
}

class AlgorithmExecutionTimer {
 public:
  explicit AlgorithmExecutionTimer(const AlgorithmExecutionContext& context)
      : context_(context), started_at_(std::chrono::steady_clock::now()) {}
  ~AlgorithmExecutionTimer() noexcept {
    if (!context_.profiler) return;
    const double elapsed_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started_at_)
            .count();
    try {
      context_.profiler(context_.operation, elapsed_ms);
    } catch (...) {
      // Profiling is observational and must not escape the Result boundary.
    }
  }

 private:
  const AlgorithmExecutionContext& context_;
  std::chrono::steady_clock::time_point started_at_;
};

inline Result<void> CheckAlgorithmCancellation(
    const AlgorithmExecutionContext& context, std::string_view boundary) {
  if (context.cancellation &&
      context.cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(WithAlgorithmContext(
        Error::Cancelled(std::string(boundary)), context));
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
