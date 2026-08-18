#pragma once

#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/server/pipeline_controller.hpp>
#include <open_lmm/server/resource_governor.hpp>

#include <compare>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm {

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

struct RuntimeSessionSnapshot {
  SessionId id;
  std::string label;
  RuntimeSessionState state = RuntimeSessionState::kCreating;
  std::filesystem::path output_directory;
  PipelineSnapshot pipeline;
};

struct SessionExecutionEvent {
  SessionId session_id;
  ExecutionEvent event;
};

class RuntimeService {
 public:
  using PortFactory = std::function<Result<std::shared_ptr<StageRuntimePort>>(
      const BootstrapRequest&, const std::filesystem::path&)>;

  explicit RuntimeService(std::size_t maximum_sessions = 8,
                          PortFactory port_factory = {});
  explicit RuntimeService(ResourceBudget budget,
                          PortFactory port_factory = {});
  ~RuntimeService();
  RuntimeService(const RuntimeService&) = delete;
  RuntimeService& operator=(const RuntimeService&) = delete;

  Result<SessionId> CreateSession(const BootstrapRequest& request);
  Result<JobId> Submit(const SessionId& session_id,
                       const ExecutionRequest& request);
  Result<void> Cancel(const SessionId& session_id, JobId job_id);
  Result<RuntimeSessionSnapshot> Snapshot(const SessionId& session_id) const;
  Result<void> CloseSession(const SessionId& session_id, CloseMode mode);
  Result<ExecutionEventSubscription> SubscribeEvents(
      const SessionId& session_id,
      std::function<void(const SessionExecutionEvent&)> callback);
  [[nodiscard]] std::vector<SessionId> SessionIds() const;
  [[nodiscard]] const ResourceGovernor& Governor() const noexcept {
    return *governor_;
  }

 private:
  struct RuntimeSession;
  Result<std::shared_ptr<RuntimeSession>> lookup(
      const SessionId& session_id) const;
  static bool IsActive(JobState state);
  static RuntimeSessionState DeriveState(const RuntimeSession& session,
                                         const PipelineSnapshot& pipeline);
  static Result<SessionId> GenerateSessionId();
  static std::string GenerateOutputNamespace();

  mutable std::mutex registry_mutex_;
  std::map<SessionId, std::shared_ptr<RuntimeSession>> sessions_;
  std::shared_ptr<ResourceGovernor> governor_;
  PortFactory port_factory_;
};

}  // namespace open_lmm
