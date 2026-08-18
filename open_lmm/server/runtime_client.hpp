#pragma once

#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/common/runtime_api.hpp>
#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace open_lmm {

struct ClientSessionSnapshot {
  SessionId id;
  std::string label;
  RuntimeSessionState state = RuntimeSessionState::kCreating;
  std::filesystem::path output_directory;
};

class RuntimeClient {
 public:
  explicit RuntimeClient(std::size_t maximum_sessions = 8);
  ~RuntimeClient();
  RuntimeClient(RuntimeClient&&) noexcept;
  RuntimeClient& operator=(RuntimeClient&&) noexcept;
  RuntimeClient(const RuntimeClient&) = delete;
  RuntimeClient& operator=(const RuntimeClient&) = delete;

  Result<SessionId> CreateSession(const BootstrapRequest& request);
  Result<SessionId> CreateSession(const BootstrapRequest& request,
                                  const ConfigCandidate& root_candidate);
  Result<RuntimeSessionReplacement> ReplaceSession(
      const SessionId& previous_session, const BootstrapRequest& request,
      const ConfigCandidate& root_candidate,
      std::function<void(const SessionExecutionEvent&)> callback);
  Result<JobId> Submit(const SessionId& session_id,
                       const ExecutionRequest& request);
  Result<JobId> SubmitRunAll(const SessionId& session_id);
  Result<void> Cancel(const SessionId& session_id, JobId job_id);
  Result<void> Wait(const SessionId& session_id, JobId job_id);
  Result<ClientSessionSnapshot> Snapshot(const SessionId& session_id) const;
  Result<RuntimeSessionSnapshot> RuntimeSnapshot(
      const SessionId& session_id) const;
  Result<std::vector<NodeDescriptor>> NodeDescriptors(
      const SessionId& session_id) const;
  Result<open_lmm::VisualizationSnapshot> VisualizationSnapshot(
      const SessionId& session_id, const AgentId& agent) const;
  Result<std::optional<open_lmm::AlignmentFeedbackSnapshot>>
  AlignmentFeedbackSnapshot(const SessionId& session_id) const;
  Result<void> RespondToAlignment(const SessionId& session_id, JobId job_id,
                                  AlignmentResponse response);
  Result<void> SetAlignmentFeedbackEnabled(const SessionId& session_id,
                                           bool enabled);
  Result<ConfigApplyReceipt> ApplyConfig(
      const SessionId& session_id, const ConfigCandidate& candidate,
      const ExpectedRevision& expected);
  Result<ExecutionEventSubscription> SubscribeEvents(
      const SessionId& session_id,
      std::function<void(const SessionExecutionEvent&)> callback);
  Result<void> CloseSession(const SessionId& session_id, CloseMode mode);
  [[nodiscard]] std::vector<SessionId> SessionIds() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace open_lmm
