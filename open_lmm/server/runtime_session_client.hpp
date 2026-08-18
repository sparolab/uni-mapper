#pragma once

#include <open_lmm/server/runtime_client.hpp>

#include <functional>
#include <memory>

namespace open_lmm {

struct BoundJob {
  SessionId session_id;
  JobId job_id = 0;
};

class RuntimeSessionClient {
 public:
  static Result<std::shared_ptr<RuntimeSessionClient>> Create(
      std::shared_ptr<RuntimeClient> runtime, SessionId session_id);
  ~RuntimeSessionClient();

  [[nodiscard]] SessionId CurrentSession() const;
  Result<void> ReplaceSession(const BootstrapRequest& request,
                              const ConfigCandidate& root_candidate);
  Result<BoundJob> Submit(const ExecutionRequest& request);
  Result<void> Cancel(const BoundJob& job);
  Result<void> CancelCurrent(JobId job_id);
  Result<void> Wait(const BoundJob& job);
  Result<RuntimeSessionSnapshot> Snapshot() const;
  Result<std::vector<NodeDescriptor>> NodeDescriptors() const;
  Result<VisualizationSnapshot> VisualizationSnapshotFor(
      const AgentId& agent) const;
  Result<std::optional<AlignmentFeedbackSnapshot>>
  AlignmentFeedbackSnapshotFor() const;
  Result<void> RespondToAlignment(const BoundJob& job,
                                  AlignmentResponse response);
  Result<ConfigApplyReceipt> ApplyConfig(
      const SessionId& expected_session, const ConfigCandidate& candidate,
      const ExpectedRevision& expected);
  Result<ExecutionEventSubscription> Subscribe(
      std::function<void(const SessionExecutionEvent&)> callback);

 private:
  struct Impl;
  explicit RuntimeSessionClient(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
};

}  // namespace open_lmm
