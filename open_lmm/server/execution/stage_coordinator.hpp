#pragma once

#include <functional>
#include <memory>

#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/execution/alignment_executor.hpp>
#include <open_lmm/server/execution/data_load_executor.hpp>
#include <open_lmm/server/execution/map_update_executor.hpp>
#include <open_lmm/server/execution/optimize_executor.hpp>
#include <open_lmm/server/execution/save_executor.hpp>
#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/server/session_manager.hpp>
#include <open_lmm/server/stage_ports.hpp>
#include <open_lmm/server/transaction/session_reconfigurer.hpp>

namespace open_lmm {

// Owns stage ordering and the file/state transaction barrier. All policy and
// configuration values are read from the command's immutable base snapshot.
class StageCoordinator {
 public:
  StageCoordinator(SessionManager& sessions, OutputRepository& outputs,
                   std::shared_ptr<ResourceGovernor> governor,
                   std::shared_ptr<const AlgorithmFactory> algorithms = {});

  Result<void> ExecuteStage(std::shared_ptr<const SessionState> base,
                            StageId stage,
                            const ExecutionContext& context);
  Result<void> ExecuteNode(std::shared_ptr<const SessionState> base,
                           NodeId node, std::optional<AgentId> agent,
                           const ExecutionContext& context);
  Result<void> ExecuteOptimizeThrough(
      std::shared_ptr<const SessionState> base, const AgentId& target,
      const ExecutionContext& context);
  Result<void> ExecuteReconfigure(std::shared_ptr<const SessionState> base,
                                  ConfigDomain domain, uint64_t revision,
                                  const ExecutionContext& context);

 private:
  using ArtifactMutation =
      std::function<Result<void>(SessionState&, ArtifactRepository&,
                                 PendingOutputSet*)>;

  Result<void> CommitCandidate(std::shared_ptr<const SessionState> base,
                               ExecutionCandidate candidate,
                               const ExecutionContext& context,
                               PendingOutputSet* pending,
                               ArtifactMutation mutation);
  Result<void> CommitSave(std::shared_ptr<const SessionState> base,
                          SaveExecutionMode mode,
                          const ExecutionContext& context);
  Result<void> RecordMapOutputs(const SessionState& state,
                                const std::vector<AgentId>& agents,
                                ArtifactRepository& artifacts) const;
  std::unique_ptr<ArtifactRepository> ArtifactEditor(
      const SessionState& state) const;

  SessionManager& sessions_;
  OutputRepository& outputs_;
  std::shared_ptr<ResourceGovernor> governor_;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
  DataLoadExecutor data_load_;
  AlignmentExecutor alignment_;
  OptimizeExecutor optimize_;
  MapUpdateExecutor map_update_;
  SaveExecutor save_;
  SessionReconfigurer reconfigurer_;
};

}  // namespace open_lmm
