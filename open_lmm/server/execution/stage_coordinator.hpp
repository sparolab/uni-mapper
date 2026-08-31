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
#include <open_lmm/server/runtime_state_store.hpp>
#include <open_lmm/server/stage_ports.hpp>
#include <open_lmm/server/transaction/runtime_reconfigurer.hpp>

namespace open_lmm {

// Owns stage ordering and the file/state transaction barrier. All policy and
// configuration values are read from the command's immutable base snapshot.
class StageCoordinator {
 public:
  using DataLoadPreviewCallback =
      std::function<void(uint64_t, const AgentId&, const AgentRawDataHandle&)>;

  StageCoordinator(RuntimeStateStore& runtime_states, OutputRepository& outputs,
                   std::shared_ptr<ResourceGovernor> governor,
                   std::shared_ptr<const AlgorithmFactory> algorithms = {},
                   DataLoadPreviewCallback data_load_preview = {});

  Result<void> ExecuteStage(std::shared_ptr<const RuntimeState> base,
                            StageId stage,
                            const ExecutionContext& context);
  Result<void> ExecuteNode(std::shared_ptr<const RuntimeState> base,
                           NodeId node, std::optional<AgentId> agent,
                           const ExecutionContext& context);
  Result<void> ExecuteOptimizeThrough(
      std::shared_ptr<const RuntimeState> base, const AgentId& target,
      const ExecutionContext& context);
  Result<void> ExecuteReconfigure(std::shared_ptr<const RuntimeState> base,
                                  ConfigDomain domain, uint64_t revision,
                                  const ExecutionContext& context);
  Result<ConfigApplyReceipt> ApplyConfig(
      std::shared_ptr<const RuntimeState> base,
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context);

 private:
  using ArtifactMutation =
      std::function<Result<void>(RuntimeState&, ArtifactRepository&,
                                 PendingOutputSet*)>;

  Result<void> CommitCandidate(std::shared_ptr<const RuntimeState> base,
                               ExecutionCandidate candidate,
                               const ExecutionContext& context,
                               PendingOutputSet* pending,
                               ArtifactMutation mutation);
  Result<void> CommitSave(std::shared_ptr<const RuntimeState> base,
                          SaveExecutionMode mode,
                          const ExecutionContext& context);
  Result<void> RecordMapOutputs(const RuntimeState& state,
                                const std::vector<AgentId>& agents,
                                ArtifactRepository& artifacts) const;
  std::unique_ptr<ArtifactRepository> ArtifactEditor(
      const RuntimeState& state) const;

  RuntimeStateStore& runtime_state_store_;
  OutputRepository& outputs_;
  std::shared_ptr<ResourceGovernor> governor_;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
  DataLoadPreviewCallback data_load_preview_;
  DataLoadExecutor data_load_;
  AlignmentExecutor alignment_;
  OptimizeExecutor optimize_;
  MapUpdateExecutor map_update_;
  SaveExecutor save_;
  RuntimeReconfigurer reconfigurer_;
};

}  // namespace open_lmm
