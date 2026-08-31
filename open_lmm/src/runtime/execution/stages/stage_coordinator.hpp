#pragma once

#include <functional>
#include <memory>

#include <plugins/host/algorithm_provider.hpp>
#include <runtime/execution/stages/alignment_executor.hpp>
#include <runtime/execution/stages/data_load_executor.hpp>
#include <runtime/execution/stages/map_update_executor.hpp>
#include <runtime/execution/stages/optimize_executor.hpp>
#include <runtime/execution/stages/save_executor.hpp>
#include <storage/transactions/output_repository.hpp>
#include <runtime/state/runtime_state_store.hpp>
#include <runtime/execution/stage_ports.hpp>
#include <config/application/runtime_reconfigurer.hpp>

namespace open_lmm {

// Owns stage ordering and the file/state transaction barrier. All policy and
// configuration values are read from the command's immutable base snapshot.
class StageCoordinator {
 public:
  using DataLoadPreviewCallback =
      std::function<void(uint64_t, const AgentId&, const AgentRawDataHandle&)>;

  StageCoordinator(RuntimeStateStore& runtime_states, OutputRepository& outputs,
                   std::shared_ptr<ResourceGovernor> governor,
                   std::shared_ptr<const AlgorithmProvider> algorithms,
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
  std::shared_ptr<const AlgorithmProvider> algorithms_;
  DataLoadPreviewCallback data_load_preview_;
  DataLoadExecutor data_load_;
  AlignmentExecutor alignment_;
  OptimizeExecutor optimize_;
  MapUpdateExecutor map_update_;
  SaveExecutor save_;
  RuntimeReconfigurer reconfigurer_;
};

}  // namespace open_lmm
