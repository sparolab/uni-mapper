#include "stage_coordinator.hpp"

#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <utility>

#include <runtime/execution/stages/alignment_artifact_store.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>

namespace open_lmm {
namespace {

void HashBytes(uint64_t& hash, const char* data, std::size_t size) {
  constexpr uint64_t kFnvPrime = 1099511628211ULL;
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= static_cast<unsigned char>(data[index]);
    hash *= kFnvPrime;
  }
}

bool HashFile(uint64_t& hash, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return false;
  char buffer[8192];
  while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
    HashBytes(hash, buffer, static_cast<std::size_t>(input.gcount()));
  }
  return true;
}

std::string HexFingerprint(uint64_t hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

const RuntimeConfigDocument* ConfigDocumentFor(
    const RuntimeConfigDocuments& documents, ConfigDomain domain) {
  switch (domain) {
    case ConfigDomain::kLoopDetector: return &documents.loop_detector;
    case ConfigDomain::kOptimizer: return &documents.optimizer;
    case ConfigDomain::kDynamicRemover: return &documents.dynamic_remover;
    case ConfigDomain::kMapSave: return &documents.map_server;
    case ConfigDomain::kGlobal:
    case ConfigDomain::kDataLoader: return nullptr;
  }
  return nullptr;
}

Result<void> ValidateInvocation(const std::shared_ptr<const RuntimeState>& base,
                                const ExecutionContext& context) {
  if (!base || !base->config || !base->payload || !base->payload->database) {
    return Result<void>::Failure(
        Error::InvalidArgument("stage command has no complete base snapshot"));
  }
  if (!context.cancellation) {
    return Result<void>::Failure(
        Error::InvalidArgument("stage command requires cancellation context"));
  }
  if (context.base_revision != base->revision) {
    return Result<void>::Failure(Error::InvalidArgument(
        "stage command base revision does not match runtime snapshot"));
  }
  return Result<void>::Ok();
}

Result<RuntimeCommitOutcome> CommitPending(PendingOutputSet* pending,
                                           uint64_t committed_revision) {
  if (!pending) return Result<RuntimeCommitOutcome>::Ok({});
  // Allocate the health holder before the file-set commit. Once the final
  // files are installed, moving the recovery payload into this holder and
  // publishing the candidate must not introduce a new allocation failure.
  auto recovery = std::make_shared<Error>(
      Error::IoError("pending file-set recovery outcome"));
  auto outcome = pending->Commit();
  if (!outcome) {
    return Result<RuntimeCommitOutcome>::Failure(outcome.GetError());
  }
  auto committed = std::move(outcome).Value();
  if (!committed.recovery_required) {
    return Result<RuntimeCommitOutcome>::Ok({});
  }
  *recovery = std::move(*committed.recovery_required);
  recovery->MarkFatalRuntime().WithRuntimeRevision(committed_revision);
  return Result<RuntimeCommitOutcome>::Ok({std::move(recovery)});
}

Error OptimizerFactoryError(const Error& source, const RuntimeState& state,
                            const ExecutionContext& command) {
  if (state.config && state.config->documents && state.config->optimizer) {
    auto context = MakeAlgorithmExecutionContext(
        state, command, {}, state.config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimizer_factory",
        state.config->optimizer->type);
    return WithAlgorithmContext(source, context);
  }
  Error error = source;
  error.WithRuntimeRevision(state.revision)
      .WithExecution("algorithm", "optimizer_factory");
  return error;
}

}  // namespace

StageCoordinator::StageCoordinator(
    RuntimeStateStore& runtime_states, OutputRepository& outputs,
    std::shared_ptr<ResourceGovernor> governor,
    std::shared_ptr<const AlgorithmProvider> algorithms,
    DataLoadPreviewCallback data_load_preview,
    AlignmentPreviewCallback alignment_preview)
    : runtime_state_store_(runtime_states),
      outputs_(outputs),
      governor_(std::move(governor)),
      algorithms_(std::move(algorithms)),
      data_load_preview_(std::move(data_load_preview)),
      alignment_(algorithms_, std::move(alignment_preview)),
      optimize_(algorithms_),
      reconfigurer_(algorithms_) {}

std::unique_ptr<ArtifactRepository> StageCoordinator::ArtifactEditor(
    const RuntimeState& state) const {
  auto artifacts = std::make_unique<ArtifactRepository>();
  artifacts->Reset(state.ordered_agents);
  artifacts->Restore(state.artifacts);
  return artifacts;
}

Result<void> StageCoordinator::CommitCandidate(
    std::shared_ptr<const RuntimeState> base, ExecutionCandidate candidate,
    const ExecutionContext& context, PendingOutputSet* pending,
    ArtifactMutation mutation) {
  if (!base || candidate.base_revision != base->revision ||
      !candidate.payload) {
    return Result<void>::Failure(
        Error::InvalidArgument("executor returned a stale or empty candidate"));
  }
  RuntimeTransaction transaction(base);
  transaction.SetPayload(std::move(candidate.payload));
  auto artifacts = ArtifactEditor(*base);
  auto mutated = mutation(transaction.Working(), *artifacts, pending);
  if (!mutated) return mutated;
  transaction.Working().artifacts = artifacts->Snapshot();
  auto finalized = std::move(transaction).Finalize(context.cancellation);
  if (!finalized) return Result<void>::Failure(finalized.GetError());
  auto finalized_state = std::move(finalized).Value();
  const uint64_t committed_revision = finalized_state->revision;
  auto committed = runtime_state_store_.CommitWithBarrier(
      base, std::move(finalized_state), [pending, committed_revision] {
        return CommitPending(pending, committed_revision);
      });
  return committed ? Result<void>::Ok()
                   : Result<void>::Failure(committed.GetError());
}

Result<void> StageCoordinator::RecordMapOutputs(
    const RuntimeState& state, const std::vector<AgentId>& agents,
    ArtifactRepository& artifacts) const {
  for (const AgentId& agent : agents) {
    const auto final_path = state.config->root.output_directory /
                            ("global_map_" + agent.Value() + ".pcd");
    auto temporary = final_path;
    temporary += ".tmp";
    uint64_t hash = 14695981039346656037ULL;
    if (!HashFile(hash, temporary)) {
      return Result<void>::Failure(Error::IoError(
          "MapUpdate temporary output is unavailable: " +
          temporary.string()));
    }
    for (ArtifactType type : {ArtifactType::kGlobalMap,
                              ArtifactType::kPcdFile}) {
      artifacts.RecordExternalFile(type, agent, final_path.string(),
                                   HexFingerprint(hash));
    }
  }
  return Result<void>::Ok();
}

Result<void> StageCoordinator::ExecuteStage(
    std::shared_ptr<const RuntimeState> base, StageId stage,
    const ExecutionContext& context) {
  auto valid = ValidateInvocation(base, context);
  if (!valid) return valid;

  if (stage == StageId::kDataLoad) {
    auto optimizer = algorithms_->CreateOptimizer(*base->config->optimizer);
    if (!optimizer) {
      return Result<void>::Failure(
          OptimizerFactoryError(optimizer.GetError(), *base, context));
    }
    auto contexts = base->payload->contexts;
    for (auto& item : contexts) {
      item.raw_data.reset();
      item.loop_output.reset();
      item.flow = ControlFlow::kContinue;
      item.cancellation = context.cancellation;
    }
    auto database = std::make_shared<SharedDatabase>();
    database->stored_alignments = base->payload->database->stored_alignments;
    database->alignment_feedback = context.alignment_feedback;
    auto candidate = data_load_.Execute(
        {base, std::move(contexts), std::move(database), governor_,
         context.cancellation, context.progress,
         std::move(optimizer).Value(), algorithms_,
         static_cast<float>(base->config->root.save_voxel_size),
         base->config->root.parallel_data_load,
         base->config->root.max_parallel_agents,
         [this, candidate_revision = base->revision](
             const AgentId& agent, const AgentRawDataHandle& raw,
             const VisualizationPointPreviewHandle& preview) {
           if (data_load_preview_) {
             data_load_preview_(candidate_revision, agent, raw, preview);
           }
         }});
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, nullptr,
        [](RuntimeState&, ArtifactRepository& artifacts, PendingOutputSet*) {
          artifacts.BeginStage(StageId::kDataLoad);
          artifacts.CompleteStage(StageId::kDataLoad);
          return Result<void>::Ok();
        });
  }

  if (stage == StageId::kAlignment) {
    auto candidate = alignment_.ExecuteStage(base, context);
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    const auto completed_agents = candidate.Value().execution_agents;
    const auto excluded_agents = candidate.Value().excluded_agents;
    auto pending = outputs_.Begin();
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, &pending,
        [completed_agents, excluded_agents](
            RuntimeState& working, ArtifactRepository& artifacts,
            PendingOutputSet* files) {
          artifacts.BeginStage(StageId::kAlignment);
          // Per-agent failures remain explicit while the shared descriptor and
          // optimizer artifacts finish ready through at least the anchor.
          if (!excluded_agents.empty()) {
            constexpr const char* detail =
                "excluded from alignment by explicit user decision";
            artifacts.FailNode(NodeId::kLoopDetect, excluded_agents, detail);
            artifacts.FailNode(NodeId::kOptimize, excluded_agents, detail);
          }
          artifacts.CompleteNode(NodeId::kLoopDetect, completed_agents);
          artifacts.CompleteNode(NodeId::kOptimize, completed_agents);
          auto store = AlignmentArtifactStore::FromCommitted(working);
          if (!store) return Result<void>::Failure(store.GetError());
          return store.Value().Prepare(
              working, working.config->root.output_directory, *files,
              artifacts);
        });
  }

  if (stage == StageId::kMapUpdate) {
    if (!base->config->root.enable_map_updater) {
      ExecutionCandidate skipped{base->revision, base->payload, {},
                                 ArtifactCompletionKind::kMapUpdateStage,
                                 std::nullopt};
      return CommitCandidate(
          std::move(base), std::move(skipped), context, nullptr,
          [](RuntimeState&, ArtifactRepository&, PendingOutputSet*) {
            return Result<void>::Ok();
          });
    }
    auto pending = outputs_.Begin();
    auto candidate = map_update_.Execute(
        {base, governor_, context.cancellation, context.progress, algorithms_,
         base->config->root.output_directory,
         base->config->root.save_voxel_size,
         base->config->root.parallel_map_update,
         base->config->root.max_parallel_agents, &pending});
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    const auto agents = candidate.Value().execution_agents;
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, &pending,
        [this, agents](RuntimeState& working, ArtifactRepository& artifacts,
                       PendingOutputSet*) {
          artifacts.BeginStage(StageId::kMapUpdate);
          artifacts.CompleteStage(StageId::kMapUpdate);
          return RecordMapOutputs(working, agents, artifacts);
        });
  }

  return CommitSave(std::move(base), SaveExecutionMode::kStage, context);
}

Result<void> StageCoordinator::ExecuteNode(
    std::shared_ptr<const RuntimeState> base, NodeId node,
    std::optional<AgentId> agent, const ExecutionContext& context) {
  auto valid = ValidateInvocation(base, context);
  if (!valid) return valid;
  auto current_artifacts = ArtifactEditor(*base);
  auto execution_agents =
      node == NodeId::kFallbackMapSave &&
              base->config->root.enable_map_updater
          ? Result<std::vector<AgentId>>::Ok({})
          : current_artifacts->ExecutionAgents(node, agent);
  if (!execution_agents) {
    return Result<void>::Failure(execution_agents.GetError());
  }

  if (node == NodeId::kDataLoad) {
    auto optimizer = algorithms_->CreateOptimizer(*base->config->optimizer);
    if (!optimizer) {
      return Result<void>::Failure(
          OptimizerFactoryError(optimizer.GetError(), *base, context));
    }
    auto candidate = data_load_.ExecuteAgent(
        {base, {}, {}, governor_, context.cancellation, context.progress,
         std::move(optimizer).Value(), algorithms_,
         static_cast<float>(base->config->root.save_voxel_size), false,
         base->config->root.max_parallel_agents,
         [this, candidate_revision = base->revision](
             const AgentId& loaded_agent, const AgentRawDataHandle& raw,
             const VisualizationPointPreviewHandle& preview) {
           if (data_load_preview_) {
             data_load_preview_(candidate_revision, loaded_agent, raw,
                                preview);
           }
         }},
        *agent);
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, nullptr,
        [agent](RuntimeState&, ArtifactRepository& artifacts,
                PendingOutputSet*) {
          artifacts.BeginNode(NodeId::kDataLoad, agent);
          artifacts.CompleteNode(NodeId::kDataLoad, agent);
          return Result<void>::Ok();
        });
  }
  if (node == NodeId::kLoopDetect) {
    auto candidate = alignment_.ReplayLoopDetectThrough(base, *agent, context);
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, nullptr,
        [target = *agent](RuntimeState& working,
                          ArtifactRepository& artifacts, PendingOutputSet*) {
          artifacts.CompleteLoopDetectThrough(target,
                                              working.ordered_agents);
          return Result<void>::Ok();
        });
  }
  if (node == NodeId::kOptimize) {
    return ExecuteOptimizeThrough(std::move(base), *agent, context);
  }
  if (node == NodeId::kMapUpdate) {
    auto pending = outputs_.Begin();
    auto candidate = map_update_.ExecuteAgent(
        {base, governor_, context.cancellation, context.progress, algorithms_,
         base->config->root.output_directory,
         base->config->root.save_voxel_size, false,
         base->config->root.max_parallel_agents, &pending},
        *agent);
    if (!candidate) return Result<void>::Failure(candidate.GetError());
    const AgentId target = *agent;
    return CommitCandidate(
        std::move(base), std::move(candidate).Value(), context, &pending,
        [this, target](RuntimeState& working, ArtifactRepository& artifacts,
                       PendingOutputSet*) {
          artifacts.BeginNode(NodeId::kMapUpdate, target);
          artifacts.CompleteNode(NodeId::kMapUpdate, target);
          return RecordMapOutputs(working, {target}, artifacts);
        });
  }
  if (node == NodeId::kPoseSave) {
    return CommitSave(std::move(base), SaveExecutionMode::kPoseSave, context);
  }
  return CommitSave(std::move(base), SaveExecutionMode::kFallbackMapSave,
                    context);
}

Result<void> StageCoordinator::ExecuteOptimizeThrough(
    std::shared_ptr<const RuntimeState> base, const AgentId& target,
    const ExecutionContext& context) {
  auto valid = ValidateInvocation(base, context);
  if (!valid) return valid;
  auto candidate = optimize_.ReplayThrough(base, target, context);
  if (!candidate) return Result<void>::Failure(candidate.GetError());
  return CommitCandidate(
      std::move(base), std::move(candidate).Value(), context, nullptr,
      [target](RuntimeState& working, ArtifactRepository& artifacts,
               PendingOutputSet*) {
        artifacts.CompleteOptimizeThrough(target, working.ordered_agents);
        return Result<void>::Ok();
      });
}

Result<void> StageCoordinator::ExecuteReconfigure(
    std::shared_ptr<const RuntimeState> base, ConfigDomain domain,
    uint64_t revision, const ExecutionContext& context) {
  auto valid = ValidateInvocation(base, context);
  if (!valid) return valid;
  auto prepared = reconfigurer_.Prepare(base, domain, revision);
  if (!prepared) return Result<void>::Failure(prepared.GetError());
  ExecutionCandidate candidate;
  candidate.base_revision = base->revision;
  candidate.payload = prepared.Value().payload;
  candidate.execution_agents = base->ordered_agents;
  candidate.completion = ArtifactCompletionKind::kAlignmentStage;
  const auto next_config = prepared.Value().config;
  return CommitCandidate(
      std::move(base), std::move(candidate), context, nullptr,
      [domain, next_config](RuntimeState& working,
                            ArtifactRepository& artifacts,
                            PendingOutputSet*) {
        working.config = next_config;
        artifacts.ApplyConfig(domain, next_config->revision);
        return Result<void>::Ok();
      });
}

Result<ConfigApplyReceipt> StageCoordinator::ApplyConfig(
    std::shared_ptr<const RuntimeState> base,
    const ConfigCandidate& candidate, const ExpectedRevision& expected,
    const ExecutionContext& context) {
  auto valid = ValidateInvocation(base, context);
  if (!valid) return Result<ConfigApplyReceipt>::Failure(valid.GetError());
  if (expected.runtime_revision != base->revision ||
      expected.config_revision != base->config->revision) {
    return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
        "config transaction expected revision does not match committed state"));
  }
  if (base->revision == std::numeric_limits<uint64_t>::max() ||
      base->config->revision == std::numeric_limits<uint64_t>::max()) {
    return Result<ConfigApplyReceipt>::Failure(
        Error::InvalidArgument("config transaction revision is exhausted"));
  }
  if (candidate.domain == ConfigDomain::kGlobal ||
      candidate.domain == ConfigDomain::kDataLoader) {
    return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
        "data/global config requires a new runtime replacement"));
  }

  const uint64_t next_config_revision = base->config->revision + 1;
  auto prepared =
      reconfigurer_.Prepare(base, candidate, next_config_revision);
  if (!prepared) {
    return Result<ConfigApplyReceipt>::Failure(prepared.GetError());
  }
  const auto next_config = prepared.Value().config;
  if (!next_config || !next_config->documents) {
    return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
        "config transaction produced no canonical documents"));
  }
  const auto* module =
      ConfigDocumentFor(*next_config->documents, candidate.domain);
  if (!module) {
    return Result<ConfigApplyReceipt>::Failure(
        Error::InvalidArgument("config transaction domain has no document"));
  }

  auto pending = outputs_.Begin();
  auto staged =
      StageConfigFile(module->path, module->canonical_json, pending);
  if (!staged)
    return Result<ConfigApplyReceipt>::Failure(staged.GetError());
  if (next_config->documents->root.canonical_json !=
      base->config->documents->root.canonical_json) {
    staged = StageConfigFile(next_config->documents->root.path,
                             next_config->documents->root.canonical_json,
                             pending);
    if (!staged)
      return Result<ConfigApplyReceipt>::Failure(staged.GetError());
  }

  const CommittedRuntimeSnapshot before{
      base->revision, base->config->revision, base->ordered_agents,
      base->artifacts};
  RuntimeTransaction transaction(base);
  transaction.SetPayload(prepared.Value().payload);
  auto artifacts = ArtifactEditor(*base);
  transaction.Working().config = next_config;
  artifacts->ApplyConfig(candidate.domain, next_config_revision);
  transaction.Working().artifacts = artifacts->Snapshot();
  auto finalized = std::move(transaction).Finalize(context.cancellation);
  if (!finalized)
    return Result<ConfigApplyReceipt>::Failure(finalized.GetError());
  const auto committed_state = finalized.Value();
  const CommittedRuntimeSnapshot after{
      committed_state->revision, committed_state->config->revision,
      committed_state->ordered_agents, committed_state->artifacts};
  auto committed = runtime_state_store_.CommitWithBarrier(
      base, committed_state, [&pending, committed_revision =
                                 committed_state->revision] {
        return CommitPending(&pending, committed_revision);
      });
  if (!committed)
    return Result<ConfigApplyReceipt>::Failure(committed.GetError());
  return Result<ConfigApplyReceipt>::Ok(
      {base->config->revision, next_config_revision, base->revision,
       committed_state->revision, ArtifactRevisionAffectedAgents(before, after)});
}

Result<void> StageCoordinator::CommitSave(
    std::shared_ptr<const RuntimeState> base, SaveExecutionMode mode,
    const ExecutionContext& context) {
  auto artifacts = ArtifactEditor(*base);
  auto pending = outputs_.Begin();
  auto prepared = save_.Prepare(
      {*base, mode, base->config->root.output_directory,
       base->config->root.enable_map_updater, context.cancellation},
      pending, *artifacts);
  if (!prepared) return Result<void>::Failure(prepared.GetError());

  RuntimeTransaction transaction(base);
  transaction.Working().artifacts = artifacts->Snapshot();
  auto finalized = std::move(transaction).Finalize(context.cancellation);
  if (!finalized) return Result<void>::Failure(finalized.GetError());
  auto finalized_state = std::move(finalized).Value();
  const uint64_t committed_revision = finalized_state->revision;
  auto committed = runtime_state_store_.CommitWithBarrier(
      base, std::move(finalized_state), [&pending, committed_revision] {
        return CommitPending(&pending, committed_revision);
      });
  return committed ? Result<void>::Ok()
                   : Result<void>::Failure(committed.GetError());
}

}  // namespace open_lmm
