#include "optimize_executor.hpp"

#include <algorithm>

#include <runtime/execution/nodes/optimize_node.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>
#include <runtime/state/runtime_payload_builder.hpp>

namespace open_lmm {
OptimizeExecutor::OptimizeExecutor(
    std::shared_ptr<const AlgorithmProvider> algorithms)
    : algorithms_(std::move(algorithms)) {
  optimizer_factory_ = [algorithms = algorithms_](const OptimizerConfig& config) {
    return algorithms->CreateOptimizer(config);
  };
}

OptimizeExecutor::OptimizeExecutor(OptimizerFactory optimizer_factory,
                                   OptimizeStep optimize_step)
    : optimizer_factory_(std::move(optimizer_factory)),
      optimize_step_(std::move(optimize_step)) {}

Result<ExecutionCandidate> OptimizeExecutor::ReplayThrough(
    std::shared_ptr<const RuntimeState> committed,
    const AgentId& target_agent, const ExecutionContext& runtime) const {
  if (!committed || !committed->config || !committed->config->optimizer ||
      !committed->payload || !committed->payload->database) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "optimizer replay requires a complete committed state"));
  }
  if (runtime.base_revision != committed->revision) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "optimizer base revision does not match committed state"));
  }
  if (committed->payload->database->raw_data.size() !=
      committed->payload->contexts.size()) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "DataLoad and loop detection must complete before optimizer replay"));
  }
  auto prefix = OrderedAgentPrefix(committed->ordered_agents, target_agent);
  if (!prefix) return Result<ExecutionCandidate>::Failure(prefix.GetError());
  auto optimizer = optimize_step_
                       ? optimizer_factory_(*committed->config->optimizer)
                       : committed->payload->optimizer->ForkCandidate();
  if (!optimizer) {
    if (committed->config->documents) {
      auto context = MakeAlgorithmExecutionContext(
          *committed, runtime, {}, committed->config->documents->optimizer,
          "open_lmm.backend_optimizer",
          optimize_step_ ? "optimize_factory" : "optimize_fork",
          committed->config->optimizer->type);
      return Result<ExecutionCandidate>::Failure(
          WithAlgorithmContext(optimizer.GetError(), context));
    }
    Error error = optimizer.GetError();
    error.WithRuntimeRevision(committed->revision)
        .WithExecution("algorithm",
                       optimize_step_ ? "optimize_factory" : "optimize_fork");
    return Result<ExecutionCandidate>::Failure(std::move(error));
  }

  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = committed->payload->database->raw_data;
  database->stored_alignments = committed->payload->database->stored_alignments;
  database->alignment_feedback = runtime.alignment_feedback;
  auto contexts = committed->payload->contexts;
  bool anchor_descriptor = true;
  for (const AgentId& id : prefix.Value()) {
    const auto item = std::find_if(contexts.begin(), contexts.end(),
                                   [&id](const auto& value) {
                                     return value.agent.id == id;
                                   });
    if (item == contexts.end() || !item->loop_output) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "Alignment loop artifacts are missing for agent " + id.Value()));
    }
    if (anchor_descriptor) {
      database->descriptor_store.set_anchor_descriptor(
          id, item->loop_output->agent_descriptors);
      anchor_descriptor = false;
    } else {
      database->descriptor_store.merge_descriptor_db(
          id, item->loop_output->agent_descriptors);
    }
  }

  if (!optimize_step_) {
    if (!committed->config->documents) {
      return Result<ExecutionCandidate>::Failure(
          Error::InvalidArgument("optimizer config document is unavailable"));
    }
    auto optimize_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimize_prefix",
        committed->config->optimizer->type);
    const auto representative = std::find_if(
        contexts.begin(), contexts.end(), [&prefix](const auto& value) {
          return value.agent.id == prefix.Value().front();
        });
    if (representative == contexts.end()) {
      return Result<ExecutionCandidate>::Failure(
          Error::InvalidArgument("optimizer prefix context is unavailable"));
    }
    optimize_context.agent = representative->agent;
    const std::size_t processed_count = optimizer.Value()->ProcessedAgentCount();
    if (processed_count > committed->ordered_agents.size()) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "committed optimizer processed-agent count is invalid"));
    }
    for (std::size_t index = 0; index < processed_count; ++index) {
      if (!optimizer.Value()->HasProcessedAgent(
              committed->ordered_agents[index])) {
        return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
            "committed optimizer agents are not an ordered runtime prefix"));
      }
    }

    if (processed_count < prefix.Value().size()) {
      // LoopDetect Through intentionally leaves its target unoptimized. Append
      // only that pending tail to the fork; never replay already-owned factors.
      OptimizeNode optimize_node(optimizer.Value(), optimize_context);
      for (std::size_t index = processed_count;
           index < prefix.Value().size(); ++index) {
        const AgentId& id = prefix.Value()[index];
        auto item = std::find_if(contexts.begin(), contexts.end(),
                                 [&id](const auto& value) {
                                   return value.agent.id == id;
                                 });
        if (item == contexts.end()) {
          return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
              "optimizer pending prefix context is unavailable"));
        }
        item->cancellation = runtime.cancellation;
        item->flow = ControlFlow::kContinue;
        auto optimized = optimize_node.Process(*item, *database);
        if (!optimized) {
          return Result<ExecutionCandidate>::Failure(optimized.GetError());
        }
      }
    } else {
      auto optimized = optimizer.Value()->OptimizePrefix(
          optimize_context, prefix.Value(), database->raw_data);
      if (!optimized) {
        return Result<ExecutionCandidate>::Failure(optimized.GetError());
      }

      std::map<AgentId, Eigen::Isometry3d> optimized_map_transforms;
      for (auto& [id, output] : optimized.Value()) {
        const auto raw = database->raw_data.find(id);
        if (raw != database->raw_data.end() &&
            !output.optimized_poses.empty()) {
          const auto& [index, global_pose] = output.optimized_poses.front();
          if (index >= 0 &&
              static_cast<std::size_t>(index) < raw->second->odom_poses.size()) {
            optimized_map_transforms[id] =
                global_pose * raw->second->odom_poses[index].inverse();
          }
        }
        database->optimized_data[id] =
            std::make_shared<const AgentOptimizedData>(std::move(output));
      }
      for (const AgentId& id : prefix.Value()) {
        const auto item = std::find_if(contexts.begin(), contexts.end(),
                                       [&id](const auto& value) {
                                         return value.agent.id == id;
                                       });
        if (!item->loop_output->accepted_global_T_agent ||
            !item->loop_output->accepted_alignment_method ||
            !item->loop_output->accepted_alignment_approval) {
          return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
              "alignment output is missing its accepted global transform"));
        }
        auto stored = database->descriptor_store.set_agent_map(
            id, item->loop_output->alignment_map,
            *item->loop_output->accepted_global_T_agent,
            *item->loop_output->accepted_alignment_method,
            *item->loop_output->accepted_alignment_approval,
            item->loop_output->accepted_target_agent,
            item->loop_output->accepted_at_unix_ms);
        if (!stored) {
          return Result<ExecutionCandidate>::Failure(stored.GetError());
        }
      }
      database->descriptor_store.update_transforms(optimized_map_transforms);
    }
  } else {
    for (const AgentId& id : prefix.Value()) {
      auto item = std::find_if(contexts.begin(), contexts.end(),
                               [&id](const auto& value) {
                                 return value.agent.id == id;
                               });
      item->cancellation = runtime.cancellation;
      item->flow = ControlFlow::kContinue;
      auto optimized = optimize_step_(*item, *database, optimizer.Value());
      if (!optimized) {
        return Result<ExecutionCandidate>::Failure(optimized.GetError());
      }
    }
  }
  if (runtime.cancellation &&
      runtime.cancellation->IsCancellationRequested()) {
    return Result<ExecutionCandidate>::Failure(
        Error::Cancelled("before optimizer replay candidate publish"));
  }

  const auto target = std::find(committed->ordered_agents.begin(),
                                committed->ordered_agents.end(), target_agent);
  for (auto suffix = std::next(target);
       suffix != committed->ordered_agents.end(); ++suffix) {
    const auto item = std::find_if(contexts.begin(), contexts.end(),
                                   [id = *suffix](const auto& value) {
                                     return value.agent.id == id;
                                   });
    if (item != contexts.end()) item->loop_output.reset();
  }

  RuntimePayloadBuilder builder(committed->payload);
  auto payload = builder.SetContexts(std::move(contexts))
                     .SetDatabase(std::move(database))
                     .SetOptimizer(optimizer.Value())
                     .Build();
  if (!payload) return Result<ExecutionCandidate>::Failure(payload.GetError());
  ExecutionCandidate candidate;
  candidate.base_revision = committed->revision;
  candidate.payload = std::move(payload).Value();
  candidate.execution_agents = prefix.Value();
  candidate.completion = ArtifactCompletionKind::kOptimizeThrough;
  candidate.replay_target = target_agent;
  return Result<ExecutionCandidate>::Ok(std::move(candidate));
}

}  // namespace open_lmm
