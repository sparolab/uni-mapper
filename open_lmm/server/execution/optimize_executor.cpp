#include "optimize_executor.hpp"

#include <algorithm>

#include <open_lmm/server/nodes/optimize_node.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/execution/algorithm_context.hpp>
#include <open_lmm/server/runtime_payload_builder.hpp>

namespace open_lmm {
OptimizeExecutor::OptimizeExecutor()
    : OptimizeExecutor(std::make_shared<AlgorithmFactory>()) {}

OptimizeExecutor::OptimizeExecutor(
    std::shared_ptr<const AlgorithmFactory> algorithms)
    : algorithms_(algorithms ? std::move(algorithms)
                             : std::make_shared<AlgorithmFactory>()) {
  optimizer_factory_ = [algorithms = algorithms_](const OptimizerConfig& config) {
    return algorithms->CreateOptimizer(config);
  };
}

OptimizeExecutor::OptimizeExecutor(OptimizerFactory optimizer_factory,
                                   OptimizeStep optimize_step)
    : optimizer_factory_(std::move(optimizer_factory)),
      optimize_step_(std::move(optimize_step)),
      algorithms_(std::make_shared<AlgorithmFactory>()) {}

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
  auto optimizer = optimizer_factory_(*committed->config->optimizer);
  if (!optimizer) {
    if (committed->config->documents) {
      auto context = MakeAlgorithmExecutionContext(
          *committed, runtime, {}, committed->config->documents->optimizer,
          "open_lmm.backend_optimizer", "optimize_factory",
          committed->config->optimizer->type);
      return Result<ExecutionCandidate>::Failure(
          WithAlgorithmContext(optimizer.GetError(), context));
    }
    Error error = optimizer.GetError();
    error.WithRuntimeRevision(committed->revision)
        .WithExecution("algorithm", "optimize_factory");
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

  std::unique_ptr<OptimizeNode> optimize_node;
  if (!optimize_step_) {
    if (!committed->config->documents) {
      return Result<ExecutionCandidate>::Failure(
          Error::InvalidArgument("optimizer config document is unavailable"));
    }
    auto optimize_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimize",
        committed->config->optimizer->type);
    optimize_node = std::make_unique<OptimizeNode>(
        optimizer.Value(), std::move(optimize_context));
  }
  for (const AgentId& id : prefix.Value()) {
    auto item = std::find_if(contexts.begin(), contexts.end(),
                             [&id](const auto& value) {
                               return value.agent.id == id;
                             });
    item->cancellation = runtime.cancellation;
    item->flow = ControlFlow::kContinue;
    Result<void> optimized = Result<void>::Ok();
    if (optimize_step_) {
      optimized = optimize_step_(*item, *database, optimizer.Value());
    } else {
      auto processed = optimize_node->Process(*item, *database);
      if (!processed) optimized = Result<void>::Failure(processed.GetError());
    }
    if (!optimized) {
      return Result<ExecutionCandidate>::Failure(optimized.GetError());
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
