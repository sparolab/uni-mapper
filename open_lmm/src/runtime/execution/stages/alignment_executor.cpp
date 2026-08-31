#include "alignment_executor.hpp"

#include <algorithm>

#include <runtime/execution/nodes/loop_detect_node.hpp>
#include <runtime/execution/nodes/optimize_node.hpp>
#include <plugins/host/algorithm_provider.hpp>
#include <runtime/execution/stages/algorithm_context.hpp>
#include <runtime/state/runtime_payload_builder.hpp>

namespace open_lmm {
namespace {

Result<void> CancelledBeforePublish(const ExecutionContext& context) {
  if (context.cancellation &&
      context.cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(
        Error::Cancelled("before alignment candidate publish"));
  }
  return Result<void>::Ok();
}

Error OptimizerFactoryFailure(Error error, const RuntimeState& committed,
                              const ExecutionContext& runtime) {
  if (committed.config && committed.config->documents &&
      committed.config->optimizer) {
    auto context = MakeAlgorithmExecutionContext(
        committed, runtime, {}, committed.config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimize_factory",
        committed.config->optimizer->type);
    return WithAlgorithmContext(std::move(error), context);
  }
  return std::move(error)
      .WithRuntimeRevision(committed.revision)
      .WithExecution("algorithm", "optimize_factory");
}

}  // namespace

AlignmentExecutor::AlignmentExecutor(
    std::shared_ptr<const AlgorithmProvider> algorithms,
    AlignmentPreviewCallback alignment_preview)
    : alignment_preview_(std::move(alignment_preview)),
      algorithms_(std::move(algorithms)) {
  optimizer_factory_ = [algorithms = algorithms_](const OptimizerConfig& config) {
    return algorithms->CreateOptimizer(config);
  };
}

AlignmentExecutor::AlignmentExecutor(OptimizerFactory optimizer_factory,
                                     LoopStep loop_step,
                                     OptimizeStep optimize_step,
                                     AlignmentPreviewCallback alignment_preview)
    : optimizer_factory_(std::move(optimizer_factory)),
      loop_step_(std::move(loop_step)),
      optimize_step_(std::move(optimize_step)),
      alignment_preview_(std::move(alignment_preview)) {}

Result<void> AlignmentExecutor::ValidateBase(
    const RuntimeState& committed, const ExecutionContext& context) {
  if (context.base_revision != committed.revision) {
    return Result<void>::Failure(Error::InvalidArgument(
        "alignment base revision does not match committed state"));
  }
  if (!committed.config || !committed.config->loop_detector ||
      !committed.config->optimizer || !committed.payload ||
      !committed.payload->database) {
    return Result<void>::Failure(
        Error::InvalidArgument("alignment requires a complete runtime state"));
  }
  if (committed.payload->database->raw_data.size() !=
      committed.payload->contexts.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad stage must complete before Alignment"));
  }
  return CancelledBeforePublish(context);
}

Result<std::shared_ptr<BackendOptimizerBase>>
AlignmentExecutor::CreateOptimizer(const RuntimeState& committed) const {
  return optimizer_factory_(*committed.config->optimizer);
}

Result<std::shared_ptr<const RuntimePayload>> AlignmentExecutor::BuildPayload(
    const RuntimeState& committed, std::vector<AgentPipelineCtx> contexts,
    std::shared_ptr<SharedDatabase> database,
    std::shared_ptr<BackendOptimizerBase> optimizer) {
  RuntimePayloadBuilder builder(committed.payload);
  return builder.SetContexts(std::move(contexts))
      .SetDatabase(std::move(database))
      .SetOptimizer(std::move(optimizer))
      .Build();
}

Result<ExecutionCandidate> AlignmentExecutor::ExecuteStage(
    std::shared_ptr<const RuntimeState> committed,
    const ExecutionContext& runtime) const {
  if (!committed) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("alignment has no committed state"));
  }
  auto valid = ValidateBase(*committed, runtime);
  if (!valid) return Result<ExecutionCandidate>::Failure(valid.GetError());
  auto optimizer = CreateOptimizer(*committed);
  if (!optimizer) {
    return Result<ExecutionCandidate>::Failure(
        OptimizerFactoryFailure(optimizer.GetError(), *committed, runtime));
  }

  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = committed->payload->database->raw_data;
  database->stored_alignments = committed->payload->database->stored_alignments;
  database->alignment_feedback = runtime.alignment_feedback;
  auto contexts = committed->payload->contexts;
  for (auto& item : contexts) {
    item.cancellation = runtime.cancellation;
    item.flow = ControlFlow::kContinue;
    item.loop_output.reset();
  }

  if (loop_step_ || optimize_step_) {
    if (!loop_step_ || !optimize_step_) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "alignment fixture must provide both loop and optimize steps"));
    }
    for (auto& item : contexts) {
      auto loop = loop_step_(item, *database);
      if (!loop) {
        if (loop.GetError().code == Error::Code::kAgentExcluded &&
            !item.agent.is_anchor()) {
          item.flow = ControlFlow::kSkip;
          item.loop_output.reset();
          continue;
        }
        return Result<ExecutionCandidate>::Failure(loop.GetError());
      }
      auto optimized = optimize_step_(item, *database, optimizer.Value());
      if (!optimized) {
        return Result<ExecutionCandidate>::Failure(optimized.GetError());
      }
      if (alignment_preview_) {
        alignment_preview_(committed->revision, contexts, *database);
      }
    }
  } else {
    if (!committed->config->documents) {
      return Result<ExecutionCandidate>::Failure(
          Error::InvalidArgument("alignment config documents are unavailable"));
    }
    auto loop_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->loop_detector,
        "open_lmm.loop_detector", "loop_detect",
        committed->config->loop_detector->model);
    auto optimize_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimize",
        committed->config->optimizer->type);
    Pipeline pipeline;
    for (NodeId node : StageNodes(StageId::kAlignment)) {
      if (node == NodeId::kLoopDetect) {
        pipeline.AddNode(std::make_unique<LoopDetectNode>(
            [algorithms = algorithms_,
             cfg = committed->config->loop_detector]() {
              return algorithms->CreateLoopDetector(*cfg);
            }, loop_context,
            static_cast<std::size_t>(
                committed->config->optimizer->min_loop_frame_gap)));
      } else if (node == NodeId::kOptimize) {
        pipeline.AddNode(std::make_unique<OptimizeNode>(
            optimizer.Value(), optimize_context,
            [this, base_revision = committed->revision, &contexts](
                const AgentPipelineCtx&, const SharedDatabase& database) {
              if (alignment_preview_) {
                alignment_preview_(base_revision, contexts, database);
              }
            }));
      } else {
        return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
            "unsupported Alignment execution spec"));
      }
    }
    auto ran = pipeline.Run(contexts, *database);
    if (!ran) return Result<ExecutionCandidate>::Failure(ran.GetError());
  }
  auto not_cancelled = CancelledBeforePublish(runtime);
  if (!not_cancelled) {
    return Result<ExecutionCandidate>::Failure(not_cancelled.GetError());
  }
  ExecutionCandidate candidate;
  candidate.base_revision = committed->revision;
  for (const auto& item : contexts) {
    if (item.flow == ControlFlow::kSkip) {
      if (item.agent.is_anchor()) {
        return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
            "the anchor agent cannot be excluded from Alignment"));
      }
      candidate.excluded_agents.push_back(item.agent.id);
    } else if (item.loop_output) {
      candidate.execution_agents.push_back(item.agent.id);
    }
  }
  if (candidate.execution_agents.empty() ||
      candidate.execution_agents.front() != committed->ordered_agents.front()) {
    return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
        "Alignment produced no successful anchor result"));
  }
  auto payload = BuildPayload(*committed, std::move(contexts), database,
                              optimizer.Value());
  if (!payload) return Result<ExecutionCandidate>::Failure(payload.GetError());
  candidate.payload = std::move(payload).Value();
  candidate.completion = ArtifactCompletionKind::kAlignmentStage;
  return Result<ExecutionCandidate>::Ok(std::move(candidate));
}

Result<ExecutionCandidate> AlignmentExecutor::ReplayLoopDetectThrough(
    std::shared_ptr<const RuntimeState> committed,
    const AgentId& target_agent, const ExecutionContext& runtime) const {
  if (!committed) {
    return Result<ExecutionCandidate>::Failure(
        Error::InvalidArgument("loop replay has no committed state"));
  }
  auto valid = ValidateBase(*committed, runtime);
  if (!valid) return Result<ExecutionCandidate>::Failure(valid.GetError());
  auto prefix = OrderedAgentPrefix(committed->ordered_agents, target_agent);
  if (!prefix) return Result<ExecutionCandidate>::Failure(prefix.GetError());
  auto optimizer = CreateOptimizer(*committed);
  if (!optimizer) {
    return Result<ExecutionCandidate>::Failure(
        OptimizerFactoryFailure(optimizer.GetError(), *committed, runtime));
  }

  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = committed->payload->database->raw_data;
  database->stored_alignments = committed->payload->database->stored_alignments;
  database->alignment_feedback = runtime.alignment_feedback;
  auto contexts = committed->payload->contexts;
  for (auto& item : contexts) {
    item.cancellation = runtime.cancellation;
    item.flow = ControlFlow::kContinue;
    item.loop_output.reset();
  }
  std::unique_ptr<LoopDetectNode> loop_node;
  std::unique_ptr<OptimizeNode> optimize_node;
  if (!loop_step_) {
    if (!committed->config->documents) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "alignment config documents are unavailable"));
    }
    auto loop_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->loop_detector,
        "open_lmm.loop_detector", "loop_detect",
        committed->config->loop_detector->model);
    auto optimize_context = MakeAlgorithmExecutionContext(
        *committed, runtime, {}, committed->config->documents->optimizer,
        "open_lmm.backend_optimizer", "optimize",
        committed->config->optimizer->type);
    loop_node = std::make_unique<LoopDetectNode>(
        [algorithms = algorithms_, cfg = committed->config->loop_detector]() {
          return algorithms->CreateLoopDetector(*cfg);
        },
        std::move(loop_context),
        static_cast<std::size_t>(
            committed->config->optimizer->min_loop_frame_gap));
    optimize_node = std::make_unique<OptimizeNode>(
        optimizer.Value(), std::move(optimize_context));
  }
  for (std::size_t index = 0; index < prefix.Value().size(); ++index) {
    const AgentId& id = prefix.Value()[index];
    auto item = std::find_if(contexts.begin(), contexts.end(),
                             [&id](const auto& value) {
                               return value.agent.id == id;
                             });
    if (item == contexts.end() || !item->raw_data) {
      return Result<ExecutionCandidate>::Failure(Error::InvalidArgument(
          "ordered replay raw payload is unavailable"));
    }
    Result<void> loop = Result<void>::Ok();
    if (loop_step_) {
      loop = loop_step_(*item, *database);
    } else {
      auto processed = loop_node->Process(*item, *database);
      if (!processed) loop = Result<void>::Failure(processed.GetError());
    }
    if (!loop) return Result<ExecutionCandidate>::Failure(loop.GetError());
    if (index + 1 == prefix.Value().size()) break;
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
  auto not_cancelled = CancelledBeforePublish(runtime);
  if (!not_cancelled) {
    return Result<ExecutionCandidate>::Failure(not_cancelled.GetError());
  }
  ExecutionCandidate candidate;
  candidate.base_revision = committed->revision;
  auto payload = BuildPayload(*committed, std::move(contexts), database,
                              optimizer.Value());
  if (!payload) return Result<ExecutionCandidate>::Failure(payload.GetError());
  candidate.payload = std::move(payload).Value();
  candidate.execution_agents = prefix.Value();
  candidate.completion = ArtifactCompletionKind::kLoopDetectThrough;
  candidate.replay_target = target_agent;
  return Result<ExecutionCandidate>::Ok(std::move(candidate));
}

}  // namespace open_lmm
