#include <open_lmm/common/algorithm_execution_context.hpp>
#include <domain/optimization/backend_optimizer_base.hpp>
#include <domain/data_loader/data_loader_base.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <domain/loop_detection/loop_detector_base.hpp>

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AgentContext Agent() {
  auto id = AgentId::Parse("A").Value();
  auto catalog = AgentSymbolCatalog::Build({id}).Value();
  auto handle = std::make_shared<const AgentSymbolCatalog>(std::move(catalog));
  return {id, handle->SymbolFor(id).Value(), handle, AgentRole::kAnchor, 0};
}

AlgorithmExecutionContext Context() {
  AlgorithmExecutionContext context;
  context.agent = Agent();
  context.config = std::make_shared<const AlgorithmConfigSnapshot>(
      AlgorithmConfigSnapshot{"fixture.schema", 3, "{}", "sha256:test"});
  context.cancellation = std::make_shared<CancellationToken>();
  context.feedback = std::make_shared<AlignmentFeedbackBroker>();
  context.resource_budget = {4, 1024, 512};
  context.base_revision = 17;
  context.operation = "fixture.process";
  context.plugin_id = "fixture.plugin";
  return context;
}

void CheckContext(const Error& error) {
  Check(error.context.runtime_revision == 17,
        "error must retain the base revision");
  Check(error.context.stage == "algorithm" &&
            error.context.node == "fixture.process",
        "error must retain operation context");
  Check(error.context.agent == Agent().id,
        "error must retain agent context");
  Check(error.context.plugin == "fixture.plugin",
        "error must retain plugin context");
  Check(error.context.config == "fixture.schema",
        "error must retain immutable config context");
}

class Loader final : public DataLoaderBase {
 public:
  enum class Mode { kSuccess, kFailure, kThrow } mode = Mode::kSuccess;
  int calls = 0;

  Result<AgentRawData> Process(const AlgorithmExecutionContext& context,
                               const DataLoaderInput&) override {
    AlgorithmExecutionTimer timer(context);
    auto cancelled = CheckAlgorithmCancellation(context, "before fixture load");
    if (!cancelled) return Result<AgentRawData>::Failure(cancelled.GetError());
    ++calls;
    if (mode == Mode::kFailure) {
      return Result<AgentRawData>::Failure(
          WithAlgorithmContext(Error::InvalidArgument("loader rejected input"),
                               context));
    }
    if (mode == Mode::kThrow) {
      return Result<AgentRawData>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("loader threw"), context));
    }
    AgentRawData output;
    output.agent_id = context.agent.id;
    return Result<AgentRawData>::Ok(std::move(output));
  }
  Result<std::size_t> VisitRawScanData(
      const AlgorithmExecutionContext&, const fs::path&,
      const RawScanVisitor&, AlgorithmProgressPhase) override {
    return Result<std::size_t>::Ok(0);
  }
};

class Detector final : public LoopDetectorBase {
 public:
  Result<LoopDetectorOutput> Process(
      const AlgorithmExecutionContext& context,
      const LoopDetectorProcessInput&) override {
    return Result<LoopDetectorOutput>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown detector exception"), context));
  }
};

class Optimizer final : public BackendOptimizerBase {
 public:
  bool observed_context_token = false;

  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext& context,
      const BackendOptimizerInput&) override {
    observed_context_token = static_cast<bool>(context.cancellation);
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed("optimizer threw"), context));
  }
  void Reset() override {}
  bool HasProcessedAgent(const AgentId&) const override { return false; }
  std::size_t ProcessedAgentCount() const override { return 0; }
};

class NullRemover final : public DynamicRemoverBase {
 public:
  Result<PointCloud::Ptr> Process(const AlgorithmExecutionContext& context,
                                  DynamicRemoverInput) override {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("dynamic remover returned a null point cloud"),
        context));
  }
};

void TestDataLoaderShim() {
  Loader loader;
  DataLoaderBase& algorithm = loader;
  auto context = Context();
  int log_calls = 0;
  int profile_calls = 0;
  context.logger = [&](AlgorithmLogLevel, std::string_view) { ++log_calls; };
  context.profiler = [&](std::string_view operation, double elapsed_ms) {
    Check(operation == "fixture.process" && elapsed_ms >= 0.0,
          "profiler must receive the operation and elapsed time");
    ++profile_calls;
  };

  auto success = algorithm.Process(context, DataLoaderInput{"data"});
  Check(success && success.Value().agent_id == context.agent.id,
        "loader shim must preserve successful legacy behavior");
  Check(profile_calls == 1, "successful calls must be profiled");

  loader.mode = Loader::Mode::kFailure;
  auto failure = algorithm.Process(context, DataLoaderInput{"data"});
  Check(!failure, "legacy Result failure must cross the new boundary");
  CheckContext(failure.GetError());
  Check(log_calls == 1 && profile_calls == 2,
        "failure must be logged and profiled exactly once");

  loader.mode = Loader::Mode::kThrow;
  auto thrown = algorithm.Process(context, DataLoaderInput{"data"});
  Check(!thrown, "legacy exception must not cross the Result boundary");
  CheckContext(thrown.GetError());

  context.cancellation->Request();
  const int calls_before_cancel = loader.calls;
  auto cancelled = algorithm.Process(context, DataLoaderInput{"data"});
  Check(!cancelled && cancelled.GetError().code == Error::Code::kCancelled &&
            loader.calls == calls_before_cancel,
        "pre-cancelled call must not enter the legacy implementation");
  CheckContext(cancelled.GetError());
}

void TestDetectorExceptionBoundary() {
  Detector detector;
  LoopDetectorBase& algorithm = detector;
  auto context = Context();
  AgentRawData raw;
  raw.agent_id = context.agent.id;
  DescriptorStore descriptors;
  AgentRawDataMap all_raw;
  AgentOptimizedDataMap optimized;
  auto result = algorithm.Process(
      context, LoopDetectorProcessInput{raw, descriptors, all_raw, optimized});
  Check(!result, "unknown detector exception must become a Result failure");
  CheckContext(result.GetError());
}

void TestOptimizerExceptionBoundaryAndTokenRestore() {
  Optimizer optimizer;
  BackendOptimizerBase& algorithm = optimizer;
  auto context = Context();
  AgentRawData raw;
  raw.agent_id = context.agent.id;
  LoopPairVec intra;
  LoopPairVec inter;
  AgentRawDataMap all_raw;
  auto result = algorithm.Process(
      context, BackendOptimizerInput{raw, intra, inter, all_raw});
  Check(!result && result.GetError().code == Error::Code::kOptimizationFailed,
        "optimizer exception must become OptimizationFailed");
  CheckContext(result.GetError());
  Check(optimizer.observed_context_token,
        "optimizer must observe command cancellation directly from context");
}

void TestNullRemoverBoundary() {
  NullRemover remover;
  DynamicRemoverBase& algorithm = remover;
  auto context = Context();
  auto result = algorithm.Process(context, DynamicRemoverInput{});
  Check(!result, "null remover success must become a Result failure");
  CheckContext(result.GetError());
}

}  // namespace

int main() {
  TestDataLoaderShim();
  TestDetectorExceptionBoundary();
  TestOptimizerExceptionBoundaryAndTokenRestore();
  TestNullRemoverBoundary();
  std::cout << "Algorithm contract tests passed\n";
  return EXIT_SUCCESS;
}
