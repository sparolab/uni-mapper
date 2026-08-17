#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/validation.hpp>
#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/config.hpp>

namespace {

using open_lmm::AgentPipelineCtx;
using open_lmm::ControlFlow;
using open_lmm::Error;
using open_lmm::PipelineNodeBase;
using open_lmm::Result;
using open_lmm::SharedDatabase;

int failures = 0;

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

class StubNode final : public PipelineNodeBase {
 public:
  StubNode(ControlFlow flow, int& calls) : flow_(flow), calls_(calls) {}

  Result<ControlFlow> Process(AgentPipelineCtx&, SharedDatabase&) override {
    ++calls_;
    return Result<ControlFlow>::Ok(flow_);
  }
  const char* Name() const override { return "Stub"; }

 private:
  ControlFlow flow_;
  int& calls_;
};

class FailingNode final : public PipelineNodeBase {
 public:
  explicit FailingNode(int& calls) : calls_(calls) {}
  Result<ControlFlow> Process(AgentPipelineCtx&, SharedDatabase&) override {
    ++calls_;
    return Result<ControlFlow>::Failure(Error::InvalidArgument("test failure"));
  }
  const char* Name() const override { return "Failing"; }

 private:
  int& calls_;
};

std::vector<AgentPipelineCtx> OneContext() {
  AgentPipelineCtx ctx;
  ctx.agent = {.id = 'A', .role = open_lmm::AgentRole::kAnchor, .order = 0};
  return {ctx};
}

void TestAgentContext() {
  open_lmm::AgentContext anchor{
      .id = 'A', .role = open_lmm::AgentRole::kAnchor, .order = 0};
  open_lmm::AgentContext follower{
      .id = 'B', .role = open_lmm::AgentRole::kFollower, .order = 1};
  Expect(anchor.is_anchor(), "anchor role must be recognized");
  Expect(!follower.is_anchor(), "follower must not be recognized as anchor");
}

void TestInputValidation() {
  Expect(open_lmm::ValidateScanPoseCount(2, 2, "test").IsOk(),
         "equal scan/pose counts must pass");
  Expect(!open_lmm::ValidateScanPoseCount(2, 1, "test").IsOk(),
         "DataLoader scan/pose mismatch must fail");
  Expect(!open_lmm::ValidateScanPoseCount(0, 0, "test").IsOk(),
         "empty DataLoader input must fail");
  Expect(open_lmm::ValidateAgentIndex(1, 2, "anchor").IsOk(),
         "valid agent index must pass");
  Expect(!open_lmm::ValidateAgentIndex(2, 2, "anchor").IsOk(),
         "out-of-range agent index must fail");

  std::vector<int> no_indices;
  Expect(!open_lmm::ValidateNearestNeighborResult(
              0, no_indices, 1, "kdtree").IsOk(),
         "empty KD-tree result must fail");
  Expect(!open_lmm::ValidateNearestNeighborResult(
              1, std::vector<int>{3}, 2, "kdtree").IsOk(),
         "out-of-range KD-tree result must fail");
  auto valid = open_lmm::ValidateNearestNeighborResult(
      1, std::vector<int>{1}, 2, "kdtree");
  Expect(valid.IsOk() && valid.Value() == 1,
         "valid KD-tree result must preserve index");
}

void TestPipelineControlFlow() {
  SharedDatabase db;
  {
    int first = 0;
    int after = 0;
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<StubNode>(ControlFlow::kSkip, first))
        .AddNode(std::make_unique<StubNode>(ControlFlow::kContinue, after));
    auto result = pipeline.Run(contexts, db);
    Expect(result.IsOk(), "skip must be a successful pipeline result");
    Expect(first == 1 && after == 0, "skip must suppress later nodes");
  }
  {
    int first = 0;
    int after = 0;
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<StubNode>(ControlFlow::kKill, first))
        .AddNode(std::make_unique<StubNode>(ControlFlow::kContinue, after));
    auto result = pipeline.Run(contexts, db);
    Expect(!result.IsOk(), "kill must propagate as pipeline failure");
    Expect(first == 1 && after == 0, "kill must suppress later nodes");
    Expect(contexts.front().flow == ControlFlow::kKill,
           "kill must be recorded in agent context");
  }
  {
    int first = 0;
    int after = 0;
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<FailingNode>(first))
        .AddNode(std::make_unique<StubNode>(ControlFlow::kContinue, after));
    auto result = pipeline.Run(contexts, db);
    Expect(!result.IsOk(), "node failure must propagate");
    Expect(first == 1 && after == 0, "failure must suppress later nodes");
  }
  {
    int first = 0;
    int second = 0;
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<StubNode>(ControlFlow::kContinue, first))
        .AddNode(std::make_unique<StubNode>(ControlFlow::kContinue, second));
    auto result = pipeline.Run(contexts, db);
    Expect(result.IsOk() && first == 1 && second == 1,
           "minimal pipeline integration must run all continuing nodes");
  }
}

void TestPluginFailurePropagation() {
  auto result = open_lmm::load_module_from_so<int>(
      "libopen_lmm_test_plugin_that_does_not_exist.so", "create");
  Expect(!result.IsOk(), "missing plugin must return an error");
  if (!result.IsOk()) {
    Expect(result.GetError().code == Error::Code::kPluginLoadFailed,
           "missing plugin must use plugin-load error code");
  }
}

void TestConfigFailurePropagation() {
  open_lmm::Config missing(
      "/tmp/open_lmm_config_file_that_does_not_exist.json");
  Expect(!missing.is_valid(), "missing config must retain a parseable error");

  const std::string malformed_path =
      "/tmp/open_lmm_malformed_safety_regression_config.json";
  {
    std::ofstream output(malformed_path);
    output << "{ malformed json";
  }
  open_lmm::Config malformed(malformed_path);
  Expect(!malformed.is_valid(), "malformed config must retain a parse error");
  std::remove(malformed_path.c_str());
}

}  // namespace

int main() {
  TestAgentContext();
  TestInputValidation();
  TestPipelineControlFlow();
  TestPluginFailurePropagation();
  TestConfigFailurePropagation();
  if (failures != 0) {
    std::cerr << failures << " safety regression test(s) failed\n";
    return 1;
  }
  std::cout << "All safety regression tests passed\n";
  return 0;
}
