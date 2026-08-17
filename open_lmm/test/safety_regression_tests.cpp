#include <cstdio>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/rigid_transform.hpp>
#include <open_lmm/core/data_loader/data_loader_file.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_incremental.hpp>
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

void TestRigidTransformInverse() {
  std::mt19937 generator(0x4c4d4dU);
  std::uniform_real_distribution<double> component(-1.0, 1.0);
  std::uniform_real_distribution<double> angle(-3.141592653589793,
                                                3.141592653589793);
  std::uniform_real_distribution<double> translation(-100.0, 100.0);

  for (int sample = 0; sample < 128; ++sample) {
    Eigen::Vector3d axis(component(generator), component(generator),
                         component(generator));
    if (axis.squaredNorm() < 1e-12) axis = Eigen::Vector3d::UnitX();
    axis.normalize();

    Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
    transform.linear() =
        Eigen::AngleAxisd(angle(generator), axis).toRotationMatrix();
    transform.translation() =
        Eigen::Vector3d(translation(generator), translation(generator),
                        translation(generator));

    const Eigen::Isometry3d actual =
        open_lmm::InvertRigidTransform(transform);
    const Eigen::Isometry3d expected = transform.inverse();
    Expect(actual.matrix().isApprox(expected.matrix(), 1e-12),
           "rigid inverse must match Eigen's isometry inverse");
    Expect((actual * transform).matrix().isApprox(
               Eigen::Matrix4d::Identity(), 1e-12),
           "rigid inverse must compose to identity");
  }
}

void TestGlobalAlignmentLoopConvention() {
  Eigen::Isometry3d global_T_target = Eigen::Isometry3d::Identity();
  global_T_target.translation() = Eigen::Vector3d(10.0, 2.0, 0.0);
  Eigen::Isometry3d global_T_source = Eigen::Isometry3d::Identity();
  global_T_source.translation() = Eigen::Vector3d(13.0, 5.0, 1.0);
  const auto target_T_source = open_lmm::TargetFromSourceScanTransform(
      global_T_target, global_T_source);
  const Eigen::Vector3d source_origin_in_target = target_T_source.translation();
  Expect(source_origin_in_target.isApprox(Eigen::Vector3d(3.0, 3.0, 1.0)),
         "global alignment loop must map source scan into target scan frame");
  Expect((global_T_target * target_T_source).matrix().isApprox(
             global_T_source.matrix(), 1e-12),
         "target_T_source convention must reconstruct global source pose");
}

void TestAlignedMapRebuildUsesLatestTransform() {
  open_lmm::DescriptorStore store;
  std::vector<Eigen::Vector3f> anchor{{1.0F, 0.0F, 0.0F}};
  std::vector<Eigen::Vector3f> follower{{0.0F, 2.0F, 0.0F}};
  store.set_agent_map('A', anchor, Eigen::Isometry3d::Identity(),
                      open_lmm::AlignmentMethod::kKissMatcher,
                      open_lmm::AlignmentApproval::kAutomatic, 'A', 1);
  Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
  initial.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  store.set_agent_map('B', follower, initial,
                      open_lmm::AlignmentMethod::kManual,
                      open_lmm::AlignmentApproval::kUser, 'A', 2);
  Expect(store.merged_map.size() == 2,
         "aligned map store must rebuild all accepted agent maps");
  Expect(store.merged_map[1].isApprox(Eigen::Vector3f(10.0F, 2.0F, 0.0F)),
         "accepted transform must place follower map in target frame");

  Eigen::Isometry3d optimized = Eigen::Isometry3d::Identity();
  optimized.translation() = Eigen::Vector3d(20.0, 0.0, 0.0);
  store.update_transform('B', optimized);
  Expect(store.merged_map.size() == 2,
         "optimized rebuild must replace rather than append follower points");
  Expect(store.merged_map[1].isApprox(Eigen::Vector3f(20.0F, 2.0F, 0.0F)),
         "latest backend transform must replace the provisional transform");
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

void TestOptimizerLifecycle() {
  const std::string config_path = "/tmp/open_lmm_optimizer_lifecycle.json";
  {
    std::ofstream output(config_path);
    output << R"({"backend_optimizer":{"relinearizeThreshold":0.1,"relinearizeSkip":1,"isam_extra_updates":0,"min_loop_frame_gap":30,"icp_search_num":0}})";
  }

  open_lmm::BackendOptimizerIncremental optimizer{open_lmm::Config(config_path)};
  open_lmm::AgentContext anchor{
      .id = "A"[0], .role = open_lmm::AgentRole::kAnchor, .order = 0};
  open_lmm::AgentRawData raw_a;
  raw_a.agent_id = "A"[0];
  raw_a.odom_poses.push_back(Eigen::Isometry3d::Identity());
  (void)optimizer.Process(anchor, raw_a, {}, {}, {});
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             optimizer.HasProcessedAgent("A"[0]),
         "successful optimizer call must commit its agent");

  try {
    (void)optimizer.Process(anchor, raw_a, {}, {}, {});
    Expect(false, "duplicate optimizer agent must fail");
  } catch (const std::invalid_argument&) {
  }
  Expect(optimizer.ProcessedAgentCount() == 1,
         "duplicate failure must not mutate optimizer lifecycle");

  open_lmm::AgentContext follower{
      .id = "B"[0], .role = open_lmm::AgentRole::kFollower, .order = 1};
  open_lmm::AgentRawData raw_b;
  raw_b.agent_id = "B"[0];
  raw_b.odom_poses.push_back(Eigen::Isometry3d::Identity());
  open_lmm::LoopPair inter_loop{
      .to = {"A"[0], 0},
      .from = {"B"[0], 0},
      .init_rel_pose = Eigen::Isometry3d::Identity()};
  try {
    (void)optimizer.Process(follower, raw_b, {}, {inter_loop}, {});
    Expect(false, "optimizer task exception must fail");
  } catch (const std::out_of_range&) {
  }
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             !optimizer.HasProcessedAgent("B"[0]),
         "failed optimizer transaction must preserve committed state");

  try {
    (void)optimizer.Process(follower, raw_b, {}, {}, {});
    Expect(false, "follower without a refined inter-agent loop must fail");
  } catch (const std::runtime_error&) {
  }
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             !optimizer.HasProcessedAgent("B"[0]),
         "zero-factor rejection must preserve optimizer transaction");
  optimizer.Reset();
  Expect(optimizer.ProcessedAgentCount() == 0 &&
             !optimizer.HasProcessedAgent("A"[0]),
         "Reset must clear graph lifecycle metadata");

  auto cancellation = std::make_shared<open_lmm::CancellationToken>();
  cancellation->Request();
  optimizer.SetCancellationToken(cancellation);
  try {
    (void)optimizer.Process(anchor, raw_a, {}, {}, {});
    Expect(false, "cancelled optimizer must not return a result");
  } catch (const open_lmm::CancellationException&) {
  }
  Expect(optimizer.ProcessedAgentCount() == 0,
         "cancelled optimizer must not commit lifecycle state");
  std::remove(config_path.c_str());
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

void TestConfigContractValidation() {
  const std::string type_path = "/tmp/open_lmm_wrong_type_config.json";
  {
    std::ofstream output(type_path);
    output << R"({"module":{"count":"not-an-integer"}})";
  }
  try {
    open_lmm::Config config(type_path);
    (void)config.param<int>("module", "count", 1);
    Expect(false, "wrong config type must throw");
  } catch (const std::exception& e) {
    const std::string message = e.what();
    Expect(message.find(type_path) != std::string::npos &&
               message.find("module/count") != std::string::npos,
           "config type error must include file and parameter context");
  }
  std::remove(type_path.c_str());

  const std::string range_path = "/tmp/open_lmm_invalid_loader_config.json";
  {
    std::ofstream output(range_path);
    output << R"({"data_loader":{"pose_format":"kitti","scan_type":"pcd","scan_dir_name":"Scans","pose_file_name":"poses.txt","voxel_size":0.0,"min_range":5.0,"max_range":1.0,"delimiter":" "}})";
  }
  open_lmm::DataLoaderFile loader{open_lmm::Config(range_path)};
  open_lmm::AgentContext agent{
      .id = "A"[0], .role = open_lmm::AgentRole::kAnchor, .order = 0};
  auto result = loader.Process(agent, "/tmp");
  Expect(!result.IsOk(), "invalid DataLoader numeric range must fail");
  std::remove(range_path.c_str());

  try {
    open_lmm::Config empty_config("");
    empty_config.save("/tmp/open_lmm_missing_dir/config.json");
    Expect(false, "config save to missing directory must throw");
  } catch (const std::exception&) {
  }
}

void TestPointCloudInputValidation() {
  try {
    (void)open_lmm::readPointsFromPCD(
        "/tmp/open_lmm_scan_that_does_not_exist.pcd");
    Expect(false, "missing PCD must throw");
  } catch (const std::exception&) {
  }

  const std::string truncated_path = "/tmp/open_lmm_truncated_scan.bin";
  {
    std::ofstream output(truncated_path, std::ios::binary);
    const float incomplete[] = {1.0f, 2.0f, 3.0f};
    output.write(reinterpret_cast<const char*>(incomplete), sizeof(incomplete));
  }
  try {
    (void)open_lmm::readPointsFromBin(truncated_path);
    Expect(false, "truncated BIN record must throw");
  } catch (const std::exception&) {
  }
  std::remove(truncated_path.c_str());
}

}  // namespace

int main() {
  TestAgentContext();
  TestRigidTransformInverse();
  TestGlobalAlignmentLoopConvention();
  TestAlignedMapRebuildUsesLatestTransform();
  TestInputValidation();
  TestPipelineControlFlow();
  TestPluginFailurePropagation();
  TestOptimizerLifecycle();
  TestConfigFailurePropagation();
  TestConfigContractValidation();
  TestPointCloudInputValidation();
  if (failures != 0) {
    std::cerr << failures << " safety regression test(s) failed\n";
    return 1;
  }
  std::cout << "All safety regression tests passed\n";
  return 0;
}
