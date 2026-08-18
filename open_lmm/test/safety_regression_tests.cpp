#include <cstdio>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <memory>
#include <random>
#include <limits>
#include <string>
#include <vector>

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/rigid_transform.hpp>
#include <open_lmm/core/data_loader/data_loader_file.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_incremental.hpp>
#include <open_lmm/core/algorithm_invariants.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_online.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_offline.hpp>
#include <open_lmm/common/validation.hpp>
#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/server/file_set_transaction.hpp>

namespace {

using open_lmm::AgentPipelineCtx;
using open_lmm::ControlFlow;
using open_lmm::Error;
using open_lmm::PipelineNodeBase;
using open_lmm::Result;
using open_lmm::SharedDatabase;

int failures = 0;

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

open_lmm::AgentSymbolCatalogHandle Catalog() {
  auto catalog = open_lmm::AgentSymbolCatalog::Build({Id("A"), Id("B")});
  return std::make_shared<const open_lmm::AgentSymbolCatalog>(
      std::move(catalog).Value());
}

void Expect(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    ++failures;
  }
}

open_lmm::ScanVec::value_type OnePointScan(float x = 0.0F) {
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->emplace_back(x, 0.0F, 0.0F, 1.0F);
  return scan;
}

class RecordingOnlineRemover final : public IOnlineRemoverPlugin {
 public:
  pcl::PointCloud<pcl::PointXYZI>::Ptr run(
      pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
      Eigen::Isometry3d& optimized_pose) override {
    (void)scan;
    translations.push_back(optimized_pose.translation().x());
    return static_map;
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override {
    return static_map;
  }

  std::vector<double> translations;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map = [] {
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->emplace_back(0.0F, 0.0F, 0.0F, 1.0F);
    cloud->emplace_back(0.01F, 0.0F, 0.0F, 1.0F);
    return cloud;
  }();
};

class RecordingOfflineRemover final : public IOfflineRemoverPlugin {
 public:
  void run(pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
           Eigen::Isometry3d& optimized_pose) override {
    scan_x.push_back(scan->front().x);
    translations.push_back(optimized_pose.translation().x());
  }
  bool needsRawMap() const override { return false; }
  void setRawMap(pcl::PointCloud<pcl::PointXYZI>::Ptr&) override {}
  pcl::PointCloud<pcl::PointXYZI>::Ptr getStaticMap() override {
    return OnePointScan();
  }

  std::vector<float> scan_x;
  std::vector<double> translations;
};

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

class UnknownThrowingNode final : public PipelineNodeBase {
 public:
  Result<ControlFlow> Process(AgentPipelineCtx&, SharedDatabase&) override {
    throw 42;
  }
  const char* Name() const override { return "UnknownThrowing"; }
};

class EmptyNestedConfigNode final : public PipelineNodeBase {
 public:
  Result<ControlFlow> Process(AgentPipelineCtx&, SharedDatabase&) override {
    auto config = open_lmm::Config::FromJson(
        R"({"outer":{"value":1}})", "nested-pipeline-fixture");
    (void)config.param_nested<int>({}, "value");
    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }
  const char* Name() const override { return "EmptyNestedConfig"; }
};

std::vector<AgentPipelineCtx> OneContext() {
  AgentPipelineCtx ctx;
  ctx.agent = {.id = Id("A"), .role = open_lmm::AgentRole::kAnchor, .order = 0};
  return {ctx};
}

void TestAgentContext() {
  open_lmm::AgentContext anchor{
      .id = Id("A"), .role = open_lmm::AgentRole::kAnchor, .order = 0};
  open_lmm::AgentContext follower{
      .id = Id("B"), .role = open_lmm::AgentRole::kFollower, .order = 1};
  Expect(anchor.is_anchor(), "anchor role must be recognized");
  Expect(!follower.is_anchor(), "follower must not be recognized as anchor");
}

void TestFileSetCommitRollsBackPartialReplacement() {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("open_lmm_file_commit_" + std::to_string(std::random_device{}()));
  fs::create_directories(root);
  const fs::path original = root / "map_a.pcd";
  const fs::path replacement = root / "map_a.pcd.tmp";
  const fs::path second_temp = root / "map_b.pcd.tmp";
  {
    std::ofstream(original) << "original";
    std::ofstream(replacement) << "replacement";
    std::ofstream(second_temp) << "second";
  }
  const auto result = open_lmm::CommitFileSet({
      {replacement, original},
      {second_temp, root / "missing" / "map_b.pcd"},
  });
  Expect(!result, "file-set commit must report a partial rename failure");
  std::ifstream restored(original);
  std::string contents;
  restored >> contents;
  Expect(contents == "original",
         "file-set commit must restore a replaced destination");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestFileSetCleanupFailureRequiresRecovery() {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("open_lmm_file_recovery_" + std::to_string(std::random_device{}()));
  fs::create_directories(root);
  const fs::path destination = root / "map.pcd";
  const fs::path temporary = root / "map.pcd.tmp";
  // A non-empty destination directory deterministically exercises an OS-level
  // backup cleanup failure, including when tests run as root.
  fs::create_directories(destination);
  std::ofstream(destination / "original") << "original";
  std::ofstream(temporary) << "replacement";

  const auto result = open_lmm::CommitFileSet({{temporary, destination}});
  Expect(!result, "backup cleanup failure must fail the transaction");
  if (!result) {
    Expect(result.GetError().severity == Error::Severity::kFatalSession &&
               result.GetError().context.node == "recovery_required",
           "cleanup failure must be a structured recovery-required error");
  }
  Expect(fs::exists(destination.string() + ".open_lmm_backup/original"),
         "cleanup failure must preserve the original in its backup");
  bool found_manifest = false;
  for (const auto& item : fs::directory_iterator(root)) {
    const std::string name = item.path().filename().string();
    if (name.starts_with(".open_lmm_recovery_") &&
        item.path().extension() == ".json") {
      found_manifest = true;
      break;
    }
  }
  Expect(found_manifest, "cleanup failure must leave a recovery manifest");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestFileSetRollbackMissingBackupRequiresRecovery() {
  namespace fs = std::filesystem;
  const fs::path root = fs::temp_directory_path() /
      ("open_lmm_file_rollback_recovery_" +
       std::to_string(std::random_device{}()));
  fs::create_directories(root);
  const fs::path first_final = root / "first.pcd";
  const fs::path first_temp = root / "first.tmp";
  const fs::path second_temp = root / "second.tmp";
  const fs::path third_temp = root / "third.tmp";
  std::ofstream(first_final) << "original";
  std::ofstream(first_temp) << "candidate-one";
  std::ofstream(second_temp) << "candidate-two";
  std::ofstream(third_temp) << "candidate-three";

  // The second replacement consumes the first entry's backup. The third then
  // fails, so rollback must detect that an acknowledged original no longer has
  // a backup instead of reporting a successful rollback.
  const auto result = open_lmm::CommitFileSet({
      {first_temp, first_final},
      {second_temp, first_final.string() + ".open_lmm_backup"},
      {third_temp, root / "missing" / "third.pcd"},
  });
  Expect(!result, "missing rollback backup must fail the transaction");
  if (!result) {
    Expect(result.GetError().severity == Error::Severity::kFatalSession &&
               result.GetError().context.node == "recovery_required" &&
               result.GetError().message.find("backup is missing") !=
                   std::string::npos,
           "missing rollback backup must require manual recovery");
  }
  bool found_manifest = false;
  for (const auto& item : fs::directory_iterator(root)) {
    if (item.path().filename().string().starts_with(
            ".open_lmm_recovery_")) {
      found_manifest = true;
      break;
    }
  }
  Expect(found_manifest,
         "rollback backup loss must leave a recovery manifest");
  std::error_code ignored;
  fs::remove_all(root, ignored);
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
  store.set_agent_map(Id("A"), anchor, Eigen::Isometry3d::Identity(),
                      open_lmm::AlignmentMethod::kKissMatcher,
                      open_lmm::AlignmentApproval::kAutomatic, Id("A"), 1);
  Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
  initial.translation() = Eigen::Vector3d(10.0, 0.0, 0.0);
  store.set_agent_map(Id("B"), follower, initial,
                      open_lmm::AlignmentMethod::kManual,
                      open_lmm::AlignmentApproval::kUser, Id("A"), 2);
  Expect(store.merged_map.size() == 2,
         "aligned map store must rebuild all accepted agent maps");
  Expect(store.merged_map[1].isApprox(Eigen::Vector3f(10.0F, 2.0F, 0.0F)),
         "accepted transform must place follower map in target frame");

  Eigen::Isometry3d optimized = Eigen::Isometry3d::Identity();
  optimized.translation() = Eigen::Vector3d(20.0, 0.0, 0.0);
  store.update_transform(Id("B"), optimized);
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
  {
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<UnknownThrowingNode>());
    auto result = pipeline.Run(contexts, db);
    Expect(!result, "unknown node exceptions must become pipeline failures");
    if (!result) {
      Expect(result.GetError().code == Error::Code::kInvalidArgument &&
                 result.GetError().context.stage == "pipeline" &&
                 result.GetError().context.node == "UnknownThrowing" &&
                 result.GetError().context.agent == Id("A"),
             "unknown pipeline exceptions must retain structured context");
    }
  }
  {
    auto contexts = OneContext();
    open_lmm::Pipeline pipeline;
    pipeline.AddNode(std::make_unique<EmptyNestedConfigNode>());
    auto result = pipeline.Run(contexts, db);
    Expect(!result, "empty nested paths must become pipeline failures");
    if (!result) {
      Expect(result.GetError().code == Error::Code::kInvalidArgument &&
                 result.GetError().context.stage == "pipeline" &&
                 result.GetError().context.node == "EmptyNestedConfig" &&
                 result.GetError().message.find("nested path") !=
                     std::string::npos,
             "empty nested paths must return structured InvalidArgument");
    }
  }
}

void TestPluginFailurePropagation() {
  auto result = open_lmm::load_plugin_v1<int>(
      "libopen_lmm_test_plugin_that_does_not_exist.so", "test", "{}");
  Expect(!result.IsOk(), "missing plugin must return an error");
  if (!result.IsOk()) {
    Expect(result.GetError().code == Error::Code::kPluginLoadFailed,
           "missing plugin must use plugin-load error code");
    Expect(result.GetError().context.plugin ==
               "libopen_lmm_test_plugin_that_does_not_exist.so",
           "plugin failure must retain plugin context");
  }
}

void TestOptimizerLifecycle() {
  const std::string config_path = "/tmp/open_lmm_optimizer_lifecycle.json";
  {
    std::ofstream output(config_path);
    output << R"({"backend_optimizer":{"backend_optimizer_type":"incremental","relinearizeThreshold":0.1,"relinearizeSkip":1,"isam_extra_updates":0,"min_loop_frame_gap":30,"icp_search_num":0}})";
  }

  auto parsed = open_lmm::ParseOptimizerConfig(open_lmm::Config(config_path));
  Expect(parsed.IsOk(), "optimizer test config must parse");
  open_lmm::BackendOptimizerIncremental optimizer{
      std::move(parsed).Value()};
  const auto catalog = Catalog();
  open_lmm::AgentContext anchor{
      .id = Id("A"), .symbol = catalog->SymbolFor(Id("A")).Value(),
      .catalog = catalog, .role = open_lmm::AgentRole::kAnchor, .order = 0};
  open_lmm::AgentRawData raw_a;
  raw_a.agent_id = Id("A");
  raw_a.odom_poses.push_back(Eigen::Isometry3d::Identity());
  raw_a.filtered_scans.push_back(OnePointScan());
  auto context = open_lmm::AlgorithmExecutionContext{};
  context.agent = anchor;
  context.cancellation = std::make_shared<open_lmm::CancellationToken>();
  context.operation = "optimizer.lifecycle";
  open_lmm::LoopPairVec no_intra;
  open_lmm::LoopPairVec no_inter;
  open_lmm::AgentRawDataMap no_other_raw;
  auto first = optimizer.Process(
      context, {raw_a, no_intra, no_inter, no_other_raw});
  Expect(first.IsOk(), "successful optimizer call must return Result success");
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             optimizer.HasProcessedAgent(Id("A")),
         "successful optimizer call must commit its agent");

  auto duplicate = optimizer.Process(
      context, {raw_a, no_intra, no_inter, no_other_raw});
  Expect(!duplicate &&
             duplicate.GetError().code == Error::Code::kOptimizationFailed,
         "duplicate optimizer agent must return failure");
  Expect(optimizer.ProcessedAgentCount() == 1,
         "duplicate failure must not mutate optimizer lifecycle");

  open_lmm::AgentContext follower{
      .id = Id("B"), .symbol = catalog->SymbolFor(Id("B")).Value(),
      .catalog = catalog, .role = open_lmm::AgentRole::kFollower, .order = 1};
  open_lmm::AgentRawData raw_b;
  raw_b.agent_id = Id("B");
  raw_b.odom_poses.push_back(Eigen::Isometry3d::Identity());
  raw_b.filtered_scans.push_back(OnePointScan());
  open_lmm::LoopPair inter_loop{
      .to = {Id("A"), 0},
      .from = {Id("B"), 0},
      .init_rel_pose = Eigen::Isometry3d::Identity()};
  context.agent = follower;
  open_lmm::LoopPairVec inter_loops{inter_loop};
  auto invalid_target = optimizer.Process(
      context, {raw_b, no_intra, inter_loops, no_other_raw});
  Expect(!invalid_target, "invalid optimizer input must return failure");
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             !optimizer.HasProcessedAgent(Id("B")),
         "failed optimizer transaction must preserve committed state");

  auto zero_factor = optimizer.Process(
      context, {raw_b, no_intra, no_inter, no_other_raw});
  Expect(!zero_factor,
         "follower without a refined inter-agent loop must return failure");
  Expect(optimizer.ProcessedAgentCount() == 1 &&
             !optimizer.HasProcessedAgent(Id("B")),
         "zero-factor rejection must preserve optimizer transaction");
  optimizer.Reset();
  Expect(optimizer.ProcessedAgentCount() == 0 &&
             !optimizer.HasProcessedAgent(Id("A")),
         "Reset must clear graph lifecycle metadata");

  auto cancellation = std::make_shared<open_lmm::CancellationToken>();
  cancellation->Request();
  context.agent = anchor;
  context.cancellation = cancellation;
  auto cancelled = optimizer.Process(
      context, {raw_a, no_intra, no_inter, no_other_raw});
  Expect(!cancelled && cancelled.GetError().code == Error::Code::kCancelled,
         "cancelled optimizer must return Cancelled");
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
  auto result = open_lmm::ParseDataLoaderConfig(open_lmm::Config(range_path));
  Expect(!result.IsOk(), "invalid DataLoader numeric range must fail");
  if (!result.IsOk()) {
    Expect(result.GetError().context.config == "data_loader",
           "typed config failure must retain config-domain context");
  }
  std::remove(range_path.c_str());

  const std::string custom_path = "/tmp/open_lmm_custom_pose_config.json";
  {
    std::ofstream output(custom_path);
    output << R"({"data_loader":{"data_loader_type":"file_based","pose_format":"custom","scan_type":"pcd","scan_dir_name":"Scans","pose_file_name":"poses.txt","voxel_size":0.2,"min_range":0.0,"max_range":100.0,"delimiter":" "}})";
  }
  auto custom = open_lmm::ParseDataLoaderConfig(open_lmm::Config(custom_path));
  Expect(!custom.IsOk(),
         "declared but unimplemented custom pose format must fail at bootstrap");
  std::remove(custom_path.c_str());

  try {
    open_lmm::Config empty_config("");
    empty_config.save("/tmp/open_lmm_missing_dir/config.json");
    Expect(false, "config save to missing directory must throw");
  } catch (const std::exception&) {
  }

  open_lmm::Config memory_config =
      open_lmm::Config::FromJson(R"({"outer":{"value":1}})",
                                 "nested-fixture");
  try {
    (void)memory_config.param_nested<int>({}, "value");
    Expect(false, "empty nested config path must fail");
  } catch (const std::invalid_argument& error) {
    Expect(std::string(error.what()).find("nested path") != std::string::npos,
           "empty nested config path must produce a contextual error");
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

void TestAlgorithmInvariants() {
  open_lmm::AgentRawData raw;
  raw.agent_id = Id("A");
  raw.odom_poses = {Eigen::Isometry3d::Identity(),
                    Eigen::Isometry3d::Identity()};
  raw.filtered_scans = {OnePointScan(), OnePointScan(1.0F)};
  Expect(open_lmm::ValidateAgentRawData(raw, "fixture").IsOk(),
         "valid raw algorithm input must pass");

  auto null_scan = raw;
  null_scan.filtered_scans[1].reset();
  Expect(!open_lmm::ValidateAgentRawData(null_scan, "fixture").IsOk(),
         "null raw scan must fail before algorithm entry");

  auto nonfinite_pose = raw;
  nonfinite_pose.odom_poses[0].translation().x() =
      std::numeric_limits<double>::quiet_NaN();
  Expect(!open_lmm::ValidateAgentRawData(nonfinite_pose, "fixture").IsOk(),
         "non-finite raw pose must fail before algorithm entry");

  Eigen::Isometry3d quantized_pose = Eigen::Isometry3d::Identity();
  // Real test1 frame 19: a proper rotation rounded to six decimal places.
  quantized_pose.matrix().topRows<3>() <<
      0.985941, 0.152187, -0.0689935, 14.2673,
      -0.149647, 0.987905, 0.0406254, -1.93745,
      0.0743416, -0.0297296, 0.99679, 0.81812;
  Expect(open_lmm::ValidateFiniteRigidPose(quantized_pose,
                                           "six-digit pose").IsOk(),
         "six-digit KITTI rotation quantization must remain valid");

  Eigen::Isometry3d sheared_pose = quantized_pose;
  sheared_pose.matrix()(0, 0) += 1e-3;
  Expect(!open_lmm::ValidateFiniteRigidPose(sheared_pose,
                                            "sheared pose").IsOk(),
         "materially non-orthonormal rotation must still fail");

  Eigen::Isometry3d reflected_pose = Eigen::Isometry3d::Identity();
  reflected_pose.matrix()(0, 0) = -1.0;
  Expect(!open_lmm::ValidateFiniteRigidPose(reflected_pose,
                                            "reflected pose").IsOk(),
         "orthonormal reflection must fail the proper-rotation check");

  open_lmm::LoopPair invalid_intra{
      .to = {Id("A"), 0},
      .from = {Id("B"), 1},
      .init_rel_pose = Eigen::Isometry3d::Identity()};
  Expect(!open_lmm::ValidateLoopPairs(raw, {invalid_intra}, {}, {},
                                      "fixture").IsOk(),
         "misclassified intra loop must fail");

  open_lmm::LoopPair invalid_frame{
      .to = {Id("A"), 0},
      .from = {Id("A"), 2},
      .init_rel_pose = Eigen::Isometry3d::Identity()};
  Expect(!open_lmm::ValidateLoopPairs(raw, {invalid_frame}, {}, {},
                                      "fixture").IsOk(),
         "out-of-range loop frame must fail");

  open_lmm::LoopPair inter{
      .to = {Id("B"), 0},
      .from = {Id("A"), 0},
      .init_rel_pose = Eigen::Isometry3d::Identity()};
  auto malformed_target = std::make_shared<open_lmm::AgentRawData>();
  malformed_target->agent_id = Id("B");
  malformed_target->odom_poses.push_back(Eigen::Isometry3d::Identity());
  malformed_target->filtered_scans.push_back(nullptr);
  open_lmm::AgentRawDataMap target_map{{Id("B"), malformed_target}};
  Expect(!open_lmm::ValidateLoopPairs(raw, {}, {inter}, target_map,
                                      "fixture").IsOk(),
         "inter-agent target null scan must fail before registration");
  malformed_target->filtered_scans[0] = OnePointScan();
  malformed_target->odom_poses[0].translation().x() =
      std::numeric_limits<double>::quiet_NaN();
  Expect(!open_lmm::ValidateLoopPairs(raw, {}, {inter}, target_map,
                                      "fixture").IsOk(),
         "inter-agent target non-finite pose must fail before registration");
  Expect(!open_lmm::FrameGapAtLeast(2, 5, 4) &&
             open_lmm::FrameGapAtLeast(5, 2, 3),
         "frame gap must be symmetric without unsigned underflow");

  open_lmm::AgentOptimizedData optimized;
  optimized.agent_id = Id("A");
  optimized.optimized_poses = {
      {1, Eigen::Isometry3d::Identity()},
      {0, Eigen::Isometry3d::Identity()},
  };
  optimized.kdtree_poses.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  optimized.kdtree_poses.push_back(pcl::PointXYZ(0.0F, 0.0F, 0.0F));
  Expect(open_lmm::ValidateOptimizedData(optimized, raw, "fixture").IsOk(),
         "shuffled optimized frames with complete IDs must pass");
  auto ordered = open_lmm::OrderOptimizedPosesByFrameId(
      optimized.optimized_poses, 2, "fixture");
  Expect(ordered.IsOk() && ordered.Value().size() == 2,
         "remover pose join must key by frame ID instead of vector position");

  optimized.optimized_poses[1].first = 1;
  Expect(!open_lmm::ValidateOptimizedData(optimized, raw, "fixture").IsOk(),
         "duplicate/missing optimized frame IDs must fail");

  auto invalid_cloud = OnePointScan();
  invalid_cloud->front().x = std::numeric_limits<float>::infinity();
  Expect(!open_lmm::ValidateVoxelizationInput(
              invalid_cloud, 0.2F, 0.0F, 0.0F, false, "fixture").IsOk(),
         "non-finite voxel input must fail");
  Expect(!open_lmm::ValidateVoxelizationInput(
              OnePointScan(), std::numeric_limits<float>::quiet_NaN(),
              0.0F, 0.0F, false, "fixture").IsOk(),
         "non-finite voxel size must fail");
}

void TestRemoverFrameIdentityAndDownsample() {
  auto plugin = std::make_shared<RecordingOnlineRemover>();
  open_lmm::DynamicRemoverConfig config;
  open_lmm::DynamicRemoverOnline remover(config, plugin);
  auto pose0 = Eigen::Isometry3d::Identity();
  pose0.translation().x() = 10.0;
  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation().x() = 20.0;
  const std::vector<std::pair<int, Eigen::Isometry3d>> shuffled{
      {1, pose1}, {0, pose0}};
  open_lmm::AlgorithmExecutionContext context;
  context.operation = "safety.dynamic_remover";
  auto processed = remover.Process(
      context, {{OnePointScan(), OnePointScan()}, shuffled});
  Expect(plugin->translations == std::vector<double>({10.0, 20.0}),
         "dynamic remover must join scan index to exact optimized frame ID");
  Expect(processed.IsOk() && processed.Value()->size() == 1,
         "online remover must return the downsampled map value");

  auto invalid_plugin = std::make_shared<RecordingOnlineRemover>();
  open_lmm::DynamicRemoverOnline invalid_remover(config, invalid_plugin);
  auto invalid = invalid_remover.Process(
      context, {{OnePointScan(), OnePointScan()},
                {{0, pose0}, {0, pose1}}});
  Expect(!invalid, "duplicate remover frame IDs must fail through Result");
  Expect(invalid_plugin->translations.empty(),
         "invalid remover frame IDs must fail before plugin invocation");
}

void TestOfflineStreamingFrameIdentity() {
  auto pose0 = Eigen::Isometry3d::Identity();
  pose0.translation().x() = 10.0;
  auto pose1 = Eigen::Isometry3d::Identity();
  pose1.translation().x() = 20.0;
  const std::vector<std::pair<int, Eigen::Isometry3d>> poses{
      {1, pose1}, {0, pose0}};

  auto plugin = std::make_shared<RecordingOfflineRemover>();
  open_lmm::DynamicRemoverConfig config;
  open_lmm::DynamicRemoverOffline remover(config, plugin);
  int traversals = 0;
  auto stateful_source = [&](const open_lmm::DynamicRemoverBase::RawScanVisitor& visitor) {
    ++traversals;
    // A second traversal deliberately changes IDs. Correct code never asks.
    const std::vector<std::pair<std::size_t, open_lmm::ScanVec::value_type>> entries =
        traversals == 1
            ? std::vector<std::pair<std::size_t, open_lmm::ScanVec::value_type>>{
                  {1, OnePointScan(1.0F)}, {0, OnePointScan(0.0F)}}
            : std::vector<std::pair<std::size_t, open_lmm::ScanVec::value_type>>{
                  {0, OnePointScan(99.0F)}, {0, OnePointScan(98.0F)}};
    std::size_t visited = 0;
    for (const auto& [frame, scan] : entries) {
      auto result = visitor(frame, scan);
      if (!result) return Result<std::size_t>::Failure(result.GetError());
      ++visited;
    }
    return Result<std::size_t>::Ok(visited);
  };
  open_lmm::AlgorithmExecutionContext context;
  context.operation = "safety.dynamic_remover.streaming";
  auto result = remover.ProcessStreaming(
      context, {stateful_source, poses, {}});
  Expect(result.IsOk() && traversals == 1,
         "offline remover must consume a stateful source exactly once");
  Expect(plugin->scan_x == std::vector<float>({0.0F, 1.0F}) &&
             plugin->translations == std::vector<double>({10.0, 20.0}),
         "offline remover must process cached scans by exact frame ID");

  auto cancelled_context = context;
  cancelled_context.cancellation =
      std::make_shared<open_lmm::CancellationToken>();
  cancelled_context.cancellation->Request();
  int cancelled_traversals = 0;
  open_lmm::DynamicRemoverBase::RawScanSource cancelled_source =
      [&](const open_lmm::DynamicRemoverBase::RawScanVisitor&) {
        ++cancelled_traversals;
        return Result<std::size_t>::Ok(0);
      };
  auto cancelled = remover.ProcessStreaming(
      cancelled_context, {cancelled_source, poses, {}});
  Expect(!cancelled && cancelled.GetError().code == Error::Code::kCancelled &&
             cancelled.GetError().context.node ==
                 "safety.dynamic_remover.streaming" &&
             cancelled_traversals == 0,
         "pre-cancelled streaming removal must preserve context and skip input");

  open_lmm::DynamicRemoverBase::RawScanSource throwing_source =
      [](const open_lmm::DynamicRemoverBase::RawScanVisitor&)
      -> Result<std::size_t> {
        throw std::runtime_error("stream source fault");
      };
  auto thrown = remover.ProcessStreaming(
      context, {throwing_source, poses, {}});
  Expect(!thrown && thrown.GetError().context.node ==
                         "safety.dynamic_remover.streaming",
         "stream source exceptions must become contextual Result failures");

  const auto expect_rejected_before_plugin = [&](const auto& entries,
                                                  const char* message) {
    auto invalid_plugin = std::make_shared<RecordingOfflineRemover>();
    open_lmm::DynamicRemoverOffline invalid_remover(config, invalid_plugin);
    auto source = [&](const open_lmm::DynamicRemoverBase::RawScanVisitor& visitor) {
      std::size_t visited = 0;
      for (const auto& [frame, scan] : entries) {
        auto item = visitor(frame, scan);
        if (!item) return Result<std::size_t>::Failure(item.GetError());
        ++visited;
      }
      return Result<std::size_t>::Ok(visited);
    };
    auto invalid = invalid_remover.ProcessStreaming(
        context, {source, poses, {}});
    Expect(!invalid && invalid_plugin->scan_x.empty(), message);
  };
  using IndexedScan =
      std::pair<std::size_t, open_lmm::ScanVec::value_type>;
  expect_rejected_before_plugin(
      std::vector<IndexedScan>{{0, OnePointScan()}, {0, OnePointScan()}},
      "duplicate source frames must fail before offline plugin invocation");
  expect_rejected_before_plugin(
      std::vector<IndexedScan>{{1, OnePointScan()}},
      "missing source frames must fail before offline plugin invocation");
  expect_rejected_before_plugin(
      std::vector<IndexedScan>{{0, OnePointScan()}, {1, OnePointScan()},
                               {2, OnePointScan()}},
      "extra source frames must fail before offline plugin invocation");
}

}  // namespace

int main() {
  TestAgentContext();
  TestFileSetCommitRollsBackPartialReplacement();
  TestFileSetCleanupFailureRequiresRecovery();
  TestFileSetRollbackMissingBackupRequiresRecovery();
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
  TestAlgorithmInvariants();
  TestRemoverFrameIdentityAndDownsample();
  TestOfflineStreamingFrameIdentity();
  if (failures != 0) {
    std::cerr << failures << " safety regression test(s) failed\n";
    return 1;
  }
  std::cout << "All safety regression tests passed\n";
  return 0;
}
