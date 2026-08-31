#include <open_lmm/core/alignment/alignment_proposer.hpp>
#include <open_lmm/core/alignment/loop_constraint_builder.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <set>
#include <tuple>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

AlgorithmExecutionContext Context(const AgentId& source) {
  AlgorithmExecutionContext context;
  context.agent.id = source;
  context.cancellation = std::make_shared<CancellationToken>();
  context.config = std::make_shared<const AlgorithmConfigSnapshot>(
      AlgorithmConfigSnapshot{"alignment.fixture", 1, "{}", "fixture"});
  context.base_revision = 9;
  context.operation = "loop_constraint_build";
  context.plugin_id = "fixture.proposer";
  return context;
}

AgentRawDataHandle Raw(const AgentId& id) {
  auto raw = std::make_shared<AgentRawData>();
  raw->agent_id = id;
  for (int frame = 0; frame < 3; ++frame) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = 11.0 * frame;
    raw->odom_poses.push_back(pose);
    auto scan = pcl::PointCloud<pcl::PointXYZI>::Ptr(
        new pcl::PointCloud<pcl::PointXYZI>());
    scan->push_back(pcl::PointXYZI{});
    raw->filtered_scans.push_back(std::move(scan));
  }
  return raw;
}

AgentOptimizedDataHandle Optimized(const AgentId& id) {
  auto optimized = std::make_shared<AgentOptimizedData>();
  optimized->agent_id = id;
  for (int frame = 0; frame < 3; ++frame) {
    Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
    pose.translation().x() = 11.0 * frame;
    optimized->optimized_poses.push_back({frame, pose});
    optimized->kdtree_poses.push_back(
        pcl::PointXYZ(static_cast<float>(11.0 * frame), 0.0F, 0.0F));
  }
  return optimized;
}

Eigen::Isometry3d Pose(double x, double y, double yaw_radians) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = Eigen::AngleAxisd(yaw_radians, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();
  pose.translation() = Eigen::Vector3d(x, y, 0.0);
  return pose;
}

AgentOptimizedDataHandle ShuffledOptimized(
    const AgentId& id, const std::array<Eigen::Isometry3d, 3>& poses) {
  auto optimized = std::make_shared<AgentOptimizedData>();
  optimized->agent_id = id;
  for (const int frame : {2, 0, 1}) {
    optimized->optimized_poses.push_back({frame, poses[frame]});
    optimized->kdtree_poses.push_back(pcl::PointXYZ(
        static_cast<float>(poses[frame].translation().x()),
        static_cast<float>(poses[frame].translation().y()), 0.0F));
  }
  return optimized;
}

MapAlignmentProposal Proposal(const AgentId& target, const AgentId& source,
                              AlignmentMethod method) {
  MapAlignmentProposal proposal;
  proposal.target_agent = target;
  proposal.source_agent = source;
  proposal.method = method;
  return proposal;
}

void TestProposerBoundary() {
  const AgentId target = Id("B");
  const AgentId source = Id("A");
  auto context = Context(source);
  AlignmentProposer proposer([&](const AlignmentProposalRequest& request) {
    return Result<std::optional<MapAlignmentProposal>>::Ok(
        Proposal(request.target_agent, request.source_agent,
                 AlignmentMethod::kDescriptor));
  });
  auto result = proposer.Propose(context, {target, source});
  Check(result && result.Value() &&
            result.Value()->method == AlignmentMethod::kDescriptor,
        "pure proposer must publish a valid proposal");

  AlignmentProposer policy_leak([&](const AlignmentProposalRequest& request) {
    return Result<std::optional<MapAlignmentProposal>>::Ok(
        Proposal(request.target_agent, request.source_agent,
                 AlignmentMethod::kManual));
  });
  auto rejected = policy_leak.Propose(context, {target, source});
  Check(!rejected && rejected.GetError().context.runtime_revision == 9,
        "proposer must reject policy-owned methods with context");

  context.cancellation->Request();
  int calls = 0;
  AlignmentProposer cancelled([&](const AlignmentProposalRequest&) {
    ++calls;
    return Result<std::optional<MapAlignmentProposal>>::Ok(std::nullopt);
  });
  auto stopped = cancelled.Propose(context, {target, source});
  Check(!stopped && calls == 0,
        "pre-cancellation must not enter the proposal algorithm");
}

void TestAllApprovalPathsShareConstraintBuilder() {
  const AgentId source = Id("A");
  const AgentId target = Id("B");
  const auto source_raw = Raw(source);
  const auto target_raw = Raw(target);
  AgentRawDataMap raw{{source, source_raw}, {target, target_raw}};
  AgentOptimizedDataMap optimized{{target, Optimized(target)}};
  auto context = Context(source);
  LoopConstraintBuilder builder;

  LoopPairVec reference;
  for (const auto& accepted : {
           std::pair{StoredAlignment{Proposal(target, source,
                                              AlignmentMethod::kKissMatcher),
                                     AlignmentApproval::kAutomatic, 1},
                     AlignmentAcceptanceSource::kAutomatic},
           std::pair{StoredAlignment{Proposal(target, source,
                                              AlignmentMethod::kDescriptor),
                                     AlignmentApproval::kUser, 2},
                     AlignmentAcceptanceSource::kInteractive},
           std::pair{StoredAlignment{Proposal(target, source,
                                              AlignmentMethod::kDescriptor),
                                     AlignmentApproval::kUser, 3},
                     AlignmentAcceptanceSource::kStored},
       }) {
    auto built = builder.Build(
        context, {accepted.first, accepted.second, *source_raw, raw, optimized,
                  1.0, 10.0});
    Check(built && built.Value().loops.size() == 2,
          "every accepted path must produce the same validated pose-NN loops");
    if (reference.empty()) {
      reference = built.Value().loops;
    } else {
      Check(reference.size() == built.Value().loops.size(),
            "approval paths must preserve loop cardinality");
      for (std::size_t index = 0; index < reference.size(); ++index) {
        Check(reference[index].from == built.Value().loops[index].from &&
                  reference[index].to == built.Value().loops[index].to &&
                  reference[index].init_rel_pose.matrix().isApprox(
                      built.Value().loops[index].init_rel_pose.matrix()),
              "approval paths must preserve identical loop constraints");
      }
    }
  }

  auto malformed = Proposal(target, source, AlignmentMethod::kDescriptor);
  malformed.target_T_source.translation().x() =
      std::numeric_limits<double>::quiet_NaN();
  auto rejected = builder.Build(
      context,
      {StoredAlignment{malformed, AlignmentApproval::kAutomatic, 4},
       AlignmentAcceptanceSource::kAutomatic, *source_raw, raw, optimized,
       1.0, 10.0});
  Check(!rejected && rejected.GetError().context.node ==
                         "loop_constraint_build",
        "all approval paths must reject malformed transforms contextually");
}

void TestTransformDirectionOrdinalAndMultipleTargets() {
  constexpr double kPi = 3.14159265358979323846;
  const AgentId source = Id("A");
  const AgentId target_b = Id("B");
  const AgentId target_c = Id("C");
  const AgentId threshold_d = Id("D");
  auto source_raw = std::make_shared<AgentRawData>(*Raw(source));
  for (std::size_t frame = 0; frame < source_raw->odom_poses.size(); ++frame) {
    source_raw->odom_poses[frame] =
        Pose(11.0 * static_cast<double>(frame), 0.0,
             static_cast<double>(frame) * 10.0 * kPi / 180.0);
  }
  const auto raw_b = Raw(target_b);
  const auto raw_c = Raw(target_c);
  const auto raw_d = Raw(threshold_d);
  AgentRawDataMap raw{{source, source_raw},
                      {target_b, raw_b},
                      {target_c, raw_c},
                      {threshold_d, raw_d}};

  StoredAlignment accepted;
  accepted.proposal =
      Proposal(target_b, source, AlignmentMethod::kDescriptor);
  accepted.proposal.target_T_source = Pose(5.0, 7.0, kPi / 2.0);
  accepted.approval = AlignmentApproval::kUser;

  std::array<Eigen::Isometry3d, 3> global_source;
  std::array<Eigen::Isometry3d, 3> poses_b;
  std::array<Eigen::Isometry3d, 3> poses_c;
  std::array<Eigen::Isometry3d, 3> poses_d;
  for (std::size_t frame = 0; frame < global_source.size(); ++frame) {
    global_source[frame] =
        accepted.proposal.target_T_source * source_raw->odom_poses[frame];
    poses_b[frame] = global_source[frame];
    poses_b[frame].linear() =
        Eigen::AngleAxisd((30.0 + frame) * kPi / 180.0,
                          Eigen::Vector3d::UnitZ())
            .toRotationMatrix();
    poses_c[frame] = poses_b[frame];
    poses_c[frame].translation().x() += 0.5;
    poses_d[frame] = poses_b[frame];
    poses_d[frame].translation().x() += 1.0;
  }
  AgentOptimizedDataMap optimized{
      {target_b, ShuffledOptimized(target_b, poses_b)},
      {target_c, ShuffledOptimized(target_c, poses_c)},
      {threshold_d, ShuffledOptimized(threshold_d, poses_d)}};

  auto built = LoopConstraintBuilder().Build(
      Context(source),
      {accepted, AlignmentAcceptanceSource::kStored, *source_raw, raw,
       optimized, 1.0, 10.0});
  Check(built && built.Value().loops.size() == 4,
        "two prior agents must contribute while strict threshold equality is excluded");

  std::set<std::tuple<AgentId, std::size_t, AgentId, std::size_t>> identities;
  for (const auto& loop : built.Value().loops) {
    Check(loop.to.first != threshold_d,
          "a nearest neighbor exactly at the threshold must be excluded");
    Check(identities.emplace(loop.from.first, loop.from.second, loop.to.first,
                             loop.to.second)
              .second,
          "generated loop pairs must be unique");
    Check(loop.to.second == loop.from.second,
          "KD-tree ordinal must map back to the shuffled frame ID");
    const auto& target_pose = loop.to.first == target_b
                                  ? poses_b[loop.to.second]
                                  : poses_c[loop.to.second];
    const Eigen::Isometry3d expected =
        target_pose.inverse() * global_source[loop.from.second];
    Check(loop.init_rel_pose.matrix().isApprox(expected.matrix(), 1.0e-9),
          "non-commuting target/source poses must produce target_scan_T_source_scan");
  }
}

void TestNoNeighborDiagnostics() {
  const AgentId source = Id("A");
  const AgentId target = Id("B");
  const auto source_raw = Raw(source);
  const auto target_raw = Raw(target);
  AgentRawDataMap raw{{source, source_raw}, {target, target_raw}};
  AgentOptimizedDataMap optimized{{target, Optimized(target)}};
  StoredAlignment accepted{
      Proposal(target, source, AlignmentMethod::kManual),
      AlignmentApproval::kUser, 0};
  accepted.proposal.target_T_source.translation().x() = 5.0;
  LoopConstraintBuildDiagnostics diagnostics;
  auto built = LoopConstraintBuilder().Build(
      Context(source),
      {accepted, AlignmentAcceptanceSource::kInteractive, *source_raw, raw,
       optimized, 1.0, 10.0, &diagnostics});
  Check(!built && diagnostics.search_completed &&
            diagnostics.sampled_source_frames == 2 &&
            diagnostics.target_frames == 3 &&
            diagnostics.within_radius == 0 &&
            std::abs(diagnostics.nearest_distance_m - 5.0) < 1.0e-6 &&
            diagnostics.threshold_m == 1.0,
        "empty manual NN search returns complete numeric diagnostics");
}

}  // namespace

int main() {
  TestProposerBoundary();
  TestAllApprovalPathsShareConstraintBuilder();
  TestTransformDirectionOrdinalAndMultipleTargets();
  TestNoNeighborDiagnostics();
  std::cout << "Alignment contract fixture tests passed\n";
  return EXIT_SUCCESS;
}
