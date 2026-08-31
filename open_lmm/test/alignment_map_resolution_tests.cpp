#include <domain/loop_detection/alignment_map_builder.hpp>
#include <domain/loop_detection/kiss_alignment_proposer.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <vector>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

AlgorithmExecutionContext Context() {
  AlgorithmExecutionContext context;
  context.agent.id = Id("A");
  context.cancellation = std::make_shared<CancellationToken>();
  context.operation = "alignment_map_build";
  context.plugin_id = "fixture.kiss";
  return context;
}

AgentRawData Raw() {
  AgentRawData raw;
  raw.agent_id = Id("A");
  raw.odom_poses.push_back(Eigen::Isometry3d::Identity());
  auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  scan->push_back(pcl::PointXYZI{0.1F, 0.0F, 0.0F, 0.0F});
  scan->push_back(pcl::PointXYZI{0.2F, 0.0F, 0.0F, 0.0F});
  scan->push_back(pcl::PointXYZI{1.1F, 0.0F, 0.0F, 0.0F});
  raw.filtered_scans.push_back(std::move(scan));
  return raw;
}

void TestConfiguredResolutionBuildsDistinctArtifacts() {
  auto context = Context();
  const auto raw = Raw();
  auto fine = BuildAlignmentMap(context, raw, 0.5F);
  auto coarse = BuildAlignmentMap(context, raw, 2.0F);
  Check(fine.IsOk() && coarse.IsOk(), "alignment maps must build");
  Check(fine.Value().voxel_size_m == 0.5F &&
            coarse.Value().voxel_size_m == 2.0F,
        "alignment map must retain its selected resolution");
  Check(fine.Value().points.size() == 2,
        "0.5m alignment map must retain two occupied voxels");
  Check(coarse.Value().points.size() == 1,
        "2m alignment map must merge the fixture into one occupied voxel");
}

void TestAlignmentMapReportsFrameProgress() {
  auto context = Context();
  std::vector<AlgorithmProgress> progress;
  context.progress = [&](const AlgorithmProgress& update) {
    progress.push_back(update);
  };
  auto built = BuildAlignmentMap(context, Raw(), 0.5F);
  Check(built.IsOk(), "progress fixture alignment map must build");
  Check(progress.size() == 2,
        "alignment map must report start and every completed frame");
  Check(progress.front().phase ==
            AlgorithmProgressPhase::kBuildAlignmentMap &&
            progress.front().current == 0 && progress.front().total == 1 &&
            progress.back().current == 1 && progress.back().total == 1,
        "alignment map progress must expose exact 0/N through N/N");
}

void TestKissRunnerReceivesSelectedResolutionAndArtifact() {
  auto context = Context();
  auto map = BuildAlignmentMap(context, Raw(), 0.5F);
  Check(map.IsOk(), "KISS input alignment map must build");
  float observed_leaf = 0.0F;
  std::size_t observed_source = 0;
  std::size_t observed_target = 0;
  KissAlignmentProposer proposer(
      [&](const auto& source, const auto& target, float leaf,
          bool use_quatro) {
        observed_leaf = leaf;
        observed_source = source.size();
        observed_target = target.size();
        Check(!use_quatro, "fixture must preserve the QUATRO selection");
        KissMatcherEstimate estimate;
        estimate.final_inliers = 5;
        estimate.rotation_inliers = 5;
        return estimate;
      });
  const auto proposal = proposer.Propose(
      map.Value().points, map.Value().points, Id("A"), Id("B"), 0.5F,
      false);
  Check(proposal.has_value(), "injected KISS estimate must be accepted");
  Check(observed_leaf == 0.5F,
        "configured KISS leaf must reach the matcher runner");
  Check(observed_source == map.Value().points.size() &&
            observed_target == map.Value().points.size(),
        "matcher must receive the exact configured-resolution artifact");
}

void TestDescriptorStoreRejectsMixedMapResolutions() {
  DescriptorStore store;
  VoxelizedAgentMap fine{0.5F, {{0.0F, 0.0F, 0.0F}}};
  VoxelizedAgentMap coarse{2.0F, {{1.0F, 0.0F, 0.0F}}};
  auto first = store.set_agent_map(
      Id("A"), fine, Eigen::Isometry3d::Identity(),
      AlignmentMethod::kKissMatcher, AlignmentApproval::kAutomatic, Id("A"),
      1);
  Check(first.IsOk(), "first configured-resolution map must be accepted");
  auto mixed = store.set_agent_map(
      Id("B"), coarse, Eigen::Isometry3d::Identity(),
      AlignmentMethod::kKissMatcher, AlignmentApproval::kAutomatic, Id("A"),
      2);
  Check(!mixed.IsOk(), "mixed alignment-map resolutions must be rejected");
  Check(store.aligned_maps.size() == 1 && store.merged_map.size() == 1,
        "rejected mixed resolution must not mutate the descriptor store");
}

void TestCancellationPreventsMapPublication() {
  auto context = Context();
  context.cancellation->Request();
  auto result = BuildAlignmentMap(context, Raw(), 0.5F);
  Check(!result.IsOk() && result.GetError().code == Error::Code::kCancelled,
        "cancelled alignment-map build must not publish an artifact");
}

}  // namespace

int main() {
  TestConfiguredResolutionBuildsDistinctArtifacts();
  TestAlignmentMapReportsFrameProgress();
  TestKissRunnerReceivesSelectedResolutionAndArtifact();
  TestDescriptorStoreRejectsMixedMapResolutions();
  TestCancellationPreventsMapPublication();
  std::cout << "alignment map resolution tests passed\n";
  return EXIT_SUCCESS;
}
