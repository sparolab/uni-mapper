#include <open_lmm/server/map_server.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/logging.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

namespace open_lmm {
namespace {

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

void Require(const Result<void>& result, const std::string& operation) {
  if (!result) Fail(operation + ": " + result.GetError().Message());
}

CommittedSessionSnapshot RequireSessionSnapshot(const MapServer& server) {
  const auto snapshot = server.SessionSnapshot();
  if (!snapshot) Fail("committed session snapshot is unavailable");
  return *snapshot;
}

VisualizationSnapshot RequireVisualizationSnapshot(const MapServer& server,
                                                    char agent) {
  auto snapshot = server.CreateVisualizationSnapshot(agent);
  if (!snapshot) {
    Fail("visualization snapshot for agent " + std::string(1, agent) +
         ": " + snapshot.GetError().Message());
  }
  return std::move(snapshot).Value();
}

ArtifactState StateOf(const CommittedSessionSnapshot& snapshot,
                      ArtifactType type, std::optional<char> agent) {
  for (const auto& artifact : snapshot.artifacts) {
    if (artifact.key == ArtifactKey{type, agent}) return artifact.state;
  }
  return ArtifactState::kMissing;
}

using EdgeKey =
    std::tuple<char, std::size_t, char, std::size_t, VisualizationEdgeType>;

std::vector<EdgeKey> EdgeKeys(const VisualizationSnapshot& snapshot) {
  std::vector<EdgeKey> result;
  result.reserve(snapshot.edges.size());
  for (const auto& edge : snapshot.edges) {
    result.emplace_back(edge.from_agent, edge.from_index, edge.to_agent,
                        edge.to_index, edge.type);
  }
  std::sort(result.begin(), result.end());
  return result;
}

struct PoseDifference {
  double max_translation_m = 0.0;
  double max_rotation_rad = 0.0;
};

PoseDifference CompareAgent(const VisualizationSnapshot& expected,
                            const VisualizationSnapshot& actual,
                            double translation_tolerance_m,
                            double rotation_tolerance_rad) {
  if (expected.agent != actual.agent) Fail("agent identity changed");
  if (expected.poses.size() != actual.poses.size()) {
    Fail("pose count changed for agent " + std::string(1, expected.agent));
  }
  if (EdgeKeys(expected) != EdgeKeys(actual)) {
    Fail("trajectory/loop edge set changed for agent " +
         std::string(1, expected.agent));
  }

  PoseDifference difference;
  for (std::size_t index = 0; index < expected.poses.size(); ++index) {
    const auto& lhs = expected.poses[index];
    const auto& rhs = actual.poses[index];
    if (lhs.index != rhs.index) {
      Fail("pose ordering changed for agent " +
           std::string(1, expected.agent));
    }
    const Eigen::Isometry3d delta =
        lhs.transform.cast<double>().inverse() * rhs.transform.cast<double>();
    difference.max_translation_m =
        std::max(difference.max_translation_m, delta.translation().norm());
    difference.max_rotation_rad = std::max(
        difference.max_rotation_rad,
        std::abs(Eigen::AngleAxisd(delta.linear()).angle()));
  }
  if (difference.max_translation_m > translation_tolerance_m ||
      difference.max_rotation_rad > rotation_tolerance_rad) {
    Fail("ordered replay exceeded pose tolerance for agent " +
         std::string(1, expected.agent));
  }
  return difference;
}

void VerifyDescriptorCounts(const CommittedSessionSnapshot& expected,
                            const CommittedSessionSnapshot& actual,
                            const std::string& operation) {
  if (actual.descriptor_count != expected.descriptor_count ||
      actual.per_agent_descriptor_count !=
          expected.per_agent_descriptor_count) {
    Fail(operation + " changed descriptor cardinality");
  }
}

}  // namespace
}  // namespace open_lmm

int main(int argc, char** argv) {
  using namespace open_lmm;
  InitializeLogging();
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_dir_path>\n";
    return 2;
  }

  GlobalConfig::instance(argv[1]);
  MapServer server;
  Require(server.ValidateReady(), "validate configuration");
  if (server.AgentIds() != std::vector<char>({'A', 'B'})) {
    Fail("replay verification requires exactly test1/test2 (agents A/B)");
  }
  Require(server.RunStage(StageId::kDataLoad), "DataLoad");
  Require(server.RunStage(StageId::kAlignment), "full Alignment");

  const auto full_state = RequireSessionSnapshot(server);
  const auto full_a = RequireVisualizationSnapshot(server, 'A');
  const auto full_b = RequireVisualizationSnapshot(server, 'B');
  if (full_a.revision != full_state.revision ||
      full_b.revision != full_state.revision) {
    Fail("visualization snapshot must use the committed session revision");
  }

  Require(server.RunNode(NodeId::kLoopDetect, 'B'), "LoopDetect(B) replay 1");
  const auto loop_once = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(full_state, loop_once, "LoopDetect(B) replay 1");
  if (loop_once.revision <= full_state.revision ||
      StateOf(loop_once, ArtifactType::kLoopCandidates, 'A') !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kLoopCandidates, 'B') !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kOptimizedPoses, 'A') !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kOptimizedPoses, 'B') !=
          ArtifactState::kStale) {
    Fail("LoopDetect(B) replay committed inconsistent artifact metadata");
  }

  Require(server.RunNode(NodeId::kLoopDetect, 'B'), "LoopDetect(B) replay 2");
  const auto loop_twice = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(loop_once, loop_twice, "LoopDetect(B) replay 2");
  if (loop_twice.revision <= loop_once.revision) {
    Fail("repeated LoopDetect(B) did not advance the session revision");
  }

  Require(server.RunNode(NodeId::kOptimize, 'B'), "Optimize(B) replay");
  const auto replay_state = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(full_state, replay_state, "Optimize(B) replay");
  if (replay_state.revision <= loop_twice.revision ||
      StateOf(replay_state, ArtifactType::kOptimizedPoses, 'A') !=
          ArtifactState::kReady ||
      StateOf(replay_state, ArtifactType::kOptimizedPoses, 'B') !=
          ArtifactState::kReady) {
    Fail("Optimize(B) replay committed inconsistent artifact metadata");
  }

  const auto replay_a = RequireVisualizationSnapshot(server, 'A');
  const auto replay_b = RequireVisualizationSnapshot(server, 'B');
  if (replay_a.revision != replay_state.revision ||
      replay_b.revision != replay_state.revision) {
    Fail("replay visualization must use the committed session revision");
  }
  constexpr double kTranslationToleranceM = 1e-3;
  constexpr double kRotationToleranceRad = 1e-4;
  const auto difference_a = CompareAgent(
      full_a, replay_a, kTranslationToleranceM, kRotationToleranceRad);
  const auto difference_b = CompareAgent(
      full_b, replay_b, kTranslationToleranceM, kRotationToleranceRad);

  std::cout << "ordered replay verification passed\n"
            << "  revisions: full=" << full_state.revision
            << " loop1=" << loop_once.revision
            << " loop2=" << loop_twice.revision
            << " optimize=" << replay_state.revision << '\n'
            << "  descriptors: total=" << replay_state.descriptor_count
            << " A=" << replay_state.per_agent_descriptor_count.at('A')
            << " B=" << replay_state.per_agent_descriptor_count.at('B')
            << '\n'
            << "  A: poses=" << replay_a.poses.size()
            << " max_translation_m=" << difference_a.max_translation_m
            << " max_rotation_rad=" << difference_a.max_rotation_rad << '\n'
            << "  B: poses=" << replay_b.poses.size()
            << " max_translation_m=" << difference_b.max_translation_m
            << " max_rotation_rad=" << difference_b.max_rotation_rad << '\n';
  return 0;
}
