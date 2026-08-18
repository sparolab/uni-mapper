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

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

[[noreturn]] void Fail(const std::string& message) {
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

template <typename T>
void Require(const Result<T>& result, const std::string& operation) {
  if (!result) Fail(operation + ": " + result.GetError().Message());
}

CommittedSessionSnapshot RequireSessionSnapshot(const MapServer& server) {
  const auto snapshot = server.Snapshot();
  if (snapshot.revision == 0) Fail("committed session snapshot is unavailable");
  return snapshot;
}

VisualizationSnapshot RequireVisualizationSnapshot(const MapServer& server,
                                                    const AgentId& agent) {
  auto snapshot = server.Visualization(agent);
  if (!snapshot) {
    Fail("visualization snapshot for agent " + agent.Value() +
         ": " + snapshot.GetError().Message());
  }
  return std::move(snapshot).Value();
}

Result<ExecutionReceipt> Execute(MapServer& server,
                                 ExecutionCommand command) {
  return server.Execute(
      command,
      {std::make_shared<CancellationToken>(),
       std::make_shared<AlignmentFeedbackBroker>(),
       server.Snapshot().revision});
}

ArtifactState StateOf(const CommittedSessionSnapshot& snapshot,
                      ArtifactType type, std::optional<AgentId> agent) {
  for (const auto& artifact : snapshot.artifacts) {
    if (artifact.key == ArtifactKey{type, agent}) return artifact.state;
  }
  return ArtifactState::kMissing;
}

using EdgeKey =
    std::tuple<AgentId, std::size_t, AgentId, std::size_t, VisualizationEdgeType>;

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
    Fail("pose count changed for agent " + expected.agent.Value());
  }
  if (EdgeKeys(expected) != EdgeKeys(actual)) {
    Fail("trajectory/loop edge set changed for agent " +
         expected.agent.Value());
  }

  PoseDifference difference;
  for (std::size_t index = 0; index < expected.poses.size(); ++index) {
    const auto& lhs = expected.poses[index];
    const auto& rhs = actual.poses[index];
    if (lhs.index != rhs.index) {
      Fail("pose ordering changed for agent " +
           expected.agent.Value());
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
         expected.agent.Value());
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
  const AgentId test1 = Id("test1");
  const AgentId test2 = Id("test2");
  if (server.Snapshot().ordered_agents !=
      std::vector<AgentId>({test1, test2})) {
    Fail("replay verification requires exactly test1/test2");
  }
  Require(Execute(server, ExecutionCommand::Stage(StageId::kDataLoad)),
          "DataLoad");
  Require(Execute(server, ExecutionCommand::Stage(StageId::kAlignment)),
          "full Alignment");

  const auto full_state = RequireSessionSnapshot(server);
  const auto full_a = RequireVisualizationSnapshot(server, test1);
  const auto full_b = RequireVisualizationSnapshot(server, test2);
  if (full_a.revision != full_state.revision ||
      full_b.revision != full_state.revision) {
    Fail("visualization snapshot must use the committed session revision");
  }

  Require(Execute(server, ExecutionCommand::Node(NodeId::kLoopDetect, test2)),
          "LoopDetect(test2) replay 1");
  const auto loop_once = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(full_state, loop_once, "LoopDetect(B) replay 1");
  if (loop_once.revision <= full_state.revision ||
      StateOf(loop_once, ArtifactType::kLoopCandidates, test1) !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kLoopCandidates, test2) !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kOptimizedPoses, test1) !=
          ArtifactState::kReady ||
      StateOf(loop_once, ArtifactType::kOptimizedPoses, test2) !=
          ArtifactState::kStale) {
    Fail("LoopDetect(B) replay committed inconsistent artifact metadata");
  }

  Require(Execute(server, ExecutionCommand::Node(NodeId::kLoopDetect, test2)),
          "LoopDetect(test2) replay 2");
  const auto loop_twice = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(loop_once, loop_twice, "LoopDetect(B) replay 2");
  if (loop_twice.revision <= loop_once.revision) {
    Fail("repeated LoopDetect(B) did not advance the session revision");
  }

  Require(Execute(server, ExecutionCommand::Node(NodeId::kOptimize, test2)),
          "Optimize(test2) replay");
  const auto replay_state = RequireSessionSnapshot(server);
  VerifyDescriptorCounts(full_state, replay_state, "Optimize(B) replay");
  if (replay_state.revision <= loop_twice.revision ||
      StateOf(replay_state, ArtifactType::kOptimizedPoses, test1) !=
          ArtifactState::kReady ||
      StateOf(replay_state, ArtifactType::kOptimizedPoses, test2) !=
          ArtifactState::kReady) {
    Fail("Optimize(B) replay committed inconsistent artifact metadata");
  }

  const auto replay_a = RequireVisualizationSnapshot(server, test1);
  const auto replay_b = RequireVisualizationSnapshot(server, test2);
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
            << " test1=" << replay_state.per_agent_descriptor_count.at(test1)
            << " test2=" << replay_state.per_agent_descriptor_count.at(test2)
            << '\n'
            << "  A: poses=" << replay_a.poses.size()
            << " max_translation_m=" << difference_a.max_translation_m
            << " max_rotation_rad=" << difference_a.max_rotation_rad << '\n'
            << "  B: poses=" << replay_b.poses.size()
            << " max_translation_m=" << difference_b.max_translation_m
            << " max_rotation_rad=" << difference_b.max_rotation_rad << '\n';
  return 0;
}
