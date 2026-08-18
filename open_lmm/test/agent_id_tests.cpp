#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/result.hpp>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <cstdlib>
#include <iostream>
#include <set>
#include <string>
#include <vector>

namespace {
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

AgentId Id(const std::string& value) {
  auto parsed = AgentId::Parse(value);
  Check(parsed.IsOk(), "test AgentId parses");
  return parsed.Value();
}

void TestLexicalAndCatalogContracts() {
  for (const char* invalid : {"", ".", "..", "_agent", "/agent",
                              "agent/name", "agent\\name", "agent name",
                              "agent\nname"}) {
    Check(!AgentId::Parse(invalid), "invalid AgentId rejected");
  }
  Check(!AgentId::Parse(std::string(65, 'a')),
        "AgentId longer than 64 characters rejected");
  Check(AgentId::Parse("test1") && AgentId::Parse("robot-2.map"),
        "valid AgentIds accepted");
  Check(!AgentSymbol::FromByte(0) && !AgentSymbol::FromByte(256),
        "reserved and overflowing symbol bytes rejected");
  Check(!AgentSymbolCatalog::Build({Id("robot"), Id("Robot")}),
        "case-only duplicate AgentIds rejected");
  Check(!AgentSymbolCatalog::Build({Id("robot"), Id("robot")}),
        "exact duplicate AgentIds rejected");

  std::vector<AgentId> overflow;
  for (int index = 0; index < 256; ++index) {
    overflow.push_back(Id("agent" + std::to_string(index)));
  }
  Check(!AgentSymbolCatalog::Build(std::move(overflow)),
        "256th agent rejected");
}

void TestRealGtsamGraphSupports255Agents() {
  std::vector<AgentId> ids;
  ids.reserve(AgentSymbolCatalog::kMaximumAgents);
  for (std::size_t index = 0; index < AgentSymbolCatalog::kMaximumAgents;
       ++index) {
    ids.push_back(Id("agent" + std::to_string(index)));
  }
  auto built = AgentSymbolCatalog::Build(ids);
  Check(built.IsOk(), "255-agent catalog builds");
  const AgentSymbolCatalog catalog = std::move(built).Value();

  gtsam::NonlinearFactorGraph graph;
  gtsam::Values initial;
  std::set<gtsam::Key> keys;
  const auto tight = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());
  const auto between = gtsam::noiseModel::Diagonal::Sigmas(
      (gtsam::Vector(6) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1).finished());

  gtsam::Key previous_pose = 0;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    const auto symbol = catalog.SymbolFor(ids[index]);
    Check(symbol.IsOk(), "configured AgentId resolves to a symbol");
    const unsigned char byte = symbol.Value().Byte();
    const gtsam::Symbol anchor(byte, ANCHOR_IDX);
    const gtsam::Symbol pose0(byte, 0);
    const gtsam::Symbol pose1(byte, 1);
    Check(keys.insert(anchor.key()).second && keys.insert(pose0.key()).second &&
              keys.insert(pose1.key()).second,
          "GTSAM keys are unique across all agents and frames");

    const gtsam::Pose3 origin(
        gtsam::Rot3(), gtsam::Point3(static_cast<double>(index), 0.0, 0.0));
    initial.insert(anchor, origin);
    initial.insert(pose0, origin);
    initial.insert(pose1, origin.compose(gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0))));
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(anchor, origin, tight));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        anchor, pose0, gtsam::Pose3(), between));
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
        pose0, pose1,
        gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)), between));
    if (index > 0) {
      graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          previous_pose, pose0, gtsam::Pose3(), between));
    }
    previous_pose = pose1.key();
  }

  Check(keys.size() == 255 * 3, "all synthetic graph keys were inserted");
  const auto optimized =
      gtsam::LevenbergMarquardtOptimizer(graph, initial).optimize();
  Check(optimized.size() == keys.size(), "GTSAM optimized every synthetic key");
  for (const gtsam::Key key : keys) {
    const gtsam::Symbol symbol(key);
    auto byte = AgentSymbol::FromByte(symbol.chr());
    Check(byte.IsOk(), "optimized GTSAM key has a valid symbol byte");
    auto agent = catalog.AgentFor(byte.Value());
    Check(agent.IsOk() && optimized.exists(key),
          "optimized GTSAM key recovers its AgentId and frame");
  }
  Check(catalog.Format(catalog.SymbolFor(ids.front()).Value(), ANCHOR_IDX,
                       ANCHOR_IDX) == "agent0/anchor",
        "anchor formatter uses the external AgentId");
  Check(catalog.Format(catalog.SymbolFor(ids.back()).Value(), 42, ANCHOR_IDX) ==
            "agent254/42",
        "frame formatter uses the external AgentId");
}

}  // namespace

int main() {
  TestLexicalAndCatalogContracts();
  TestRealGtsamGraphSupports255Agents();
  std::cout << "AgentId and 255-agent GTSAM tests passed\n";
  return 0;
}
