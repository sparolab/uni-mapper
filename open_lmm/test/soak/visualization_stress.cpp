#include "support/soak/owner_stress_support.hpp"

#include <runtime/state/runtime_state.hpp>
#include <visualization/projection/visualization_projector.hpp>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

namespace {
using namespace open_lmm;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

AgentRawDataHandle MakeRaw(const AgentId& agent) {
  auto raw = std::make_shared<AgentRawData>();
  raw->agent_id = agent;
  raw->odom_poses = {Eigen::Isometry3d::Identity(),
                     Eigen::Isometry3d::Identity()};
  raw->odom_poses[1].translation().x() = 1.0;
  for (int frame = 0; frame != 2; ++frame) {
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->push_back(pcl::PointXYZI{0.0F, 0.0F, 0.0F,
                                  static_cast<float>(frame)});
    scan->push_back(pcl::PointXYZI{0.1F, 0.0F, 0.0F,
                                  static_cast<float>(frame)});
    raw->filtered_scans.push_back(std::move(scan));
  }
  return raw;
}

std::shared_ptr<RuntimeState> MakeState(uint64_t revision) {
  const AgentId agent = Id("agent");
  auto raw = MakeRaw(agent);
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data.emplace(agent, raw);
  auto payload = std::make_shared<RuntimePayload>();
  payload->database = database;
  AgentPipelineCtx context;
  context.agent.id = agent;
  context.raw_data = raw;
  payload->contexts.push_back(std::move(context));
  auto config = std::make_shared<RuntimeConfig>();
  config->root.save_voxel_size = 0.2;
  auto state = std::make_shared<RuntimeState>();
  state->revision = revision;
  state->config = config;
  state->ordered_agents = {agent};
  state->payload = payload;
  return state;
}

VisualizationSource MakeSource(const std::shared_ptr<RuntimeState>& state) {
  VisualizationSource source;
  source.revision = state->revision;
  source.preview_voxel_size_m =
      static_cast<float>(state->config->root.save_voxel_size);
  for (const auto& [agent, raw] : state->payload->database->raw_data) {
    const auto agent_id = agent;
    const auto context = std::find_if(
        state->payload->contexts.begin(), state->payload->contexts.end(),
        [agent_id](const AgentPipelineCtx& item) {
          return item.agent.id == agent_id;
        });
    source.agents.push_back(
        {agent, raw, nullptr,
         context == state->payload->contexts.end() ? nullptr
                                                   : context->loop_output});
  }
  return source;
}

Json Owner(const VisualizationProjector& projector) {
  Json owner = soak::EmptyOwnerMetrics();
  owner["visualization_cache_entries"] = projector.PointCacheEntryCount();
  owner["visualization_cache_bytes"] = projector.PointCacheBytes();
  return owner;
}

soak::ProcessSeries Run(const soak::RunOptions& options, Json& report) {
  VisualizationProjector projector;
  soak::ProcessSeries series;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    const uint64_t revision = iteration * 3 + 1;
    auto committed = MakeState(revision);
    projector.Publish(MakeSource(committed),
                      VisualizationPhase::kOptimization, false);
    for (std::size_t shape = 1; shape <= 24; ++shape) {
      auto projected = projector.Project(
          {Id("agent"), true, static_cast<float>(shape) / 1000.0F,
           shape + 1});
      Require(projected.IsOk(), "cache-shape projection failed");
    }
    Require(projector.PointCacheEntryCount() == 16 &&
                projector.PointCacheBytes() > 0,
            "point cache did not enforce its exact retained-entry cap");
    soak::AppendOwnerSample(report, iteration, "cache_bounded",
                            soak::SampleProcessMetrics(), Owner(projector));

    const AgentId second = Id("second");
    const auto second_raw = MakeRaw(second);
    projector.PublishDataLoadCandidate(revision, second, second_raw);
    auto candidate = projector.Project({second, false});
    Require(candidate && candidate.Value().revision == revision,
            "candidate presentation was not queryable");
    projector.RollbackDataLoadCandidate(revision);
    auto restored = projector.Project({Id("agent"), false});
    Require(restored && restored.Value().revision == revision &&
                restored.Value().phase == VisualizationPhase::kOptimization &&
                !projector.Project({second, false}),
            "candidate rollback did not preserve the last valid presentation");

    projector.PublishDataLoadCandidate(revision, second, second_raw);
    auto newer = MakeState(revision + 1);
    projector.Publish(MakeSource(newer), VisualizationPhase::kOptimization,
                      false);
    projector.PublishDataLoadCandidate(revision, second, second_raw);
    auto authoritative = projector.Project({Id("agent"), false});
    Require(authoritative && authoritative.Value().revision == revision + 1 &&
                !projector.Project({second, false}),
            "stale generation replaced a newer presentation");

    const std::size_t before_cancel = projector.PointCacheEntryCount();
    auto cancellation = std::make_shared<CancellationToken>();
    cancellation->Request();
    {
      CancellationContextScope scope(cancellation);
      auto cancelled = projector.Project({Id("agent"), true, 0.0007F, 99});
      Require(!cancelled &&
                  cancelled.GetError().code == Error::Code::kCancelled,
              "cancelled projection reported success");
    }
    Require(projector.PointCacheEntryCount() == before_cancel,
            "cancelled projection populated the point cache");

    projector.Clear(revision + 2, 0.2F);
    Require(projector.PointCacheEntryCount() == 0 &&
                projector.PointCacheBytes() == 0 &&
                !projector.Project({Id("agent"), false}),
            "projector clear did not return cache/presentation to baseline");
    const auto process = soak::SampleProcessMetrics();
    soak::AppendOwnerSample(report, iteration, "owner_idle", process,
                            Owner(projector));
    soak::AddProcessPoint(series, iteration, process);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = soak::ParseRunOptions(argc, argv);
    Json report = soak::InitialOwnerReport(
        options, "visualization-supersession", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "visualization_projector"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "visualization stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << options.iterations << '\n';
    return passed ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid soak request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak infrastructure failure: " << error.what() << '\n';
    return 2;
  }
}
