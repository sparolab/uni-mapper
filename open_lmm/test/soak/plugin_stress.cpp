#include "plugins/fixtures/plugin_fixture_interface.hpp"
#include "support/soak/owner_stress_support.hpp"
#include "support/synchronization.hpp"

#include <plugins/host/load_module.hpp>

#include <array>
#include <filesystem>
#include <fstream>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <thread>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

uint64_t MappingCount(const std::array<fs::path, 12>& fixtures) {
  std::ifstream maps("/proc/self/maps");
  if (!maps) throw std::runtime_error("/proc/self/maps is unavailable");
  uint64_t count = 0;
  for (std::string line; std::getline(maps, line);) {
    for (const auto& fixture : fixtures) {
      if (line.find(fixture.string()) != std::string::npos) {
        ++count;
        break;
      }
    }
  }
  return count;
}

uint64_t MappingCount(const fs::path& plugin) {
  std::ifstream maps("/proc/self/maps");
  if (!maps) throw std::runtime_error("/proc/self/maps is unavailable");
  uint64_t count = 0;
  for (std::string line; std::getline(maps, line);) {
    if (line.find(plugin.string()) != std::string::npos) ++count;
  }
  return count;
}

Json Owner(uint64_t mappings) {
  Json owner = soak::EmptyOwnerMetrics();
  owner["loaded_plugin_mappings"] = mappings;
  return owner;
}

soak::ProcessSeries Run(const soak::RunOptions& options,
                        const std::array<fs::path, 12>& fixtures,
                        const std::optional<fs::path>& built_in,
                        Json& report) {
  soak::ProcessSeries series;
  Require(MappingCount(fixtures) == 0,
          "synthetic fixture was mapped before the stress run");
  if (built_in)
    Require(MappingCount(*built_in) == 0,
            "built-in plugin was mapped before the stress run");
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    if (built_in) {
      auto built_in_metadata = open_lmm::inspect_plugin_v1(
          built_in->string(), "descriptor",
          {{"descriptor:kdtree-v3"}, {"scan_context"}, 1,
           {"open-lmm-3.0.0"}});
      Require(
          built_in_metadata &&
              built_in_metadata.Value().name == "scan_context" &&
              built_in_metadata.Value().capability ==
                  "descriptor:kdtree-v3" &&
              built_in_metadata.Value().build_version ==
                  "open-lmm-3.0.0" &&
              MappingCount(*built_in) == 0,
          "built-in plugin inspection or unload contract failed");
    }

    PluginFixtureCounters counters;
    open_lmm::PluginMetadata metadata;
    auto inspected = open_lmm::inspect_plugin_v1(
        fixtures[0].string(), "test", {{"test:lifecycle"}});
    Require(inspected && inspected.Value().name == "fixture" &&
                counters.creates == 0,
            "valid inspection created or rejected a plugin");
    Require(!open_lmm::inspect_plugin_v1(fixtures[0].string(), "other") &&
                !open_lmm::inspect_plugin_v1(
                    fixtures[0].string(), "test", {{"other"}}),
            "kind/capability mismatch passed inspection");

    auto loaded = open_lmm::load_plugin_v1<PluginFixture>(
        fixtures[0].string(), "test", "{}", &metadata, &counters,
        {{"test:lifecycle"}});
    Require(loaded && loaded.Value()->Value() == 42 &&
                metadata.capability == "test:lifecycle" &&
                counters.creates == 1 && MappingCount(fixtures) > 0,
            "valid plugin create/lifetime contract failed");
    auto instance = std::move(loaded).Value();

    auto cancellation = std::make_shared<open_lmm::CancellationToken>();
    open_lmm::test::ManualResetEvent entered;
    std::promise<void> release_promise;
    auto release = release_promise.get_future().share();
    bool cancellation_observed = false;
    std::thread plugin_algorithm([&] {
      cancellation_observed = instance->RunCancelable(
          cancellation, [&] { entered.Signal(); }, release);
    });
    entered.Wait("plugin cancellation safe point");
    cancellation->Request();
    release_promise.set_value();
    plugin_algorithm.join();
    Require(cancellation_observed && MappingCount(fixtures) > 0,
            "loaded plugin algorithm did not observe cancellation");

    instance.reset();
    Require(counters.destroys == 1 && MappingCount(fixtures) == 0,
            "plugin object was not destroyed before DSO unload");

    for (std::size_t index : {1U, 2U, 3U, 4U, 5U, 6U, 9U, 10U, 11U}) {
      Require(!open_lmm::load_plugin_v1<PluginFixture>(
                  fixtures[index].string(), "test", "{}"),
              "invalid plugin fixture unexpectedly loaded");
    }
    for (std::size_t index : {7U, 8U}) {
      auto empty = open_lmm::inspect_plugin_v1(
          fixtures[index].string(), "test", {{std::string_view{}}});
      Require(empty && empty.Value().capability.empty() &&
                  !open_lmm::inspect_plugin_v1(
                      fixtures[index].string(), "test", {{"required"}}),
              "empty capability normalization contract failed");
    }
    const uint64_t mappings = MappingCount(fixtures);
    Require(mappings == 0, "failed plugin candidate retained a DSO mapping");
    const auto process = soak::SampleProcessMetrics();
    soak::AppendOwnerSample(report, iteration, "plugin_owner_idle", process,
                            Owner(mappings));
    soak::AddProcessPoint(series, iteration, process);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 13)
      throw std::invalid_argument(
          "twelve synthetic fixtures are required");
    std::array<fs::path, 12> fixtures;
    for (std::size_t index = 0; index < fixtures.size(); ++index) {
      fixtures[index] = fs::canonical(argv[index + 1]);
    }
    std::optional<fs::path> built_in;
    int option_begin = 13;
    if (argc > 13 && std::string_view(argv[13]).rfind("--", 0) != 0) {
      built_in = fs::canonical(argv[13]);
      option_begin = 14;
    }
    const auto options = soak::ParseRunOptions(argc, argv, option_begin);
    Json report = soak::InitialOwnerReport(
        options, "plugin-failure-lifetime", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, fixtures, built_in, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "plugin_host"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "plugin lifetime stress=" << (passed ? "PASS" : "FAIL")
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
