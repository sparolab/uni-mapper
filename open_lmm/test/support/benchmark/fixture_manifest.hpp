#pragma once

#include "benchmark_report.hpp"

#include <nlohmann/json_fwd.hpp>
#include <filesystem>
#include <string>

namespace open_lmm::test::benchmark {

ValidationResult ValidateFixtureManifest(const nlohmann::json& manifest);
std::string BenchmarkConfigFingerprint(const nlohmann::json& manifest);
std::string BenchmarkPairFingerprint(
    const std::filesystem::path& fixture_root,
    const nlohmann::json& manifest);
nlohmann::json BenchmarkPluginIds(const nlohmann::json& manifest);

}  // namespace open_lmm::test::benchmark
