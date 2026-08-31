#pragma once

#include "benchmark_report.hpp"

#include <filesystem>
#include <optional>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace open_lmm::test::benchmark {

// Aggregates already-materialized raw reports. Every report is validated and
// retained by path+content hash; failed repetitions are never silently
// omitted. A missing baseline yields an explicit uncalibrated bundle.
nlohmann::json AggregatePerformanceReports(
    const std::vector<std::filesystem::path>& report_paths,
    const std::optional<std::filesystem::path>& baseline_path = std::nullopt);

ValidationResult ValidatePerformanceBundle(const nlohmann::json& bundle);

}  // namespace open_lmm::test::benchmark
