#pragma once

#include "benchmark_report.hpp"

#include <cstdint>
#include <filesystem>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace open_lmm::test::benchmark {

struct PairParityEvidence {
  uint64_t iteration = 0;
  bool passed = false;
  std::filesystem::path log_path;
};

nlohmann::json BuildMapUpdatePairReport(
    const std::filesystem::path& sequential_bundle,
    const std::filesystem::path& parallel_bundle,
    const std::filesystem::path& sequential_owner_bundle,
    const std::filesystem::path& parallel_owner_bundle,
    const std::vector<PairParityEvidence>& parity);

ValidationResult ValidateMapUpdatePairReport(const nlohmann::json& report);

}  // namespace open_lmm::test::benchmark
