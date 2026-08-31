#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace open_lmm::test::soak {

struct ProcessMetrics {
  std::optional<uint64_t> rss_bytes;
  std::optional<uint64_t> peak_rss_bytes;
  std::optional<uint64_t> thread_count;
  std::optional<uint64_t> fd_count;
  std::optional<uint64_t> cpu_time_ns;
  std::vector<std::string> unavailable;
};

struct MetricPoint {
  uint64_t iteration = 0;
  double value = 0.0;
};

struct SlopeAnalysis {
  std::size_t sample_count = 0;
  std::size_t warmup_samples = 0;
  double theil_sen_per_iteration = 0.0;
  double previous_quarter_median = 0.0;
  double last_quarter_median = 0.0;
  double quarter_median_delta = 0.0;
  double minimum = 0.0;
  double maximum = 0.0;
};

struct ValidationIssue {
  std::string pointer;
  std::string message;
};

struct ValidationResult {
  std::vector<ValidationIssue> issues;
  [[nodiscard]] bool Ok() const noexcept { return issues.empty(); }
  [[nodiscard]] std::string Summary() const;
};

ProcessMetrics SampleProcessMetrics();
SlopeAnalysis AnalyzeSlope(const std::vector<MetricPoint>& points,
                           std::size_t warmup_samples);
nlohmann::json ProcessMetricsJson(const ProcessMetrics& metrics);
nlohmann::json SlopeAnalysisJson(const SlopeAnalysis& analysis);
ValidationResult ValidateSoakReport(const nlohmann::json& report);
void WriteJsonExclusive(const std::filesystem::path& path,
                        const nlohmann::json& value);

}  // namespace open_lmm::test::soak
