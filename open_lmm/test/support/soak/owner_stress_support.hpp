#pragma once

#include "support/soak/soak_metrics.hpp"

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace open_lmm::test::soak {

struct RunOptions {
  uint64_t iterations = 100;
  uint64_t warmup = 10;
  uint64_t seed = 104729;
  std::string profile = "fast";
  std::string git_commit = std::string(40, '0');
  bool git_dirty = false;
  std::optional<std::filesystem::path> report;
};

struct ProcessSeries {
  std::vector<MetricPoint> rss;
  std::vector<MetricPoint> threads;
  std::vector<MetricPoint> fds;
};

RunOptions ParseRunOptions(int argc, char** argv, int first_option = 1);
nlohmann::json InitialOwnerReport(const RunOptions& options,
                                  std::string scenario,
                                  std::string sanitizer);
nlohmann::json EmptyOwnerMetrics();
void AppendOwnerSample(nlohmann::json& report, uint64_t iteration,
                       std::string checkpoint, const ProcessMetrics& process,
                       nlohmann::json owner);
void AddProcessPoint(ProcessSeries& series, uint64_t iteration,
                     const ProcessMetrics& process);
void FinishOwnerReport(const RunOptions& options, const ProcessSeries& series,
                       nlohmann::json& report);

class TemporaryDirectory {
 public:
  explicit TemporaryDirectory(std::string prefix);
  ~TemporaryDirectory();
  TemporaryDirectory(const TemporaryDirectory&) = delete;
  TemporaryDirectory& operator=(const TemporaryDirectory&) = delete;
  [[nodiscard]] const std::filesystem::path& Path() const noexcept {
    return path_;
  }

 private:
  std::filesystem::path path_;
};

}  // namespace open_lmm::test::soak
