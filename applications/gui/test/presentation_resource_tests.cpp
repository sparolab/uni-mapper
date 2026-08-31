#include "visualization_repository.hpp"

#include <open_lmm/common/result.hpp>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>

namespace {

constexpr std::size_t kPointCount = 2'000'000;
constexpr std::size_t kMaximumCommitRssGrowthKiB = 8U * 1024U;

void Require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

std::size_t ProcStatusKiB(const std::string& field) {
  std::ifstream status("/proc/self/status");
  std::string line;
  while (std::getline(status, line)) {
    if (!line.starts_with(field)) continue;
    std::istringstream value_stream(line.substr(field.size()));
    std::size_t value = 0;
    value_stream >> value;
    return value;
  }
  return 0;
}

}  // namespace

int main() {
  auto snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  snapshot->agent = open_lmm::AgentId::Parse("resource-probe").Value();
  snapshot->revision = 1;
  snapshot->points_available = true;
  snapshot->points_complete = true;
  snapshot->displayed_point_count = kPointCount;
  snapshot->source_point_count = kPointCount;
  snapshot->points.resize(kPointCount);

  const auto source_bytes =
      snapshot->points.size() * sizeof(open_lmm::VisualizationPoint);
  const auto rss_before_commit_kib = ProcStatusKiB("VmRSS:");
  const auto hwm_before_commit_kib = ProcStatusKiB("VmHWM:");

  open_lmm::VisualizationRepository repository;
  const auto update = repository.Commit(snapshot);
  const auto stored = repository.Latest(snapshot->agent);
  const auto rss_after_commit_kib = ProcStatusKiB("VmRSS:");
  const auto hwm_after_commit_kib = ProcStatusKiB("VmHWM:");
  const auto commit_hwm_growth_kib =
      hwm_after_commit_kib > hwm_before_commit_kib
          ? hwm_after_commit_kib - hwm_before_commit_kib
          : 0;

  Require(update.changed, "representative payload must commit");
  Require(stored && stored.get() != snapshot.get(),
          "repository must own an independent metadata snapshot");
  Require(stored->points.empty(),
          "repository must not copy the full point payload");
  Require(snapshot.use_count() == 1,
          "repository must not retain the source payload owner");
  Require(commit_hwm_growth_kib <= kMaximumCommitRssGrowthKiB,
          "metadata commit must not allocate another full point payload");

  std::cout << "GUI presentation resource probe passed: point_count="
            << kPointCount << " source_bytes=" << source_bytes
            << " full_point_payload_copies=0"
            << " rss_before_commit_kib=" << rss_before_commit_kib
            << " rss_after_commit_kib=" << rss_after_commit_kib
            << " hwm_before_commit_kib=" << hwm_before_commit_kib
            << " hwm_after_commit_kib=" << hwm_after_commit_kib
            << " commit_hwm_growth_kib=" << commit_hwm_growth_kib
            << " threshold_kib=" << kMaximumCommitRssGrowthKiB << '\n';
  return 0;
}
