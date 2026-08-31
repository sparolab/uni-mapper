#include "benchmark_statistics.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace open_lmm::test::benchmark {
namespace {

double MedianSortedIntegers(const std::vector<uint64_t>& values) {
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 != 0) return static_cast<double>(values[middle]);
  return static_cast<double>(values[middle - 1]) / 2.0 +
         static_cast<double>(values[middle]) / 2.0;
}

double MedianDoubles(std::vector<double> values) {
  std::sort(values.begin(), values.end());
  const std::size_t middle = values.size() / 2;
  if (values.size() % 2 != 0) return values[middle];
  return values[middle - 1] / 2.0 + values[middle] / 2.0;
}

}  // namespace

IntegerStatistics SummarizeIntegers(
    const std::vector<uint64_t>& samples) {
  if (samples.empty() || samples.size() > 100'000) {
    throw std::invalid_argument(
        "benchmark statistics require 1..100000 samples");
  }

  std::vector<uint64_t> ordered = samples;
  std::sort(ordered.begin(), ordered.end());
  const double median = MedianSortedIntegers(ordered);
  std::vector<double> deviations;
  deviations.reserve(samples.size());
  for (const uint64_t value : samples) {
    deviations.push_back(
        std::abs(static_cast<double>(value) - median));
  }

  std::optional<uint64_t> p95;
  if (ordered.size() >= 5) {
    const std::size_t rank =
        static_cast<std::size_t>(std::ceil(0.95 * ordered.size()));
    p95 = ordered[rank - 1];
  }
  IntegerStatistics summary;
  summary.sample_count = samples.size();
  summary.median = median;
  summary.p95 = p95;
  summary.median_absolute_deviation = MedianDoubles(deviations);
  summary.minimum = ordered.front();
  summary.maximum = ordered.back();
  return summary;
}

}  // namespace open_lmm::test::benchmark
