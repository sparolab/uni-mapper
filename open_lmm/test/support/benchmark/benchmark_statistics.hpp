#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace open_lmm::test::benchmark {

struct IntegerStatistics {
  std::size_t sample_count = 0;
  double median = 0.0;
  std::optional<uint64_t> p95;
  double median_absolute_deviation = 0.0;
  uint64_t minimum = 0;
  uint64_t maximum = 0;
};

// p95 is intentionally unavailable below five samples. Callers must not turn
// an undersized calibration set into a regression threshold.
IntegerStatistics SummarizeIntegers(const std::vector<uint64_t>& samples);

}  // namespace open_lmm::test::benchmark
