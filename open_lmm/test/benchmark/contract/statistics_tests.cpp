#include "support/benchmark/benchmark_statistics.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

}  // namespace

int main() {
  using open_lmm::test::benchmark::SummarizeIntegers;
  const auto odd = SummarizeIntegers({50, 10, 40, 20, 30});
  Check(odd.sample_count == 5 && odd.median == 30.0 && odd.p95 == 50 &&
            odd.median_absolute_deviation == 10.0 && odd.minimum == 10 &&
            odd.maximum == 50,
        "odd sample summary uses nearest-rank p95 and MAD");

  const auto even = SummarizeIntegers({4, 1, 3, 2});
  Check(even.median == 2.5 && !even.p95 &&
            even.median_absolute_deviation == 1.0,
        "undersized sample omits p95 and preserves exact median");

  bool rejected = false;
  try {
    static_cast<void>(SummarizeIntegers({}));
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "empty benchmark sample is rejected");
  std::cout << "benchmark statistics tests passed\n";
}
