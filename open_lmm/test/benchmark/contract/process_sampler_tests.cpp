#include "support/benchmark/process_window_sampler.hpp"

#include <atomic>
#include <chrono>
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
  using namespace open_lmm::test::benchmark;
  bool rejected = false;
  try {
    ProcessWindowSampler invalid(std::chrono::nanoseconds::zero());
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "non-positive sample interval is rejected");

  const auto direct = SampleProcessObservation();
#if defined(__linux__)
  Check(direct.monotonic_time_ns > 0 && direct.rss_bytes &&
            direct.process_hwm_bytes && direct.cpu_time_ns &&
            direct.io.rchar && direct.io.wchar && direct.io.syscr &&
            direct.io.syscw && direct.io.read_bytes &&
            direct.io.write_bytes && direct.io.cancelled_write_bytes,
        "Linux process observation exposes memory CPU and I/O owners");
#endif

  ProcessWindowSampler sampler(std::chrono::milliseconds(1));
  sampler.Start();
  std::atomic<uint64_t> accumulator{0};
  const auto deadline =
      std::chrono::steady_clock::now() + std::chrono::milliseconds(3);
  while (std::chrono::steady_clock::now() < deadline) {
    accumulator.fetch_add(1, std::memory_order_relaxed);
  }
  const auto summary = sampler.Stop();
  Check(accumulator.load(std::memory_order_relaxed) > 0 &&
            !sampler.Running() && summary.sample_count >= 2 &&
            summary.wall_time_ns > 0 &&
            summary.sample_interval_ns == 1'000'000 &&
            !summary.memory_confidence.empty(),
        "target window joins sampler and returns closed timing metadata");
#if defined(__linux__)
  Check(summary.cpu_time_ns && summary.rss_start_bytes &&
            summary.rss_end_bytes && summary.sampled_peak_rss_bytes &&
            summary.process_hwm_bytes && summary.target_peak_delta_bytes &&
            summary.retained_rss_delta_bytes && summary.io.rchar &&
            summary.io.wchar && summary.io.syscr && summary.io.syscw &&
            summary.io.read_bytes && summary.io.write_bytes &&
            summary.io.cancelled_write_bytes && summary.unavailable.empty(),
        "Linux target window reports every required process metric");
#endif
  std::cout << "benchmark process sampler tests passed\n";
}
