#pragma once

#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace open_lmm::test::benchmark {

struct ProcessIoCounters {
  std::optional<uint64_t> rchar;
  std::optional<uint64_t> wchar;
  std::optional<uint64_t> syscr;
  std::optional<uint64_t> syscw;
  std::optional<uint64_t> read_bytes;
  std::optional<uint64_t> write_bytes;
  std::optional<uint64_t> cancelled_write_bytes;
};

struct ProcessObservation {
  uint64_t monotonic_time_ns = 0;
  std::optional<uint64_t> rss_bytes;
  std::optional<uint64_t> process_hwm_bytes;
  std::optional<uint64_t> cpu_time_ns;
  ProcessIoCounters io;
  std::vector<std::string> unavailable;
};

struct ProcessIoDelta {
  std::optional<uint64_t> rchar;
  std::optional<uint64_t> wchar;
  std::optional<uint64_t> syscr;
  std::optional<uint64_t> syscw;
  std::optional<uint64_t> read_bytes;
  std::optional<uint64_t> write_bytes;
  std::optional<uint64_t> cancelled_write_bytes;
};

struct ProcessWindowSummary {
  uint64_t wall_time_ns = 0;
  std::optional<uint64_t> cpu_time_ns;
  std::optional<uint64_t> rss_start_bytes;
  std::optional<uint64_t> rss_end_bytes;
  std::optional<uint64_t> sampled_peak_rss_bytes;
  std::optional<uint64_t> process_hwm_bytes;
  std::optional<uint64_t> target_peak_delta_bytes;
  std::optional<int64_t> retained_rss_delta_bytes;
  ProcessIoDelta io;
  std::size_t sample_count = 0;
  uint64_t sample_interval_ns = 0;
  std::string memory_confidence;
  std::vector<std::string> unavailable;
};

ProcessObservation SampleProcessObservation();

class ProcessWindowSampler {
 public:
  explicit ProcessWindowSampler(
      std::chrono::nanoseconds interval = std::chrono::milliseconds(1));
  ~ProcessWindowSampler();
  ProcessWindowSampler(const ProcessWindowSampler&) = delete;
  ProcessWindowSampler& operator=(const ProcessWindowSampler&) = delete;

  void Start();
  [[nodiscard]] ProcessWindowSummary Stop();
  [[nodiscard]] bool Running() const;

 private:
  void SampleLoop();

  const std::chrono::nanoseconds interval_;
  mutable std::mutex mutex_;
  std::condition_variable stop_requested_;
  std::thread worker_;
  bool running_ = false;
  bool stopping_ = false;
  std::vector<ProcessObservation> observations_;
};

}  // namespace open_lmm::test::benchmark
