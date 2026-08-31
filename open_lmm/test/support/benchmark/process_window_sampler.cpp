#include "process_window_sampler.hpp"

#include <algorithm>
#include <fstream>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string_view>

#include <time.h>

namespace open_lmm::test::benchmark {
namespace {

std::optional<uint64_t> StatusBytes(std::string_view key) {
  std::ifstream input("/proc/self/status");
  for (std::string line; std::getline(input, line);) {
    if (!line.starts_with(key)) continue;
    std::istringstream fields(line.substr(key.size()));
    uint64_t kibibytes = 0;
    std::string unit;
    if (!(fields >> kibibytes >> unit) || unit != "kB" ||
        kibibytes > std::numeric_limits<uint64_t>::max() / 1024) {
      return std::nullopt;
    }
    return kibibytes * 1024;
  }
  return std::nullopt;
}

std::optional<uint64_t> ClockNanoseconds(clockid_t clock) {
  timespec value{};
  if (clock_gettime(clock, &value) != 0 || value.tv_sec < 0 ||
      value.tv_nsec < 0) {
    return std::nullopt;
  }
  constexpr uint64_t kNanosecondsPerSecond = 1'000'000'000;
  const auto seconds = static_cast<uint64_t>(value.tv_sec);
  const auto nanoseconds = static_cast<uint64_t>(value.tv_nsec);
  if (seconds >
      (std::numeric_limits<uint64_t>::max() - nanoseconds) /
          kNanosecondsPerSecond) {
    return std::nullopt;
  }
  return seconds * kNanosecondsPerSecond + nanoseconds;
}

ProcessIoCounters ReadIoCounters() {
  ProcessIoCounters counters;
  std::ifstream input("/proc/self/io");
  for (std::string line; std::getline(input, line);) {
    const auto separator = line.find(':');
    if (separator == std::string::npos) continue;
    std::istringstream value(line.substr(separator + 1));
    uint64_t parsed = 0;
    if (!(value >> parsed)) continue;
    const auto key = std::string_view(line).substr(0, separator);
    if (key == "rchar") counters.rchar = parsed;
    else if (key == "wchar") counters.wchar = parsed;
    else if (key == "syscr") counters.syscr = parsed;
    else if (key == "syscw") counters.syscw = parsed;
    else if (key == "read_bytes") counters.read_bytes = parsed;
    else if (key == "write_bytes") counters.write_bytes = parsed;
    else if (key == "cancelled_write_bytes") {
      counters.cancelled_write_bytes = parsed;
    }
  }
  return counters;
}

std::optional<uint64_t> Delta(const std::optional<uint64_t>& start,
                              const std::optional<uint64_t>& end) {
  if (!start || !end || *end < *start) return std::nullopt;
  return *end - *start;
}

void AddUnavailable(std::set<std::string>& unavailable,
                    const std::optional<uint64_t>& value,
                    const char* name) {
  if (!value) unavailable.emplace(name);
}

}  // namespace

ProcessObservation SampleProcessObservation() {
  ProcessObservation observation;
#if defined(__linux__)
  observation.monotonic_time_ns = ClockNanoseconds(CLOCK_MONOTONIC).value_or(0);
  observation.rss_bytes = StatusBytes("VmRSS:");
  observation.process_hwm_bytes = StatusBytes("VmHWM:");
  observation.cpu_time_ns = ClockNanoseconds(CLOCK_PROCESS_CPUTIME_ID);
  observation.io = ReadIoCounters();
  std::set<std::string> unavailable;
  AddUnavailable(unavailable, observation.rss_bytes, "rss_bytes");
  AddUnavailable(unavailable, observation.process_hwm_bytes,
                 "process_hwm_bytes");
  AddUnavailable(unavailable, observation.cpu_time_ns, "cpu_time_ns");
  AddUnavailable(unavailable, observation.io.rchar, "rchar");
  AddUnavailable(unavailable, observation.io.wchar, "wchar");
  AddUnavailable(unavailable, observation.io.syscr, "syscr");
  AddUnavailable(unavailable, observation.io.syscw, "syscw");
  AddUnavailable(unavailable, observation.io.read_bytes, "read_bytes");
  AddUnavailable(unavailable, observation.io.write_bytes, "write_bytes");
  AddUnavailable(unavailable, observation.io.cancelled_write_bytes,
                 "cancelled_write_bytes");
  observation.unavailable.assign(unavailable.begin(), unavailable.end());
#else
  observation.unavailable = {"rss_bytes", "process_hwm_bytes",
                             "cpu_time_ns", "rchar", "wchar", "syscr",
                             "syscw", "read_bytes", "write_bytes",
                             "cancelled_write_bytes"};
#endif
  return observation;
}

ProcessWindowSampler::ProcessWindowSampler(std::chrono::nanoseconds interval)
    : interval_(interval) {
  if (interval_ <= std::chrono::nanoseconds::zero()) {
    throw std::invalid_argument("process sampler interval must be positive");
  }
}

ProcessWindowSampler::~ProcessWindowSampler() {
  if (Running()) static_cast<void>(Stop());
}

void ProcessWindowSampler::Start() {
  std::lock_guard lock(mutex_);
  if (running_) throw std::logic_error("process sampler is already running");
  observations_.clear();
  observations_.push_back(SampleProcessObservation());
  stopping_ = false;
  running_ = true;
  worker_ = std::thread([this] { SampleLoop(); });
}

ProcessWindowSummary ProcessWindowSampler::Stop() {
  {
    std::lock_guard lock(mutex_);
    if (!running_) throw std::logic_error("process sampler is not running");
    stopping_ = true;
  }
  stop_requested_.notify_all();
  worker_.join();
  observations_.push_back(SampleProcessObservation());

  ProcessWindowSummary summary;
  summary.sample_count = observations_.size();
  summary.sample_interval_ns = static_cast<uint64_t>(interval_.count());
  const auto& start = observations_.front();
  const auto& end = observations_.back();
  if (end.monotonic_time_ns >= start.monotonic_time_ns) {
    summary.wall_time_ns = end.monotonic_time_ns - start.monotonic_time_ns;
  }
  summary.cpu_time_ns = Delta(start.cpu_time_ns, end.cpu_time_ns);
  summary.rss_start_bytes = start.rss_bytes;
  summary.rss_end_bytes = end.rss_bytes;
  for (const auto& observation : observations_) {
    if (observation.rss_bytes &&
        (!summary.sampled_peak_rss_bytes ||
         *observation.rss_bytes > *summary.sampled_peak_rss_bytes)) {
      summary.sampled_peak_rss_bytes = observation.rss_bytes;
    }
    if (observation.process_hwm_bytes &&
        (!summary.process_hwm_bytes ||
         *observation.process_hwm_bytes > *summary.process_hwm_bytes)) {
      summary.process_hwm_bytes = observation.process_hwm_bytes;
    }
  }
  if (summary.rss_start_bytes && summary.sampled_peak_rss_bytes) {
    summary.target_peak_delta_bytes =
        *summary.sampled_peak_rss_bytes > *summary.rss_start_bytes
            ? *summary.sampled_peak_rss_bytes - *summary.rss_start_bytes
            : 0;
  }
  if (summary.rss_start_bytes && summary.rss_end_bytes) {
    if (*summary.rss_end_bytes >= *summary.rss_start_bytes) {
      const uint64_t difference =
          *summary.rss_end_bytes - *summary.rss_start_bytes;
      if (difference <= static_cast<uint64_t>(
                            std::numeric_limits<int64_t>::max())) {
        summary.retained_rss_delta_bytes = static_cast<int64_t>(difference);
      }
    } else {
      const uint64_t difference =
          *summary.rss_start_bytes - *summary.rss_end_bytes;
      if (difference <= static_cast<uint64_t>(
                            std::numeric_limits<int64_t>::max())) {
        summary.retained_rss_delta_bytes = -static_cast<int64_t>(difference);
      }
    }
  }
  summary.io = {Delta(start.io.rchar, end.io.rchar),
                Delta(start.io.wchar, end.io.wchar),
                Delta(start.io.syscr, end.io.syscr),
                Delta(start.io.syscw, end.io.syscw),
                Delta(start.io.read_bytes, end.io.read_bytes),
                Delta(start.io.write_bytes, end.io.write_bytes),
                Delta(start.io.cancelled_write_bytes,
                      end.io.cancelled_write_bytes)};
  summary.memory_confidence =
      summary.wall_time_ns < 20'000'000 ? "low" : "normal";

  std::set<std::string> unavailable;
  AddUnavailable(unavailable, summary.cpu_time_ns, "cpu_time_ns");
  AddUnavailable(unavailable, summary.rss_start_bytes, "rss_start_bytes");
  AddUnavailable(unavailable, summary.rss_end_bytes, "rss_end_bytes");
  AddUnavailable(unavailable, summary.sampled_peak_rss_bytes,
                 "sampled_peak_rss_bytes");
  AddUnavailable(unavailable, summary.process_hwm_bytes,
                 "process_hwm_bytes");
  AddUnavailable(unavailable, summary.io.rchar, "rchar");
  AddUnavailable(unavailable, summary.io.wchar, "wchar");
  AddUnavailable(unavailable, summary.io.syscr, "syscr");
  AddUnavailable(unavailable, summary.io.syscw, "syscw");
  AddUnavailable(unavailable, summary.io.read_bytes, "read_bytes");
  AddUnavailable(unavailable, summary.io.write_bytes, "write_bytes");
  AddUnavailable(unavailable, summary.io.cancelled_write_bytes,
                 "cancelled_write_bytes");
  summary.unavailable.assign(unavailable.begin(), unavailable.end());
  {
    std::lock_guard lock(mutex_);
    running_ = false;
    stopping_ = false;
  }
  return summary;
}

bool ProcessWindowSampler::Running() const {
  std::lock_guard lock(mutex_);
  return running_;
}

void ProcessWindowSampler::SampleLoop() {
  std::unique_lock lock(mutex_);
  while (!stop_requested_.wait_for(lock, interval_,
                                   [this] { return stopping_; })) {
    lock.unlock();
    auto observation = SampleProcessObservation();
    lock.lock();
    observations_.push_back(std::move(observation));
  }
}

}  // namespace open_lmm::test::benchmark
