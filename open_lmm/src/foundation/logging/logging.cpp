#include <foundation/logging/logging.hpp>

#include <algorithm>
#include <memory>
#include <mutex>

#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/spdlog.h>

namespace open_lmm {
namespace {
std::mutex logging_mutex;
std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> runtime_sink;
}  // namespace

void InitializeLogging(std::size_t runtime_buffer_size) {
  std::lock_guard lock(logging_mutex);
  if (runtime_sink) return;

  runtime_sink = std::make_shared<spdlog::sinks::ringbuffer_sink_mt>(
      std::max<std::size_t>(runtime_buffer_size, 1));
  runtime_sink->set_pattern("[%H:%M:%S.%e] [%^%l%$] %v");
  spdlog::default_logger()->sinks().push_back(runtime_sink);
}

void LogDebug(const std::string& message) {
  InitializeLogging();
  spdlog::debug("{}", message);
}

void LogInfo(const std::string& message) {
  InitializeLogging();
  spdlog::info("{}", message);
}

void LogWarning(const std::string& message) {
  InitializeLogging();
  spdlog::warn("{}", message);
}

void LogError(const std::string& message) {
  InitializeLogging();
  spdlog::error("{}", message);
}

std::vector<std::string> RecentRuntimeLogs(std::size_t max_lines) {
  InitializeLogging();
  std::shared_ptr<spdlog::sinks::ringbuffer_sink_mt> sink;
  {
    std::lock_guard lock(logging_mutex);
    sink = runtime_sink;
  }
  return sink->last_formatted(max_lines);
}

}  // namespace open_lmm
