#include "iridescence/application_log.hpp"

#include <spdlog/sinks/ringbuffer_sink.h>
#include <spdlog/spdlog.h>

#include <memory>

namespace open_lmm {

void ApplicationLogInfo(const std::string& message) {
  spdlog::info("{}", message);
}

std::vector<std::string> RecentApplicationLogs(std::size_t max_lines) {
  const auto logger = spdlog::default_logger();
  if (!logger) return {};
  for (const auto& sink : logger->sinks()) {
    auto ring = std::dynamic_pointer_cast<
        spdlog::sinks::ringbuffer_sink_mt>(sink);
    if (ring) return ring->last_formatted(max_lines);
  }
  return {};
}

}  // namespace open_lmm
