#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace open_lmm {

// Installs OpenLMM's process-wide logging sinks. Safe to call repeatedly.
void InitializeLogging(std::size_t runtime_buffer_size = 512);
void LogDebug(const std::string& message);
void LogInfo(const std::string& message);
void LogWarning(const std::string& message);
void LogError(const std::string& message);

// Returns the newest formatted runtime log lines retained in memory.
[[nodiscard]] std::vector<std::string> RecentRuntimeLogs(
    std::size_t max_lines = 512);

}  // namespace open_lmm
