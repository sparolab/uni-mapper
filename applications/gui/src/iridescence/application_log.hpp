#pragma once

#include <cstddef>
#include <string>
#include <vector>

namespace open_lmm {

void ApplicationLogInfo(const std::string& message);
[[nodiscard]] std::vector<std::string> RecentApplicationLogs(
    std::size_t max_lines = 512);

}  // namespace open_lmm
