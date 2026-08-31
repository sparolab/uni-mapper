#include <foundation/logging/logging.hpp>

#include <algorithm>
#include <iostream>
#include <string>

int main() {
  constexpr char kMessage[] = "open_lmm runtime ring buffer test";
  open_lmm::InitializeLogging(8);
  open_lmm::LogInfo(kMessage);
  const auto lines = open_lmm::RecentRuntimeLogs(8);
  const bool found = std::any_of(
      lines.begin(), lines.end(), [kMessage](const std::string& line) {
        return line.find(kMessage) != std::string::npos;
      });
  if (!found) {
    std::cerr << "runtime log was not retained by the ring buffer\n";
    return 1;
  }
  return 0;
}
