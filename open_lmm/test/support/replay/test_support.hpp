#pragma once

#include "support/check.hpp"

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>
#include <string_view>

#include <unistd.h>

namespace open_lmm::test::replay {

using ::Check;

class TemporaryTree {
 public:
  explicit TemporaryTree(std::string_view prefix) {
    const auto suffix =
        std::to_string(::getpid()) + "_" +
        std::to_string(std::chrono::steady_clock::now()
                           .time_since_epoch()
                           .count());
    path_ = std::filesystem::temp_directory_path() /
            (std::string(prefix) + suffix);
    std::filesystem::create_directories(path_);
  }

  ~TemporaryTree() {
    std::error_code ignored;
    std::filesystem::remove_all(path_, ignored);
  }

  const std::filesystem::path& Path() const { return path_; }

 private:
  std::filesystem::path path_;
};

}  // namespace open_lmm::test::replay
