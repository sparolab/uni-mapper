#pragma once

#include <functional>
#include <thread>
#include <utility>

namespace open_lmm {

using ThreadTask = std::function<void()>;
using ThreadLauncher = std::function<std::thread(ThreadTask)>;

inline ThreadLauncher DefaultThreadLauncher() {
  return [](ThreadTask task) { return std::thread(std::move(task)); };
}

}  // namespace open_lmm
