#pragma once

#include <cstdlib>
#include <iostream>

namespace open_lmm::test::benchmark {

inline void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

}  // namespace open_lmm::test::benchmark
