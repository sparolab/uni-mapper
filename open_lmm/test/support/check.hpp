#pragma once

#include <cstdlib>
#include <iostream>
#include <string_view>

inline void Check(bool condition, std::string_view message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(EXIT_FAILURE);
}
