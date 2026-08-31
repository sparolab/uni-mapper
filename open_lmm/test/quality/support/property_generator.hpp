#pragma once

#include <charconv>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>

namespace open_lmm::test::property {

inline uint64_t EnvironmentUnsigned(const char* name, uint64_t fallback) {
  const char* value = std::getenv(name);
  if (!value || *value == '\0') return fallback;
  uint64_t parsed = 0;
  const std::string_view text(value);
  const auto result = std::from_chars(text.data(), text.data() + text.size(),
                                      parsed);
  if (result.ec != std::errc{} || result.ptr != text.data() + text.size()) {
    std::cerr << "invalid " << name << ": " << text << '\n';
    std::exit(2);
  }
  return parsed;
}

inline uint64_t Seed() {
  return EnvironmentUnsigned("OPEN_LMM_PROPERTY_SEED",
                             UINT64_C(0x4f70656e4c4d4d08));
}

inline std::size_t Cases(std::size_t fallback) {
  const uint64_t value = EnvironmentUnsigned("OPEN_LMM_PROPERTY_CASES",
                                             fallback);
  if (value == 0 || value > std::numeric_limits<std::size_t>::max()) {
    std::cerr << "OPEN_LMM_PROPERTY_CASES must be a positive size_t\n";
    std::exit(2);
  }
  return static_cast<std::size_t>(value);
}

class Generator {
 public:
  explicit Generator(uint64_t seed) : state_(seed) {}

  uint64_t Next() {
    uint64_t value = (state_ += UINT64_C(0x9e3779b97f4a7c15));
    value = (value ^ (value >> 30U)) * UINT64_C(0xbf58476d1ce4e5b9);
    value = (value ^ (value >> 27U)) * UINT64_C(0x94d049bb133111eb);
    return value ^ (value >> 31U);
  }

  std::size_t Index(std::size_t upper_exclusive) {
    return static_cast<std::size_t>(Next() % upper_exclusive);
  }

  double Unit() {
    constexpr double denominator =
        static_cast<double>(UINT64_C(1) << 53U);
    return static_cast<double>(Next() >> 11U) / denominator;
  }

  double Between(double minimum, double maximum) {
    return minimum + (maximum - minimum) * Unit();
  }

 private:
  uint64_t state_;
};

inline void Fail(const char* property, uint64_t seed, std::size_t case_index,
                 std::string_view detail) {
  std::cerr << "PROPERTY FAIL: " << property << " seed=" << seed
            << " case=" << case_index << " detail=" << detail << '\n';
  std::exit(1);
}

}  // namespace open_lmm::test::property
