#pragma once

#include <open_lmm/common/result.hpp>

#include <compare>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

namespace open_lmm {

class SessionId {
 public:
  [[nodiscard]] static Result<SessionId> Parse(std::string_view value);
  [[nodiscard]] const std::string& Value() const noexcept { return value_; }
  friend bool operator==(const SessionId&, const SessionId&) = default;
  friend auto operator<=>(const SessionId&, const SessionId&) = default;

 private:
  explicit SessionId(std::string value) : value_(std::move(value)) {}
  std::string value_;
  friend class RuntimeService;
};

using JobId = uint64_t;

struct BootstrapRequest {
  std::filesystem::path config_directory;
  std::string label;
  std::optional<std::filesystem::path> output_root;
};

enum class RuntimeSessionState : uint8_t {
  kCreating,
  kReady,
  kRunning,
  kCancelling,
  kFailedRecoverable,
  kFailedFatal,
  kClosing,
  kClosed,
};

enum class CloseMode : uint8_t { kRejectIfRunning, kCancelAndWait };

}  // namespace open_lmm
