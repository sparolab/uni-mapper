#pragma once
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <variant>

namespace open_lmm {

struct Error {
  enum class Code : uint8_t {
    kFileNotFound,
    kParseError,
    kInvalidArgument,
    kPluginLoadFailed,
    kRegistrationFailed,
    kOptimizationFailed,
    kIoError,
    kCancelled,
  };

  Code        code;
  std::string message;

  [[nodiscard]] const std::string& Message() const { return message; }

  static Error FileNotFound(std::string_view path) {
    return {Code::kFileNotFound, "File not found: " + std::string(path)};
  }
  static Error InvalidArgument(std::string_view detail) {
    return {Code::kInvalidArgument, std::string(detail)};
  }
  static Error ParseError(std::string_view detail) {
    return {Code::kParseError, "Parse error: " + std::string(detail)};
  }
  static Error PluginLoadFailed(std::string_view detail) {
    return {Code::kPluginLoadFailed, "Plugin load failed: " + std::string(detail)};
  }
  static Error RegistrationFailed(std::string_view detail) {
    return {Code::kRegistrationFailed,
            "Registration failed: " + std::string(detail)};
  }
  static Error OptimizationFailed(std::string_view detail) {
    return {Code::kOptimizationFailed,
            "Optimization failed: " + std::string(detail)};
  }
  static Error IoError(std::string_view detail) {
    return {Code::kIoError, "I/O error: " + std::string(detail)};
  }
  static Error Cancelled(std::string_view detail) {
    return {Code::kCancelled, "Cancelled: " + std::string(detail)};
  }
};

template <typename T>
class Result {
 public:
  static auto Ok(T value)      -> Result { return Result(std::move(value)); }
  static auto Failure(Error e) -> Result { return Result(std::move(e)); }

  [[nodiscard]] bool IsOk() const { return std::holds_alternative<T>(data_); }
  explicit operator bool() const  { return IsOk(); }

  auto Value() &&       -> T               { return std::get<T>(std::move(data_)); }
  auto Value() const&   -> const T&        { return std::get<T>(data_); }
  auto GetError() const -> const Error&    { return std::get<Error>(data_); }

  template <typename F>
  auto AndThen(F&& f) && -> std::invoke_result_t<F, T> {
    if (IsOk()) return std::forward<F>(f)(std::move(*this).Value());
    using Ret = std::invoke_result_t<F, T>;
    return Ret::Failure(GetError());
  }

 private:
  std::variant<T, Error> data_;
  explicit Result(T v)    : data_(std::move(v)) {}
  explicit Result(Error e): data_(std::move(e)) {}
};

template <>
class Result<void> {
 public:
  static auto Ok()             -> Result { return Result(std::nullopt); }
  static auto Failure(Error e) -> Result { return Result(std::move(e)); }

  [[nodiscard]] bool IsOk() const  { return !error_.has_value(); }
  explicit operator bool() const   { return IsOk(); }
  auto GetError() const -> const Error& { return *error_; }

 private:
  std::optional<Error> error_;
  explicit Result(std::nullopt_t)  : error_(std::nullopt) {}
  explicit Result(Error e)         : error_(std::move(e)) {}
};

}  // namespace open_lmm
