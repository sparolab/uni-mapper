#pragma once
#include <open_lmm/common/agent_id.hpp>
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
    kAgentExcluded,
  };

  enum class Severity : uint8_t { kRecoverable, kFatalRuntime };

  struct Context {
    std::optional<uint64_t> runtime_revision;
    std::string stage;
    std::string node;
    std::optional<AgentId> agent;
    std::string plugin;
    std::string config;
    std::string json_pointer;
    std::string expected;
    std::string actual;
    std::optional<uint32_t> schema_version;
  };

  Code        code;
  std::string message;
  Severity severity = Severity::kRecoverable;
  Context context;

  [[nodiscard]] const std::string& Message() const { return message; }

  Error& MarkFatalRuntime() & {
    severity = Severity::kFatalRuntime;
    return *this;
  }
  Error&& MarkFatalRuntime() && {
    severity = Severity::kFatalRuntime;
    return std::move(*this);
  }
  Error& WithRuntimeRevision(uint64_t revision) & {
    context.runtime_revision = revision;
    return *this;
  }
  Error&& WithRuntimeRevision(uint64_t revision) && {
    context.runtime_revision = revision;
    return std::move(*this);
  }
  Error& WithExecution(std::string stage_name, std::string node_name = {},
                       std::optional<AgentId> agent_id = std::nullopt) & {
    context.stage = std::move(stage_name);
    context.node = std::move(node_name);
    context.agent = agent_id;
    return *this;
  }
  Error&& WithExecution(std::string stage_name, std::string node_name = {},
                        std::optional<AgentId> agent_id = std::nullopt) && {
    context.stage = std::move(stage_name);
    context.node = std::move(node_name);
    context.agent = agent_id;
    return std::move(*this);
  }
  Error& WithPlugin(std::string plugin_name) & {
    context.plugin = std::move(plugin_name);
    return *this;
  }
  Error&& WithPlugin(std::string plugin_name) && {
    context.plugin = std::move(plugin_name);
    return std::move(*this);
  }
  Error& WithConfig(std::string config_name) & {
    context.config = std::move(config_name);
    return *this;
  }
  Error&& WithConfig(std::string config_name) && {
    context.config = std::move(config_name);
    return std::move(*this);
  }
  Error& WithValidation(std::string pointer, std::string expected_value,
                        std::string actual_value, uint32_t version) & {
    context.json_pointer = std::move(pointer);
    context.expected = std::move(expected_value);
    context.actual = std::move(actual_value);
    context.schema_version = version;
    return *this;
  }
  Error&& WithValidation(std::string pointer, std::string expected_value,
                         std::string actual_value, uint32_t version) && {
    context.json_pointer = std::move(pointer);
    context.expected = std::move(expected_value);
    context.actual = std::move(actual_value);
    context.schema_version = version;
    return std::move(*this);
  }

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
  static Error AgentExcluded(std::string_view detail) {
    return {Code::kAgentExcluded,
            "Agent excluded: " + std::string(detail)};
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
