#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm {

enum class CancellationMode : uint8_t {
  kTokenPolling,
  kHostSafePoints,
  kNonCooperative,
};

struct CancellationCapability {
  bool cooperative = true;
  CancellationMode mode = CancellationMode::kTokenPolling;
  std::vector<std::string> non_interruptible_operations;
  bool requires_process_isolation = false;
};

struct CancellationTelemetry {
  CancellationCapability capability;
  std::optional<int64_t> cancel_requested_at_unix_ns;
  std::optional<int64_t> cancel_observed_at_unix_ns;
  std::optional<int64_t> cancel_completed_at_unix_ns;

  [[nodiscard]] bool Pending() const noexcept {
    return cancel_requested_at_unix_ns.has_value() &&
           !cancel_completed_at_unix_ns.has_value();
  }
};

class CancellationToken {
 public:
  explicit CancellationToken(CancellationCapability capability = {})
      : capability_(std::move(capability)) {}

  void Request() noexcept {
    StoreFirst(requested_at_unix_ns_);
    requested_.store(true, std::memory_order_release);
  }
  [[nodiscard]] bool IsCancellationRequested() const noexcept {
    const bool requested = requested_.load(std::memory_order_acquire);
    if (requested) StoreFirst(observed_at_unix_ns_);
    return requested;
  }
  void Complete() noexcept {
    if (requested_.load(std::memory_order_acquire)) {
      StoreFirst(completed_at_unix_ns_);
    }
  }
  [[nodiscard]] CancellationTelemetry Telemetry() const {
    CancellationTelemetry result;
    result.capability = capability_;
    result.cancel_requested_at_unix_ns = Load(requested_at_unix_ns_);
    result.cancel_observed_at_unix_ns = Load(observed_at_unix_ns_);
    result.cancel_completed_at_unix_ns = Load(completed_at_unix_ns_);
    return result;
  }

 private:
  static int64_t NowUnixNanoseconds() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::system_clock::now().time_since_epoch())
        .count();
  }
  static void StoreFirst(std::atomic<int64_t>& destination) noexcept {
    int64_t empty = 0;
    destination.compare_exchange_strong(empty, NowUnixNanoseconds(),
                                        std::memory_order_acq_rel);
  }
  static std::optional<int64_t> Load(
      const std::atomic<int64_t>& source) noexcept {
    const int64_t value = source.load(std::memory_order_acquire);
    return value == 0 ? std::nullopt : std::optional<int64_t>(value);
  }

  std::atomic<bool> requested_{false};
  mutable std::atomic<int64_t> requested_at_unix_ns_{0};
  mutable std::atomic<int64_t> observed_at_unix_ns_{0};
  mutable std::atomic<int64_t> completed_at_unix_ns_{0};
  CancellationCapability capability_;
};

class CancellationException final : public std::runtime_error {
 public:
  explicit CancellationException(std::string detail)
      : std::runtime_error(std::move(detail)) {}
};

// Carries cooperative cancellation through an existing synchronous call chain
// without changing every provider signature. The state lives in the contracts
// DSO so callers and providers in different OpenLMM DSOs share one TLS slot.
class CancellationContextScope {
 public:
  explicit CancellationContextScope(std::shared_ptr<CancellationToken> token);
  ~CancellationContextScope();
  CancellationContextScope(const CancellationContextScope&) = delete;
  CancellationContextScope& operator=(const CancellationContextScope&) = delete;

 private:
  std::shared_ptr<CancellationToken> previous_;
};

[[nodiscard]] std::shared_ptr<CancellationToken> CurrentCancellationToken();

inline void ThrowIfCancellationRequested(
    const std::shared_ptr<CancellationToken>& token, std::string_view detail) {
  if (token && token->IsCancellationRequested()) {
    throw CancellationException(std::string(detail));
  }
}

}  // namespace open_lmm
