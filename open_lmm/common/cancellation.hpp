#pragma once

#include <atomic>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>

namespace open_lmm {

class CancellationToken {
 public:
  void Request() noexcept { requested_.store(true, std::memory_order_release); }
  [[nodiscard]] bool IsCancellationRequested() const noexcept {
    return requested_.load(std::memory_order_acquire);
  }

 private:
  std::atomic<bool> requested_{false};
};

class CancellationException final : public std::runtime_error {
 public:
  explicit CancellationException(std::string detail)
      : std::runtime_error(std::move(detail)) {}
};

inline void ThrowIfCancellationRequested(
    const std::shared_ptr<CancellationToken>& token, std::string_view detail) {
  if (token && token->IsCancellationRequested()) {
    throw CancellationException(std::string(detail));
  }
}

}  // namespace open_lmm
