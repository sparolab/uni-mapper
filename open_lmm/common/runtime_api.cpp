#include "runtime_api.hpp"

#include <utility>

namespace open_lmm {
namespace {
thread_local std::shared_ptr<CancellationToken> current_cancellation;
}  // namespace

CancellationContextScope::CancellationContextScope(
    std::shared_ptr<CancellationToken> token)
    : previous_(std::move(current_cancellation)) {
  current_cancellation = std::move(token);
}

CancellationContextScope::~CancellationContextScope() {
  current_cancellation = std::move(previous_);
}

std::shared_ptr<CancellationToken> CurrentCancellationToken() {
  return current_cancellation;
}

ExecutionEventSubscription::ExecutionEventSubscription(
    std::function<void()> unsubscribe)
    : unsubscribe_(std::move(unsubscribe)) {}

ExecutionEventSubscription::~ExecutionEventSubscription() { Reset(); }

ExecutionEventSubscription::ExecutionEventSubscription(
    ExecutionEventSubscription&& other) noexcept
    : unsubscribe_(std::move(other.unsubscribe_)) {}

ExecutionEventSubscription& ExecutionEventSubscription::operator=(
    ExecutionEventSubscription&& other) noexcept {
  if (this != &other) {
    Reset();
    unsubscribe_ = std::move(other.unsubscribe_);
  }
  return *this;
}

void ExecutionEventSubscription::Reset() {
  if (!unsubscribe_) return;
  auto unsubscribe = std::move(unsubscribe_);
  unsubscribe();
}

}  // namespace open_lmm
