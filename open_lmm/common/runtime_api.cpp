#include "runtime_api.hpp"

#include <utility>

namespace open_lmm {

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
