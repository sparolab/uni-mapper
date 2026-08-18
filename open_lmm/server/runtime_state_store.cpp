#include "runtime_state_store.hpp"

namespace open_lmm {

RuntimeStateStore::RuntimeStateStore(std::shared_ptr<const RuntimeState> initial)
    : committed_(std::move(initial)) {}

void RuntimeStateStore::Initialize(std::shared_ptr<const RuntimeState> initial) {
  std::lock_guard lock(mutex_);
  committed_ = std::move(initial);
}

std::shared_ptr<const RuntimeState> RuntimeStateStore::Snapshot() const {
  std::lock_guard lock(mutex_);
  return committed_;
}

bool RuntimeStateStore::Matches(
    const std::shared_ptr<const RuntimeState>& expected) const {
  std::lock_guard lock(mutex_);
  return committed_ && expected && committed_.get() == expected.get() &&
         committed_->revision == expected->revision;
}

Result<void> RuntimeStateStore::Commit(
    const std::shared_ptr<const RuntimeState>& expected,
    std::shared_ptr<const RuntimeState> candidate) {
  return CommitWithBarrier(expected, std::move(candidate), [] {
    return Result<void>::Ok();
  });
}

Result<void> RuntimeStateStore::CommitWithBarrier(
    const std::shared_ptr<const RuntimeState>& expected,
    std::shared_ptr<const RuntimeState> candidate,
    const std::function<Result<void>()>& commit_side_effects) {
  std::lock_guard lock(mutex_);
  if (!committed_ || !expected || committed_.get() != expected.get() ||
      committed_->revision != expected->revision) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime transaction revision conflict"));
  }
  if (!candidate || candidate->revision != expected->revision + 1) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime transaction candidate revision is invalid"));
  }
  auto side_effects = commit_side_effects();
  if (!side_effects) return side_effects;
  committed_ = std::move(candidate);
  return Result<void>::Ok();
}

}  // namespace open_lmm
