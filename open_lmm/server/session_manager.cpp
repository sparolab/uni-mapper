#include "session_manager.hpp"

namespace open_lmm {

SessionManager::SessionManager(std::shared_ptr<const SessionState> initial)
    : committed_(std::move(initial)) {}

void SessionManager::Initialize(std::shared_ptr<const SessionState> initial) {
  std::lock_guard lock(mutex_);
  committed_ = std::move(initial);
}

std::shared_ptr<const SessionState> SessionManager::Snapshot() const {
  std::lock_guard lock(mutex_);
  return committed_;
}

bool SessionManager::Matches(
    const std::shared_ptr<const SessionState>& expected) const {
  std::lock_guard lock(mutex_);
  return committed_ && expected && committed_.get() == expected.get() &&
         committed_->revision == expected->revision;
}

Result<void> SessionManager::Commit(
    const std::shared_ptr<const SessionState>& expected,
    std::shared_ptr<const SessionState> candidate) {
  return CommitWithBarrier(expected, std::move(candidate), [] {
    return Result<void>::Ok();
  });
}

Result<void> SessionManager::CommitWithBarrier(
    const std::shared_ptr<const SessionState>& expected,
    std::shared_ptr<const SessionState> candidate,
    const std::function<Result<void>()>& commit_side_effects) {
  std::lock_guard lock(mutex_);
  if (!committed_ || !expected || committed_.get() != expected.get() ||
      committed_->revision != expected->revision) {
    return Result<void>::Failure(
        Error::InvalidArgument("session transaction revision conflict"));
  }
  if (!candidate || candidate->revision != expected->revision + 1) {
    return Result<void>::Failure(
        Error::InvalidArgument("session transaction candidate revision is invalid"));
  }
  auto side_effects = commit_side_effects();
  if (!side_effects) return side_effects;
  committed_ = std::move(candidate);
  return Result<void>::Ok();
}

}  // namespace open_lmm
