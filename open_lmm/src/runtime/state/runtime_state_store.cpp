#include "runtime_state_store.hpp"

namespace open_lmm {

RuntimeStateStore::RuntimeStateStore(std::shared_ptr<const RuntimeState> initial)
    : committed_(std::move(initial)) {}

void RuntimeStateStore::Initialize(
    std::shared_ptr<const RuntimeState> initial,
    std::shared_ptr<const Error> recovery_required) {
  std::lock_guard lock(mutex_);
  committed_ = std::move(initial);
  if (recovery_required && committed_ &&
      recovery_required->context.runtime_revision != committed_->revision) {
    auto normalized = std::make_shared<Error>(*recovery_required);
    normalized->MarkFatalRuntime().WithRuntimeRevision(committed_->revision);
    recovery_required = std::move(normalized);
  }
  recovery_required_ = std::move(recovery_required);
}

std::shared_ptr<const RuntimeState> RuntimeStateStore::Snapshot() const {
  std::lock_guard lock(mutex_);
  return committed_;
}

RuntimeAuthoritySnapshot RuntimeStateStore::AuthoritySnapshot() const {
  std::lock_guard lock(mutex_);
  return {committed_, recovery_required_};
}

bool RuntimeStateStore::Matches(
    const std::shared_ptr<const RuntimeState>& expected) const {
  std::lock_guard lock(mutex_);
  return committed_ && expected && committed_.get() == expected.get() &&
         committed_->revision == expected->revision;
}

void RuntimeStateStore::LatchRecoveryRequired(
    std::shared_ptr<const Error> error) noexcept {
  std::lock_guard lock(mutex_);
  if (committed_ && error) recovery_required_ = std::move(error);
}

Result<void> RuntimeStateStore::Commit(
    const std::shared_ptr<const RuntimeState>& expected,
    std::shared_ptr<const RuntimeState> candidate) {
  auto committed = CommitWithBarrier(expected, std::move(candidate), [] {
    return Result<RuntimeCommitOutcome>::Ok({});
  });
  return committed ? Result<void>::Ok()
                   : Result<void>::Failure(committed.GetError());
}

Result<RuntimeCommitOutcome> RuntimeStateStore::CommitWithBarrier(
    const std::shared_ptr<const RuntimeState>& expected,
    std::shared_ptr<const RuntimeState> candidate,
    const std::function<Result<RuntimeCommitOutcome>()>& commit_side_effects) {
  std::lock_guard lock(mutex_);
  if (recovery_required_) {
    return Result<RuntimeCommitOutcome>::Failure(*recovery_required_);
  }
  if (!committed_ || !expected || committed_.get() != expected.get() ||
      committed_->revision != expected->revision) {
    return Result<RuntimeCommitOutcome>::Failure(
        Error::InvalidArgument("runtime transaction revision conflict"));
  }
  if (!candidate || candidate->revision != expected->revision + 1) {
    return Result<RuntimeCommitOutcome>::Failure(
        Error::InvalidArgument("runtime transaction candidate revision is invalid"));
  }
  auto side_effects = commit_side_effects();
  if (!side_effects) return side_effects;
  committed_ = std::move(candidate);
  auto outcome = std::move(side_effects).Value();
  if (outcome.recovery_required) {
    recovery_required_ = outcome.recovery_required;
  }
  return Result<RuntimeCommitOutcome>::Ok(std::move(outcome));
}

}  // namespace open_lmm
