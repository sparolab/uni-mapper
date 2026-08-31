#pragma once

#include <memory>
#include <mutex>
#include <functional>

#include <open_lmm/common/result.hpp>
#include <runtime/state/runtime_state.hpp>

namespace open_lmm {

struct RuntimeCommitOutcome {
  std::shared_ptr<const Error> recovery_required;
};

struct RuntimeAuthoritySnapshot {
  std::shared_ptr<const RuntimeState> state;
  std::shared_ptr<const Error> recovery_required;
};

// Owns the single externally visible committed runtime pointer. Algorithms
// work on RuntimeTransaction snapshots; only this component may replace the
// active revision.
class RuntimeStateStore {
 public:
  RuntimeStateStore() = default;
  explicit RuntimeStateStore(std::shared_ptr<const RuntimeState> initial);

  void Initialize(std::shared_ptr<const RuntimeState> initial,
                  std::shared_ptr<const Error> recovery_required);
  [[nodiscard]] std::shared_ptr<const RuntimeState> Snapshot() const;
  [[nodiscard]] RuntimeAuthoritySnapshot AuthoritySnapshot() const;
  [[nodiscard]] bool Matches(
      const std::shared_ptr<const RuntimeState>& expected) const;
  void LatchRecoveryRequired(std::shared_ptr<const Error> error) noexcept;
  Result<void> Commit(const std::shared_ptr<const RuntimeState>& expected,
                      std::shared_ptr<const RuntimeState> candidate);
  Result<RuntimeCommitOutcome> CommitWithBarrier(
      const std::shared_ptr<const RuntimeState>& expected,
      std::shared_ptr<const RuntimeState> candidate,
      const std::function<Result<RuntimeCommitOutcome>()>& commit_side_effects);

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const RuntimeState> committed_;
  std::shared_ptr<const Error> recovery_required_;
};

}  // namespace open_lmm
