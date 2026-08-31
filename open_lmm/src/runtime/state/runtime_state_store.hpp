#pragma once

#include <memory>
#include <mutex>
#include <functional>

#include <open_lmm/common/result.hpp>
#include <runtime/state/runtime_state.hpp>

namespace open_lmm {

// Owns the single externally visible committed runtime pointer. Algorithms
// work on RuntimeTransaction snapshots; only this component may replace the
// active revision.
class RuntimeStateStore {
 public:
  RuntimeStateStore() = default;
  explicit RuntimeStateStore(std::shared_ptr<const RuntimeState> initial);

  void Initialize(std::shared_ptr<const RuntimeState> initial);
  [[nodiscard]] std::shared_ptr<const RuntimeState> Snapshot() const;
  [[nodiscard]] bool Matches(
      const std::shared_ptr<const RuntimeState>& expected) const;
  Result<void> Commit(const std::shared_ptr<const RuntimeState>& expected,
                      std::shared_ptr<const RuntimeState> candidate);
  Result<void> CommitWithBarrier(
      const std::shared_ptr<const RuntimeState>& expected,
      std::shared_ptr<const RuntimeState> candidate,
      const std::function<Result<void>()>& commit_side_effects);

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const RuntimeState> committed_;
};

}  // namespace open_lmm
