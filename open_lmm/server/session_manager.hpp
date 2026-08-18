#pragma once

#include <memory>
#include <mutex>
#include <functional>

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/session_state.hpp>

namespace open_lmm {

// Owns the single externally visible committed session pointer. Algorithms
// work on SessionTransaction snapshots; only this component may replace the
// active revision.
class SessionManager {
 public:
  SessionManager() = default;
  explicit SessionManager(std::shared_ptr<const SessionState> initial);

  void Initialize(std::shared_ptr<const SessionState> initial);
  [[nodiscard]] std::shared_ptr<const SessionState> Snapshot() const;
  [[nodiscard]] bool Matches(
      const std::shared_ptr<const SessionState>& expected) const;
  Result<void> Commit(const std::shared_ptr<const SessionState>& expected,
                      std::shared_ptr<const SessionState> candidate);
  Result<void> CommitWithBarrier(
      const std::shared_ptr<const SessionState>& expected,
      std::shared_ptr<const SessionState> candidate,
      const std::function<Result<void>()>& commit_side_effects);

 private:
  mutable std::mutex mutex_;
  std::shared_ptr<const SessionState> committed_;
};

}  // namespace open_lmm
