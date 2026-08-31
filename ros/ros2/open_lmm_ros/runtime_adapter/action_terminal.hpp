#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/runtime_api.hpp>

#include <optional>

namespace open_lmm {

enum class RosActionTerminal { kSucceeded, kCanceled, kAborted };

// Runtime completion, not the ROS cancel-request flag, owns the terminal
// result. A valid committed receipt therefore wins over a late cancel.
inline RosActionTerminal ResolveRosActionTerminal(
    const Result<void>& waited, uint64_t expected_job_id,
    const std::optional<JobSnapshot>& authoritative_job) {
  if (waited) return RosActionTerminal::kSucceeded;
  if (authoritative_job && authoritative_job->id == expected_job_id) {
    switch (authoritative_job->state) {
      case JobState::kSucceeded: return RosActionTerminal::kSucceeded;
      case JobState::kCancelled: return RosActionTerminal::kCanceled;
      case JobState::kFailed: return RosActionTerminal::kAborted;
      default: break;
    }
  }
  return waited.GetError().code == Error::Code::kCancelled
             ? RosActionTerminal::kCanceled
             : RosActionTerminal::kAborted;
}

}  // namespace open_lmm
