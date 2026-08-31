#pragma once

#include <cstddef>
#include <deque>
#include <utility>

#include <open_lmm/common/descriptor_index.hpp>

namespace open_lmm {

// Keeps recent descriptors out of the searchable index until they satisfy the
// configured intra-loop frame gap. Descriptors are only deferred, never
// discarded: Flush() publishes the remaining tail before the descriptor store
// is committed.
class IntraLoopTemporalGate {
 public:
  explicit IntraLoopTemporalGate(std::size_t minimum_frame_gap)
      : minimum_frame_gap_(minimum_frame_gap) {}

  void Defer(std::size_t frame_index, DescriptorArtifact descriptor) {
    pending_.push_back(
        PendingDescriptor{frame_index, std::move(descriptor)});
  }

  template <typename Consumer>
  void PromoteEligible(std::size_t current_frame, Consumer&& consume) {
    while (!pending_.empty() &&
           current_frame >= pending_.front().frame_index &&
           current_frame - pending_.front().frame_index >=
               minimum_frame_gap_) {
      consume(pending_.front().frame_index,
              std::move(pending_.front().descriptor));
      pending_.pop_front();
    }
  }

  template <typename Consumer>
  void Flush(Consumer&& consume) {
    while (!pending_.empty()) {
      consume(pending_.front().frame_index,
              std::move(pending_.front().descriptor));
      pending_.pop_front();
    }
  }

  [[nodiscard]] std::size_t pending_size() const { return pending_.size(); }

 private:
  struct PendingDescriptor {
    std::size_t frame_index;
    DescriptorArtifact descriptor;
  };

  std::size_t minimum_frame_gap_ = 0;
  std::deque<PendingDescriptor> pending_;
};

}  // namespace open_lmm
