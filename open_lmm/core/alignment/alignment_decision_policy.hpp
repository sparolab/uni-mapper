#pragma once

#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/result.hpp>

#include <cstdint>
#include <optional>

namespace open_lmm {

enum class AlignmentFeedbackMode : uint8_t {
  kAutomatic,
  kInteractive,
  kAlwaysManual,
};

enum class HeadlessAlignmentPolicy : uint8_t {
  kKissOnly,
  kKissThenDescriptor,
  kFail,
};

enum class InteractiveAlignmentState : uint8_t {
  kPending,
  kAccepted,
  kCancelled,
};

struct InteractiveAlignmentResult {
  InteractiveAlignmentState state = InteractiveAlignmentState::kPending;
  std::optional<MapAlignmentProposal> proposal;
};

enum class AlignmentPolicyAction : uint8_t {
  kAccept,
  kRequestInteractive,
};

struct AlignmentPolicyInput {
  AgentId source_agent;
  bool source_is_anchor = false;
  AlignmentFeedbackMode feedback_mode = AlignmentFeedbackMode::kAutomatic;
  HeadlessAlignmentPolicy headless_policy =
      HeadlessAlignmentPolicy::kKissThenDescriptor;
  bool interactive_service_available = false;
  std::optional<StoredAlignment> stored_alignment;
  std::optional<MapAlignmentProposal> kiss_proposal;
  std::optional<MapAlignmentProposal> descriptor_proposal;
  InteractiveAlignmentResult interactive;
  // Supplied by the orchestrator so this policy remains deterministic and
  // contains no clock or GUI dependency.
  uint64_t accepted_at_unix_ms = 0;
};

struct AlignmentPolicyOutcome {
  AlignmentPolicyAction action = AlignmentPolicyAction::kAccept;
  std::optional<StoredAlignment> accepted;
  bool manual_only = false;
};

// Pure selection/approval policy. Proposal computation, interactive transport,
// transform-to-loop conversion, and persistence are intentionally external.
class AlignmentDecisionPolicy {
 public:
  [[nodiscard]] Result<AlignmentPolicyOutcome> Decide(
      const AlignmentPolicyInput& input) const;
};

}  // namespace open_lmm
