#include <open_lmm/core/alignment/alignment_decision_policy.hpp>

#include <cstdlib>
#include <iostream>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

MapAlignmentProposal Proposal(AlignmentMethod method) {
  MapAlignmentProposal proposal;
  proposal.target_agent = Id("A");
  proposal.source_agent = Id("B");
  proposal.method = method;
  proposal.target_T_source.translation().x() =
      method == AlignmentMethod::kKissMatcher ? 1.0 : 2.0;
  return proposal;
}

AlignmentPolicyInput Follower() {
  AlignmentPolicyInput input;
  input.source_agent = Id("B");
  input.accepted_at_unix_ms = 42;
  return input;
}

void TestAnchorAndStoredPrecedence() {
  AlignmentDecisionPolicy policy;
  auto anchor = Follower();
  anchor.source_is_anchor = true;
  anchor.headless_policy = HeadlessAlignmentPolicy::kFail;
  const auto anchor_result = policy.Decide(anchor);
  Check(anchor_result && anchor_result.Value().accepted &&
            anchor_result.Value().accepted->proposal.source_agent == Id("B") &&
            anchor_result.Value().accepted->approval ==
                AlignmentApproval::kAutomatic,
        "anchor deterministically accepts identity without a proposer");

  auto stored = Follower();
  stored.feedback_mode = AlignmentFeedbackMode::kAlwaysManual;
  stored.headless_policy = HeadlessAlignmentPolicy::kFail;
  stored.stored_alignment =
      StoredAlignment{Proposal(AlignmentMethod::kDescriptor),
                      AlignmentApproval::kUser, 7};
  const auto stored_result = policy.Decide(stored);
  Check(stored_result && stored_result.Value().accepted &&
            stored_result.Value().accepted->accepted_at_unix_ms == 7 &&
            stored_result.Value().accepted->approval == AlignmentApproval::kUser,
        "stored approval bypasses interactive and headless selection unchanged");

  stored.stored_alignment->proposal.source_agent = Id("C");
  Check(!policy.Decide(stored), "stored approval validates source ownership");
}

void TestInteractivePolicy() {
  AlignmentDecisionPolicy policy;
  auto input = Follower();
  input.feedback_mode = AlignmentFeedbackMode::kInteractive;
  Check(!policy.Decide(input),
        "interactive mode fails without an interaction service");
  input.interactive_service_available = true;
  const auto request = policy.Decide(input);
  Check(request && request.Value().action ==
                       AlignmentPolicyAction::kRequestInteractive &&
            !request.Value().manual_only,
        "interactive mode requests a transport-independent interaction");

  input.interactive = {InteractiveAlignmentState::kAccepted,
                       Proposal(AlignmentMethod::kDescriptor)};
  const auto accepted = policy.Decide(input);
  Check(accepted && accepted.Value().accepted &&
            accepted.Value().accepted->approval == AlignmentApproval::kUser &&
            accepted.Value().accepted->accepted_at_unix_ms == 42,
        "interactive selection owns user approval metadata");

  input.feedback_mode = AlignmentFeedbackMode::kAlwaysManual;
  input.interactive = {};
  const auto manual_request = policy.Decide(input);
  Check(manual_request &&
            manual_request.Value().action ==
                AlignmentPolicyAction::kRequestInteractive &&
            manual_request.Value().manual_only,
        "always-manual request carries a transport-neutral manual-only intent");
  input.interactive = {InteractiveAlignmentState::kAccepted,
                       Proposal(AlignmentMethod::kDescriptor)};
  Check(!policy.Decide(input),
        "always-manual mode rejects a non-manual interaction result");
  input.interactive = {InteractiveAlignmentState::kAccepted,
                       Proposal(AlignmentMethod::kManual)};
  Check(policy.Decide(input).IsOk(),
        "always-manual mode accepts an explicit manual transform");
  input.interactive = {InteractiveAlignmentState::kCancelled, std::nullopt};
  const auto cancelled = policy.Decide(input);
  Check(!cancelled && cancelled.GetError().code == Error::Code::kCancelled,
        "interactive cancellation remains distinct from registration failure");
}

void TestAutomaticPolicies() {
  AlignmentDecisionPolicy policy;
  auto input = Follower();
  input.headless_policy = HeadlessAlignmentPolicy::kFail;
  Check(!policy.Decide(input), "fail policy never auto-approves a follower");

  input.headless_policy = HeadlessAlignmentPolicy::kKissOnly;
  Check(!policy.Decide(input), "kiss-only requires a KISS proposal");
  input.kiss_proposal = Proposal(AlignmentMethod::kKissMatcher);
  auto kiss = policy.Decide(input);
  Check(kiss && kiss.Value().accepted &&
            kiss.Value().accepted->approval == AlignmentApproval::kAutomatic,
        "kiss-only accepts KISS with automatic approval");

  input.headless_policy = HeadlessAlignmentPolicy::kKissThenDescriptor;
  input.kiss_proposal.reset();
  input.descriptor_proposal = Proposal(AlignmentMethod::kDescriptor);
  auto descriptor = policy.Decide(input);
  Check(descriptor && descriptor.Value().accepted &&
            descriptor.Value().accepted->proposal.method ==
                AlignmentMethod::kDescriptor,
        "fallback policy accepts Descriptor when KISS is absent");

  input.kiss_proposal = Proposal(AlignmentMethod::kKissMatcher);
  Check(policy.Decide(input).Value().accepted->proposal.method ==
            AlignmentMethod::kKissMatcher,
        "fallback policy deterministically prefers KISS");

  input.headless_policy = HeadlessAlignmentPolicy::kKissThenDescriptor;
  Check(policy.Decide(input).Value().accepted->proposal.method ==
            AlignmentMethod::kKissMatcher,
        "fallback mode preserves deterministic proposal preference");
  input.kiss_proposal->target_T_source.linear()(0, 0) = 2.0;
  Check(!policy.Decide(input), "automatic proposals must be rigid transforms");
}

}  // namespace

int main() {
  TestAnchorAndStoredPrecedence();
  TestInteractivePolicy();
  TestAutomaticPolicies();
  std::cout << "alignment decision policy tests passed\n";
  return EXIT_SUCCESS;
}
