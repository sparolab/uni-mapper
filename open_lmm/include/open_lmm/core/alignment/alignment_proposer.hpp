#pragma once

#include <functional>
#include <optional>

#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/alignment_types.hpp>

namespace open_lmm {

struct AlignmentProposalRequest {
  AgentId target_agent;
  AgentId source_agent;
};

// Pure proposal boundary. The algorithm callback receives identities only;
// interactive approval and AlignmentFeedbackBroker access belong to the
// decision policy that consumes the proposal.
class AlignmentProposer {
 public:
  using ProposalAlgorithm = std::function<
      Result<std::optional<MapAlignmentProposal>>(
          const AlignmentProposalRequest&)>;

  explicit AlignmentProposer(ProposalAlgorithm algorithm);

  Result<std::optional<MapAlignmentProposal>> Propose(
      const AlgorithmExecutionContext& context,
      const AlignmentProposalRequest& request) const;

 private:
  ProposalAlgorithm algorithm_;
};

}  // namespace open_lmm
