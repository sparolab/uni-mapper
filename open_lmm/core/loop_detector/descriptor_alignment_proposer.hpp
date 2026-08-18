#pragma once

#include <open_lmm/common/shared_data.hpp>

#include <map>
#include <optional>
#include <string>

namespace open_lmm {

struct DescriptorAlignmentOptions {
  double pcm_translation_threshold = 10.0;
  double pcm_rotation_threshold_rad = 0.3490658503988659;
  std::string solver = "heuristic";
  int threads = 1;
  std::size_t max_candidates = 0;
};

struct DescriptorAlignmentDiagnostics {
  std::vector<std::size_t> inlier_loop_indices;
};

class DescriptorAlignmentProposer {
 public:
  DescriptorAlignmentProposer(double pcm_translation_threshold,
                              double pcm_rotation_threshold_rad);
  explicit DescriptorAlignmentProposer(DescriptorAlignmentOptions options);

  std::optional<MapAlignmentProposal> Propose(
      char target_agent, char source_agent,
      const PoseVec& source_odometry,
      const AgentOptimizedDataMap& optimized_agents,
      const LoopPairVec& descriptor_loops,
      DescriptorAlignmentDiagnostics* diagnostics = nullptr) const;

 private:
  DescriptorAlignmentOptions options_;
};

}  // namespace open_lmm
