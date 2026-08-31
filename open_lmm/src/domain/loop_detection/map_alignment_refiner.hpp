#pragma once

#include <open_lmm/common/alignment_types.hpp>

#include <Eigen/Core>
#include <vector>

namespace open_lmm {

struct MapAlignmentRefinementResult {
  Eigen::Isometry3d target_T_source = Eigen::Isometry3d::Identity();
  std::size_t correspondence_count = 0;
  std::optional<double> fitness;
  std::optional<double> overlap_ratio;
  bool refined = false;
};

MapAlignmentRefinementResult RefineMapAlignment(
    const std::vector<Eigen::Vector3f>& target_map,
    const std::vector<Eigen::Vector3f>& source_map,
    const Eigen::Isometry3d& initial_target_T_source,
    bool run_icp_refinement);

}  // namespace open_lmm
