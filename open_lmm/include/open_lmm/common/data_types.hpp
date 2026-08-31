#pragma once

#include <open_lmm/common/agent_id.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstdint>
#include <Eigen/Geometry>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace open_lmm {

constexpr uint32_t ANCHOR_IDX = std::numeric_limits<uint32_t>::max();

using PoseVec = std::vector<Eigen::Isometry3d>;
// TODO(gil) : refactor base type from pcl to eigen
using ScanVec = std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

struct LoopPair {
  std::pair<AgentId, size_t> to;
  std::pair<AgentId, size_t> from;
  Eigen::Isometry3d init_rel_pose;
};

using LoopPairVec = std::vector<LoopPair>;
using LoopCandidateInfo = std::tuple<AgentId, size_t, Eigen::Isometry3d>;

}  // namespace open_lmm
