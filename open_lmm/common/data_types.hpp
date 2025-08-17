#pragma once

#include <Eigen/Geometry>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/scan_context/scan_context.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

namespace open_lmm {

constexpr uint32_t ANCHOR_IDX = std::numeric_limits<uint32_t>::max();

using PoseVec = std::vector<Eigen::Isometry3d>;
// TODO(gil) : refactor base type from pcl to eigen
using ScanVec = std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>;

struct LoopPair {
  std::pair<char, size_t> to;
  std::pair<char, size_t> from;
  Eigen::Isometry3d init_rel_pose;
};

using LoopPairVec = std::vector<LoopPair>;
using LoopCandidateInfo = std::tuple<char, size_t, Eigen::Isometry3d>;

}  // namespace open_lmm

// LoopPair makeLoopPair(const char db_id, const size_t db_idx, const char
// agent_id, const size_t agent_idx)
// {
//   LoopPair loop;
//   loop.to = std::make_pair(db_id, db_idx);
//   loop.from = std::make_pair(agent_id, agent_idx);
//   return loop;
// }
