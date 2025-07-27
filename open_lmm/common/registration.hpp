#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>

#include "data_types.hpp"
#include "shared_data.hpp"

namespace open_lmm {
std::optional<Eigen::Isometry3d> registerPointCloud(const SharedDatabase& db,
                                                    const LoopPair& loop_pair,
                                                    const int& searchNum);
// void mergeNearKFPointcloud(const
// std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr>&, const int& key, const
// int& searchNum);
}  // namespace open_lmm
