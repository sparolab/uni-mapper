#include "map_alignment_refiner.hpp"

#include <pcl/kdtree/kdtree_flann.h>

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <limits>

namespace open_lmm {
namespace {
pcl::PointCloud<pcl::PointXYZ>::Ptr ToCloud(
    const std::vector<Eigen::Vector3f>& points) {
  auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  cloud->reserve(points.size());
  for (const auto& point : points) {
    if (point.allFinite()) cloud->emplace_back(point.x(), point.y(), point.z());
  }
  return cloud;
}
}  // namespace

MapAlignmentRefinementResult RefineMapAlignment(
    const std::vector<Eigen::Vector3f>& target_map,
    const std::vector<Eigen::Vector3f>& source_map,
    const Eigen::Isometry3d& initial_target_T_source,
    bool run_icp_refinement) {
  MapAlignmentRefinementResult result;
  result.target_T_source = initial_target_T_source;
  auto target = ToCloud(target_map);
  auto source = ToCloud(source_map);
  if (target->empty() || source->empty()) return result;

  constexpr double kMaximumCorrespondenceDistance = 5.0;
  if (run_icp_refinement && target->size() >= 10 && source->size() >= 10) {
    pcl::KdTreeFLANN<pcl::PointXYZ> refinement_tree;
    refinement_tree.setInputCloud(target);
    const float maximum_squared_distance = static_cast<float>(
        kMaximumCorrespondenceDistance * kMaximumCorrespondenceDistance);
    constexpr std::size_t kMaximumSamples = 50000;
    const std::size_t stride =
        std::max<std::size_t>(1, source->size() / kMaximumSamples);
    Eigen::Isometry3d current = initial_target_T_source;
    std::vector<int> indices(1);
    std::vector<float> squared_distances(1);
    for (int iteration = 0; iteration < 40; ++iteration) {
      std::vector<std::pair<Eigen::Vector3d, Eigen::Vector3d>> pairs;
      pairs.reserve(std::min(source->size(), kMaximumSamples));
      Eigen::Vector3d source_centroid = Eigen::Vector3d::Zero();
      Eigen::Vector3d target_centroid = Eigen::Vector3d::Zero();
      for (std::size_t i = 0; i < source->size(); i += stride) {
        const auto& point = (*source)[i];
        const Eigen::Vector3d source_point = current * Eigen::Vector3d(
            point.x, point.y, point.z);
        const pcl::PointXYZ query(static_cast<float>(source_point.x()),
                                  static_cast<float>(source_point.y()),
                                  static_cast<float>(source_point.z()));
        if (refinement_tree.nearestKSearch(
                query, 1, indices, squared_distances) != 1 ||
            squared_distances[0] > maximum_squared_distance) {
          continue;
        }
        const auto& match = (*target)[indices[0]];
        const Eigen::Vector3d target_point(match.x, match.y, match.z);
        pairs.emplace_back(source_point, target_point);
        source_centroid += source_point;
        target_centroid += target_point;
      }
      if (pairs.size() < 3) break;
      source_centroid /= static_cast<double>(pairs.size());
      target_centroid /= static_cast<double>(pairs.size());
      Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
      for (const auto& [source_point, target_point] : pairs) {
        covariance += (source_point - source_centroid) *
                      (target_point - target_centroid).transpose();
      }
      const Eigen::JacobiSVD<Eigen::Matrix3d> svd(
          covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d rotation = svd.matrixV() * svd.matrixU().transpose();
      if (rotation.determinant() < 0.0) {
        Eigen::Matrix3d corrected_v = svd.matrixV();
        corrected_v.col(2) *= -1.0;
        rotation = corrected_v * svd.matrixU().transpose();
      }
      Eigen::Isometry3d delta = Eigen::Isometry3d::Identity();
      delta.linear() = rotation;
      delta.translation() = target_centroid - rotation * source_centroid;
      current = delta * current;
      result.refined = true;
      const double cosine =
          std::clamp((rotation.trace() - 1.0) * 0.5, -1.0, 1.0);
      if (delta.translation().norm() < 1.0e-4 &&
          std::acos(cosine) < 1.0e-4) {
        break;
      }
    }
    if (result.refined &&
        ValidateRigidTransform(current, "ICP refinement output")) {
      result.target_T_source = current;
    }
  }

  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud(target);
  const float maximum_squared_distance = static_cast<float>(
      kMaximumCorrespondenceDistance * kMaximumCorrespondenceDistance);
  double squared_distance_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (const auto& point : *source) {
    const Eigen::Vector3d transformed_point =
        result.target_T_source * Eigen::Vector3d(point.x, point.y, point.z);
    const pcl::PointXYZ query(static_cast<float>(transformed_point.x()),
                              static_cast<float>(transformed_point.y()),
                              static_cast<float>(transformed_point.z()));
    if (tree.nearestKSearch(query, 1, indices, squared_distances) == 1 &&
        squared_distances[0] <= maximum_squared_distance) {
      ++result.correspondence_count;
      squared_distance_sum += squared_distances[0];
    }
  }
  result.overlap_ratio = static_cast<double>(result.correspondence_count) /
                         static_cast<double>(source->size());
  if (result.correspondence_count > 0) {
    result.fitness = squared_distance_sum /
                     static_cast<double>(result.correspondence_count);
  }
  return result;
}

}  // namespace open_lmm
