/** @brief This file partially implements the ScanContext descriptor interface
 * defined in scan_context.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */
#include "scan_context.h"

#include <cmath>
#include <iostream>
#include <numeric>

ScanContextParams::ScanContextParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_loop_detector"));
  number_sectors = config.param<int>("loop_detector", "num_sector", 60);
  number_rings = config.param<int>("loop_detector", "num_ring", 20);
  max_range = config.param<double>("loop_detector", "max_range", 80);
}

bool ScanContextParams::equals(const ScanContextParams& other) const {
  // Check all parameters are equal
  return this->number_sectors == other.number_sectors &&  //
         this->number_rings == other.number_rings &&  //
         this->max_range == other.max_range;
}

/*********************************************************************************************************************/
ScanContext::ScanContext(const ScanContextParams& params) : params_(params) {
  descriptor_ =
      Eigen::MatrixXd::Zero(params_.number_rings, params.number_sectors);
  ring_key_ = Eigen::VectorXd::Zero(params_.number_rings);
}

/*********************************************************************************************************************/
const Eigen::MatrixXd& ScanContext::getDescriptor() const {
  return descriptor_;
}

/*********************************************************************************************************************/
const Eigen::VectorXd& ScanContext::getDescriptorKey() const {
  return ring_key_;
}

/*********************************************************************************************************************/
const ScanContextParams& ScanContext::getParams() const { return params_; }

/*********************************************************************************************************************/
std::pair<double, Eigen::Isometry3d> ScanContext::distance(
    const std::shared_ptr<IDescriptorKdtree>& other) const {
  // Guard Code: Two ScanContexts are only comparable if they are constructed
  // with identical parameters
  // TODO : remove param comparison
  // if (!params_.equals(other.params_)) {
  //   throw std::runtime_error(
  //       "ScanContext::distance The two ScanContexts are not comparable due to
  //       different parameters.");
  // }

  // Compute the min distance over all column shifts ("rotations") of this
  // descriptor
  double min_distance = std::numeric_limits<double>::max();
  size_t min_sector_offset = 0;
  for (size_t sector_offset = 0; sector_offset < params_.number_sectors;
       sector_offset++) {
    double distance = shiftedDistance(sector_offset, other);
    if (distance < min_distance) {
      min_distance = distance;
      min_sector_offset = sector_offset;
    }
  }
  const double sector_resolution =
      (2 * M_PI) / params_.number_sectors;  // radians
  // TODO(gil): AngleAxisd?
  Eigen::Isometry3d min_rel_pose = Eigen::Isometry3d::Identity();
  min_rel_pose.rotate(Eigen::AngleAxisd(min_sector_offset * sector_resolution,
                                        Eigen::Vector3d::UnitZ()));
  return {min_distance, min_rel_pose};
}

/*********************************************************************************************************************/
double ScanContext::ringKeyDistance(const ScanContext& other) const {
  return (ring_key_ - other.ring_key_).norm();
}

/*********************************************************************************************************************/
double ScanContext::shiftedDistance(
    const size_t& sector_offset,
    const std::shared_ptr<IDescriptorKdtree>& other) const {
  double sum_term = 0.0;
  for (size_t sector_idx = 0; sector_idx < params_.number_sectors;
       sector_idx++) {
    // Compute the sector for this sector idx after the rotation by
    // sector_offset
    const size_t this_offset_sector_idx =
        (sector_idx + sector_offset) % params_.number_sectors;
    // Extract the respective columns from the two descriptors
    const Eigen::VectorXd this_sector = descriptor_.col(this_offset_sector_idx);
    const double this_norm = this_sector.norm();
    const Eigen::VectorXd other_sector = other->getDescriptor().col(sector_idx);
    const double other_norm = other_sector.norm();

    /** Compute the cosine distance between the two sectors
     * Note: there are two edge cases, if either sector is empty
     * If both are empty we say the distance is zero
     * If only one is empty we say distance is equal two 1 (i.e. vectors are
     * orthogonal) WARN: These edge cases are never addressed in [1], so these
     * are based on intuition
     */
    if (this_norm > 0 && other_norm > 0) {  // Standard Cosine Distance
      sum_term += 1 - ((this_sector.dot(other_sector)) /
                       (this_sector.norm() * other_sector.norm()));
    } else if (this_norm == 0 &&
               other_norm == 0) {  // All zero sectors, assume zero distance
      sum_term += 0;
    } else {  // One zero and one non-zero sector, assume orthogonal distance
      sum_term += 1;
    }
  }

  // Average the summed cosine distances
  return sum_term / params_.number_sectors;
}

/*********************************************************************************************************************/
std::shared_ptr<IDescriptorKdtree> ScanContext::makeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) {
  std::shared_ptr<ScanContext> sc = std::make_shared<ScanContext>(params_);

  // Compute the resolutions used for computing ring and sector indicies
  const double ring_resolution =
      sc->params_.max_range / sc->params_.number_rings;  // meters
  const double sector_resolution =
      (2 * M_PI) / sc->params_.number_sectors;  // radians

  // Track the minimum height in the scan so we can normalize the descriptor to
  // be positive starting at zero WARN: This is only ever implied in [1] in Fig.
  // 1, never explicitly outlined
  double min_height = std::numeric_limits<double>::max();

  // Iterate over all points
  for (const pcl::PointXYZI& point : lidar_scan->points) {
    // Compute range to point
    const double x = point.x;  // meters
    const double y = point.y;  // meters
    const double height = point.z;  // meters
    const double range = std::sqrt(x * x + y * y);  // meters
    const double angle_rad =
        std::fmod(std::atan2(y, x) + 2 * M_PI, 2 * M_PI);  // radians

    // Include points only they are within the parameterized range
    if (range < sc->params_.max_range) {
      // Get the index for this point
      const size_t ring_idx = static_cast<size_t>(range / ring_resolution);
      const size_t sector_idx =
          static_cast<size_t>(angle_rad / sector_resolution);

      // Update the descriptor at the corresponding location
      if (sc->descriptor_(ring_idx, sector_idx) ==
          0.0) {  // If uninitialized, initialize the cell
        sc->descriptor_(ring_idx, sector_idx) = height;
      } else {  // If initialized, take the max
        sc->descriptor_(ring_idx, sector_idx) =
            std::max(sc->descriptor_(ring_idx, sector_idx), height);
      }

      // Update the min height for later normalization
      min_height = std::min(min_height, height);
    }
  }

  // Normalize the descriptor according to the min height, leaving uninitialized
  // cells as zero
  for (size_t r = 0; r < sc->params_.number_rings; r++) {
    for (size_t s = 0; s < sc->params_.number_sectors; s++) {
      if (sc->descriptor_(r, s) != 0.0) sc->descriptor_(r, s) -= min_height;
    }
  }

  // Construct the ring key for the now complete ScanContext
  for (size_t r = 0; r < sc->params_.number_rings; r++) {
    sc->ring_key_(r) =
        static_cast<double>(sc->descriptor_.row(r).array().count()) /
        sc->params_.number_sectors;
  }

  return sc;
}
