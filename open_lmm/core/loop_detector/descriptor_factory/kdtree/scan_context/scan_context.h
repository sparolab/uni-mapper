/** @brief Class definition for the Scan Context descriptor.
 * ScanContext is a lidar scan descriptor introduced by [1] used for place
 * recognition. The following library implements ScanContext exactly as
 * described in [1] using only the paper as reference.
 *
 * [1] G. Kim and A. Kim, "Scan Context: Egocentric Spatial Descriptor for Place
 * Recognition Within 3D Point Cloud Map," 2018 IEEE/RSJ International
 * Conference on Intelligent Robots and Systems (IROS), Madrid, Spain, 2018, pp.
 * 4802-4809,
 *
 * @author Dan McGann
 * @date March 2024
 */

#pragma once
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp>
#include <open_lmm/utils/config.hpp>

#include <Eigen/Dense>
#include <cstddef>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <vector>

struct ScanContextParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  ScanContextParams();
  // explicit Params(Config config);
  ~ScanContextParams() = default;

 public:
  size_t number_sectors{60};
  size_t number_rings{20};
  double max_range{80};
  size_t descriptor_vector_dim{number_rings};
  bool equals(const ScanContextParams& other) const;
};

/// @brief The Scan Context Descriptor
class ScanContext : public IDescriptorKdtree {
  /** Fields **/
 protected:
  /// @brief The parameters used to construct this ScanContext descriptor
  ScanContextParams params_;
  /// @brief The ScanContext descriptor for a lidar scan
  /// Of size [number_rings, number_sectors] where each row corresponds to a
  /// ring of lidar data
  Eigen::MatrixXd descriptor_;
  /// @brief The ring key descriptor, a rotational invariant summary of the
  /// ScanContext descriptor Of size [number_rings] where each element is the L1
  /// norm of a row in descriptor_
  Eigen::VectorXd ring_key_;

  /** Interface **/
 public:
  /// @brief Constructs an Empty ScanContext. See fromScan to construct from a
  /// lidar Scan.
  ScanContext(const ScanContextParams& params);

  /** @brief Constructs a ScanContext descriptor from a lidar scan
   * @tparam Accessor - Class used to access the x, y, z data from points in the
   * point clouds
   * @tparam PointType - The type of the point in the pointcloud must contain x,
   * y, z data in some form
   * @tparam Alloc - The allocator used for the pointcloud vector
   * @param lidar_scan: The lidar scan to describe
   * @param params: The ScanContext parameters
   * @return A ScanContext descriptor for lidar_scan
   */
  std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) override;

  /// @brief Accessor to the descriptor that guards against modification
  const Eigen::MatrixXd& getDescriptor() const override;

  /// @brief Accessor to the ring key that guards against modification
  const Eigen::VectorXd& getDescriptorKey() const override;

  /// @brief Accessor to the params that guards against modification
  const ScanContextParams& getParams() const;

  /** @brief Computes the distance between this descriptor and the other
   * descriptor Distance is defined as as the column-wise cosine distance
   * minimized over all possible column shifts. See [1] Eq.6 for exact details.
   * @param other: The other ScanContext descriptor with which to compare
   * distances
   * @returns The distance between the two descriptors
   * @note Distance is symmetric i.e. this.distance(other) ==
   * other.distance(this)
   * @note we can only compute distances for "comparable" descriptors i.e. those
   * with equal params
   */
  std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const override;

  /** @brief Computes the distance between the ring key of this descriptor and
   * the ring key of the other descriptor. Distance is defined as the euclidean
   * distance between the two ring key vectors.
   * @param other: The other ScanContext descriptor with which to compare ring
   * key distances
   * @returns The ring key distance between two descriptors
   * @note By definition ring key distance is symmetric
   */
  double ringKeyDistance(const ScanContext& other) const;

  /** Helpers **/
 private:
  /** @brief Computes the distance between this ScanContext "rotated" by the
   * given sector offset and the other. Used as a helper in
   * ScanContext::distance, and is ebullient to [1] Eq.5
   * @param sector_offset: The sector offset used to "rotate" this descriptor
   * when comparing to other
   * @param other: The other descriptor with which we are comparing
   * @returns The cosine distance between this rotated by sector offset and
   * other
   * @note Assumes that this and other are comparable
   */
  double shiftedDistance(const size_t& sector_offset,
                         const std::shared_ptr<IDescriptorKdtree>& other) const;
};

/// Include the implementation of the descriptor
// #include "scan_context-inl.h"