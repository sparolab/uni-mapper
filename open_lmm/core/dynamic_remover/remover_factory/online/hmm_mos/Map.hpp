#pragma once

#ifndef HMMMOS_MAP_H_
#define HMMMOS_MAP_H_

#include <boost/circular_buffer.hpp>

#include "Scan.hpp"
#include "VoxelHash.hpp"
#include "nanoflannUtils.hpp"
#include "utils.hpp"

/**
 * @brief The Map is recurisvely updated using Scans (see Scan.hpp) and is
 *        used to build confidence in the state of the environment. The Map
 *        is represented using voxels. Each voxel has several attributes to
 *        describe its behaviour. These are used to detect determine voxels,
 *        that are projected back to the original point cloud measurements.
 */

struct HmmMosParams;
class Map {
 public:
  /**
   * @brief Container to store the HMM configuration parameters.
   *
   */
  struct HmmConfig {
    unsigned int numStates;
    Eigen::MatrixXd stateTransitionMatrix;
    double sigOcc;
    double beliefThreshold;
  };

  /**
   * @brief Construct a new Map object.
   *
   * @param params
   */
  Map(const HmmMosParams &params);

  /**
   * @brief Destroy the Map object.
   *
   */
  ~Map() = default;

  /**
   * @brief Container to store and set the HMM config parameters.
   *
   */
  HmmConfig hmmConfig;

  /**
   * @brief Determine the dynamic voxels for the current scan using the
   *        recursively updated map.
   *
   * @param scan          The scan to find the dynamic voxels in.
   * @param scanHistory   A history of the previous n scans.
   */
  void findDynamicVoxels(Scan &scan, boost::circular_buffer<Scan> &scanHistory);

  /**
   * @brief Perform a median filtering operation on the scan's
   *        convolution scores.
   *
   * @param scan The scan to perform the filtering operation on.
   */
  void findMedianValue(Scan &scan);

  /**
   * @brief Update the map with the current scan.
   *
   * @param scan      The scan to update the map.
   * @param scanNum   The current scan number.
   */
  void update(Scan &scan, unsigned int scanNum);

 private:
  /**
   * @brief  States of each voxel in the map.
   *         These are the attributes used to describe the voxel.
   *
   */
  struct MapVoxelState {
    // Used for Gaussian Distance Field (GDF) construction.
    double closestDistance = 0;

    // The current state of the voxel,
    // (0,1,2) = (unobserved, occupied, free).
    unsigned int currentState = 0;

    // The scan the current state was updated.
    unsigned int currentStateScan = 0;

    // Dynamic state of the voxel.
    // True is dynamic, false if static.
    bool isDynamic = false;

    // The scan of the last change in state.
    unsigned int lastStateChangeScan[2] = {0, 0};

    // The last time the voxel was seen.
    unsigned int scanLastSeen = 0;

    // The voxel's state vector, initialized in the unobserved state.
    Eigen::Vector3d xHat = {1, 0, 0};
  };

  using pointsIterator = std::vector<Eigen::Vector3i>::const_iterator;

  int convSize_, edge_, globalWinLen_, nBins_, scanNum_;
  double normDistOccDen_;
  double minOtsu_, maxRangeSqr_, voxelSize_;
  Eigen::Matrix4d sensorPose_;

  // The previous scans dynamic voxels used to perserve dynamic voxels.
  std::vector<Voxel> prevScanDynamicVoxels_;

  // The current scans dynamic voxels.
  std::vector<Voxel> scanDynamicVoxels_;

  // Container for a nanoflann-friendly point cloud.
  NanoflannPointsContainer<double> pcForKdTree_;

  // Voxelized map object.
  ankerl::unordered_dense::map<Voxel, MapVoxelState, VoxelHash> map_;

  /**
   * @brief Add voxels to the current map.
   *
   * @param voxels Voxels to add to the current map.
   */
  void addVoxels(const std::vector<Voxel> &voxels);

  /**
   * @brief Convert the points to a KD-Tree friendly container.
   *
   * @param pts The points to convert to a KD-Tree friendly container.
   */
  void convertToKdTreeContainer(std::vector<Eigen::Vector3d> &pts);

  /**
   * @brief Remove voxels outside the global moving window and the
   *        maximum range of the sensor.
   *
   */
  void removeVoxelsOutsideWindowAndMaxRange();

  void initializeMapParams(const HmmMosParams &params);
};

#endif  // HMMMOS_MAP_H_