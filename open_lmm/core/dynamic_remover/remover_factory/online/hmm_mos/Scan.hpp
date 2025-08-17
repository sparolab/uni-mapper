#pragma once

#ifndef HMMMOS_SCAN_H_
#define HMMMOS_SCAN_H_

#include <boost/circular_buffer.hpp>

#include <Eigen/Core>
#include <fstream>
#include <iomanip>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <set>
#include <string>
#include <vector>

#include "VoxelHash.hpp"
#include "ankerl/unordered_dense.h"
#include "nanoflannUtils.hpp"
#include "tbb/parallel_for.h"
#include "utils.hpp"

/**
 * @brief A Scan voxelizes a point cloud input. A raycasting operation is
 *        performed to find all observed voxels (occupied and free). The scan
 *        is integrated into the map (see Map.hpp) to determine the static and
 *        dynamic points using information from the recursively updated map.
 */

struct HmmMosParams;
class Scan {
 public:
  Scan(const HmmMosParams& params);

  /**
   * @brief Destroy the Scan object.
   *
   */
  ~Scan() = default;

  /**
   * @brief Check if a voxel exists in the current scan.
   *
   * @param voxel     The voxel to be checked.
   * @return true     If the voxel exists in the current scan.
   * @return false    If the voxel does not exist in the current scan.
   */
  bool checkIfEntryExists(const Voxel& voxel);

  /**
   * @brief  Find all observed voxels for the current scan using a
   *         3D variant of Bresenham's line algroithm.
   *
   */
  void findObservedVoxels();

  /**
   * @brief Get the voxel's convolution score.
   *
   * @param voxel     The voxel to get the convolution score of.
   * @return double   The convolution score.
   */
  double getConvScore(Voxel& voxel);

  /**
   * @brief Get the voxel's convolution score over the window.
   *
   * @param voxel     The voxel to get the convolution score of.
   * @return double   The conolution score.
   */
  double getConvScoreOverWindow(Voxel& voxel);

  /**
   * @brief Get the dynamic state of the voxel.
   *
   * @param voxel     The voxel to get the dynamic state of.
   * @return true     If the voxel is currently dynamic.
   * @return false    If the voxel is current static.
   */
  bool getDynamic(Voxel& voxel);

  /**
   * @brief Get the high confidence dynamic state of the voxel.
   *
   * @param voxel     The voxel to get the high confidence dynamic
   *                  state of.
   * @return true     If the voxel is dynamic with high confidence.
   * @return false    If the voxel is not dyanmic with high confidence.
   */
  bool getDynamicHighConfidence(Voxel& voxel);

  /**
   * @brief Get the original point cloud indicies captured by the voxel.
   *
   * @param voxel                 The voxel to get the indicies of.
   * @return std::vector<int>     The list of point cloud indicies
   *                              captured by the voxel.
   */
  std::vector<int> getIndicies(Voxel& voxel);

  /**
   * @brief Read scan from the .bin file specified in fileName and
   *        transform it by the current pose.
   *
   * @param fileName Scan file. Must be in the KITTI .bin format.
   * @param pose     Pose of the sensor at the current scan, given as a
   *                 6x1 vector, (roll, pitch, yaw, x, y, z) [rad,m].
   */
  void readScan(const std::string& fileName, const std::vector<double>& pose);

  /**
   * @brief Read scan from the .pcd file specified in fileName and
   *        transform it by the current pose.
   *
   * @param fileName Scan file. Must be in the PCD format.
   * @param pose     Pose of the sensor at the current scan, given as a
   *                 6x1 vector, (roll, pitch, yaw, x, y, z) [rad,m].
   */
  void readScanPCDFile(const std::string& fileName,
                       const std::vector<double>& pose);

  void readScanPCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
                   const Eigen::Isometry3d& pose);

  /**
   * @brief Remove voxels outside the maximum range specified
   *        in the configuration.
   *        This step is performed on all observed voxels to ensure
   *        free space voxels are included even if the range is
   *        greater than the configured value.
   *
   */
  void removeVoxelsOutsideMaxRange();

  /**
   * @brief Set the voxel's convolution score.
   *
   * @param voxel     The voxel for which the score is to be set.
   * @param convScore The convolution score to be set.
   */
  void setConvScore(Voxel voxel, double convScore);

  /**
   * @brief Set the voxel's convolution score over the window.
   *
   * @param voxel     The voxel for which the score is to be set.
   * @param convScore The convolution score to be set.
   */
  void setConvScoreOverWindow(Voxel& voxel, double convScore);

  /**
   * @brief Set the voxel to be in dynamic state.
   *
   * @param voxel The voxel to be set as dynamic.
   */
  void setDynamic(Voxel& voxel);

  /**
   * @brief Set the voxel to be in a high confidence dynamic state.
   *
   * @param voxel The voxel to be set as dynamic.
   */
  void setDynamicHighConfidence(Voxel& voxel);

  /**
   * @brief Resets the scan object and updates attributes with the new
   *        scan. The scan is voxelized, a raycasting operation is
   *        perfomed to determine all observed voxels, and voxels outside
   *        the maximum range are removed.
   *
   */
  void voxelizeScan();

  /**
   * @brief Write the dynamic indicies to the output file.
   *
   * @param outFile The file handler to write the result to.
   * @param scanNum The current scan number.
   */
  void writeFile(std::ofstream& outFile, unsigned int scanNum);

  /**
   * @brief Write the .label file for the current scan.
   *
   * @param scanNum
   */
  void writeLabel(const std::string& outputLabelFolder, unsigned int scanNum);

  pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalStaticScan();
  pcl::PointCloud<pcl::PointXYZI>::Ptr getGlobalDynamicScan();

  /**
   * @brief The current sensor pose as a 4x4 Homogeneous matrix.
   *
   */
  Eigen::Matrix4d sensorPose;

  // Unique set of the occupied and observed voxels used to upate the global
  // map.
  std::vector<Eigen::Vector3i> observedVoxels;
  std::vector<Eigen::Vector3i> occupiedVoxels;

  // The unquantized point cloud measurements used to construct the
  // scan's KD-Tree and then the Euclidean Distance Field.
  std::vector<Eigen::Vector3d> ptsOccupied;
  std::vector<Eigen::Vector3d> ptsOccupiedOverWindow;

  // The Otsu threshold for the dynamic voxel segmentation used for
  // comparison with the minOtsu configuration parameter.
  double dynThreshold;

 private:
  /**
   * @brief States of each voxel in the scan.
   *        Note the dynamic object segmentation is performed in the
   *        scan using information updated in the map.
   *
   */
  struct ScanVoxelState {
    // Voxel dynamicity.
    bool isDynamic = false;
    bool isDynamicHighConfidence = false;

    // Point cloud indicies in the current voxel.
    std::vector<unsigned int> pointIndicies;

    // Spatial 3D convolution score.
    double convScore = 0;

    // Spatio-temporal 4D convolution score.
    double convScoreOverWindow = 0;
  };

  // Configuration.
  int dim_;
  double voxelSize_, minRangeSqr_, maxRange_, maxRangeSqr_, minOtsu_;
  std::string outputLabelFolder_;

  // Original scan Cartesian points read from the file.
  Eigen::MatrixXd scanPts_;

  // Scan points transformed by the current pose estimate.
  std::vector<Eigen::Vector3d> scanPtsTf_;

  // History of the scan points used for KD-Tree construction.
  boost::circular_buffer<std::vector<Eigen::Vector3d> > ptsOccupiedHistory_;

  // Voxelized scan.
  ankerl::unordered_dense::map<Voxel, ScanVoxelState, VoxelHash> scan_;

  /**
   * @brief Add the measurements from the new scan to the
   *        voxelized scan.
   *
   */
  void addPointsWithIndex();

  void initializeScanParams(const HmmMosParams& params);
};

#endif  // HMMMOS_SCAN_H_