#include "Scan.hpp"

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "params.hpp"

// TODO(gil) : refactor parameters
Scan::Scan(const HmmMosParams &params) { initializeScanParams(params); }

void Scan::initializeScanParams(const HmmMosParams &params) {
  voxelSize_ = params.voxel_size;
  minRangeSqr_ = params.min_range * params.min_range;
  maxRange_ = params.max_range;
  minOtsu_ = params.min_otsu;
  outputLabelFolder_ = "";
  // Compute the dimension of the convolution kernel.
  dim_ = params.max_range / params.voxel_size * 2 + 1;

  maxRangeSqr_ = maxRange_ * maxRange_;
  ptsOccupiedHistory_.resize(params.local_window_size);
}

void Scan::addPointsWithIndex() {
  int i = 0;
  std::for_each(scanPtsTf_.cbegin(), scanPtsTf_.cend(), [&](const auto &point) {
    double ptNormSquared = 0;
    double ptDiff = 0;
    for (int j = 0; j < 3; j++) {
      ptDiff = point(j) - sensorPose(j, 3);
      ptNormSquared += ptDiff * ptDiff;
    }
    if (ptNormSquared > minRangeSqr_) {
      auto voxel = Voxel((point / voxelSize_).template cast<int>());
      if (scan_.contains(voxel)) {
        scan_[voxel].pointIndicies.push_back(i);
      } else {
        ScanVoxelState v;
        v.pointIndicies.push_back(i);
        scan_.insert({voxel, v});
        occupiedVoxels.push_back(voxel);
        ptsOccupied.push_back({voxel(0) * voxelSize_, voxel(1) * voxelSize_,
                               voxel(2) * voxelSize_});
      }
    }
    i++;
  });

  // Save the previous and current scan points for the EDF construction.
  ptsOccupiedOverWindow = ptsOccupied;
  unsigned int j = ptsOccupiedHistory_.size() - 1;
  for (auto pt : ptsOccupiedHistory_[j]) {
    ptsOccupiedOverWindow.push_back(pt);
  }
  ptsOccupiedHistory_.push_back(ptsOccupied);
}

bool Scan::checkIfEntryExists(const Voxel &voxel) {
  return scan_.contains(voxel);
}

void Scan::findObservedVoxels() {
  std::vector<std::vector<std::vector<bool> > > obs(
      dim_, std::vector<std::vector<bool> >(dim_, std::vector<bool>(dim_)));
  std::vector<std::vector<Eigen::Vector3i> > all(occupiedVoxels.size());

  // Sensor position.
  int sX = std::floor(sensorPose(0, 3) / voxelSize_);
  int sY = std::floor(sensorPose(1, 3) / voxelSize_);
  int sZ = std::floor(sensorPose(2, 3) / voxelSize_);

  // Find the free voxels traversed for each occupied voxel using the
  // Bresenham line algorihtm.
  tbb::parallel_for(tbb::blocked_range<int>(0, occupiedVoxels.size()),
                    [&](tbb::blocked_range<int> r) {
                      for (unsigned int occupiedVoxNum = r.begin();
                           occupiedVoxNum < r.end(); occupiedVoxNum++) {
                        // To support multi-threading.
                        std::vector<Eigen::Vector3i> ptsObs;

                        // Extract the raycast start and end points.
                        int x0 = sX;
                        int y0 = sY;
                        int z0 = sZ;
                        int x1 = occupiedVoxels[occupiedVoxNum](0);
                        int y1 = occupiedVoxels[occupiedVoxNum](1);
                        int z1 = occupiedVoxels[occupiedVoxNum](2);

                        int sx = x0 < x1 ? 1 : -1;
                        int sy = y0 < y1 ? 1 : -1;
                        int sz = z0 < z1 ? 1 : -1;

                        int dx = abs(x1 - x0);
                        int dy = abs(y1 - y0);
                        int dz = abs(z1 - z0);
                        int dm = std::max(std::max(dx, dy), dz);
                        x1 = y1 = z1 = dm / 2;

                        double ptNorm;
                        double ptDiffX, ptDiffY, ptDiffZ;
                        bool occFlag = false;
                        for (int i = dm; i > -1; i--) {
                          ptDiffX = (sX - x0) * voxelSize_;
                          ptDiffY = (sY - y0) * voxelSize_;
                          ptDiffZ = (sZ - z0) * voxelSize_;
                          ptNorm = (ptDiffX * ptDiffX) + (ptDiffY * ptDiffY) +
                                   (ptDiffZ * ptDiffZ);

                          if (ptNorm <= (maxRangeSqr_)) {
                            ptsObs.push_back({x0, y0, z0});
                          }

                          // Update the next voxel to be traversed.
                          x1 -= dx;
                          if (x1 < 0) {
                            x1 += dm;
                            x0 += sx;
                          }
                          y1 -= dy;
                          if (y1 < 0) {
                            y1 += dm;
                            y0 += sy;
                          }
                          z1 -= dz;
                          if (z1 < 0) {
                            z1 += dm;
                            z0 += sz;
                          }
                        }
                        all[occupiedVoxNum] = ptsObs;
                      }
                    });

  // Save the unique observed voxels from the repeated values in all.
  int i, j, k;
  int offset = maxRange_ / voxelSize_;
  for (const auto &x : all) {
    for (const auto &y : x) {
      i = y(0) + offset - sX;
      j = y(1) + offset - sY;
      k = y(2) + offset - sZ;
      if (!obs[i][j][k]) {
        obs[i][j][k] = true;
        observedVoxels.push_back(y);
      }
    }
  }
}

double Scan::getConvScore(Voxel &voxel) { return scan_[voxel].convScore; }

double Scan::getConvScoreOverWindow(Voxel &voxel) {
  return scan_[voxel].convScoreOverWindow;
}

bool Scan::getDynamic(Voxel &voxel) { return scan_[voxel].isDynamic; }

bool Scan::getDynamicHighConfidence(Voxel &voxel) {
  return scan_[voxel].isDynamic && scan_[voxel].isDynamicHighConfidence;
}

std::vector<int> Scan::getIndicies(Voxel &voxel) {
  std::vector<int> inds;
  for (auto x : scan_[voxel].pointIndicies) {
    inds.push_back(x);
  }
  return inds;
}

void Scan::setConvScore(Voxel voxel, double convScore) {
  scan_[voxel].convScore = convScore;
}

void Scan::setConvScoreOverWindow(Voxel &voxel, double convScore) {
  scan_[voxel].convScoreOverWindow = convScore;
}

void Scan::setDynamic(Voxel &voxel) { scan_[voxel].isDynamic = true; }

void Scan::setDynamicHighConfidence(Voxel &voxel) {
  scan_[voxel].isDynamic = true;
  scan_[voxel].isDynamicHighConfidence = true;
}

void Scan::readScan(const std::string &fileName,
                    const std::vector<double> &pose) {
  unsigned int numColumns = 4;  // Files expected in the .bin KITTI format.
  sensorPose << pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], pose[6],
      pose[7], pose[8], pose[9], pose[10], pose[11], 0, 0, 0, 1;

  // Read the .bin scan file.
  std::ifstream file(fileName, std::ios::in | std::ios::binary);

  if (!file) {
    std::cerr << "./hmm-mos: Scan file " + fileName +
                     " could not be opened! Exiting program."
              << std::endl;
    exit(1);
  }

  float item;
  std::vector<double> ptsFromFile;

  while (file.read((char *)&item, sizeof(item))) {
    ptsFromFile.push_back(item);
  }

  unsigned int numPts = ptsFromFile.size() / numColumns;
  scanPts_.resize(numPts, numColumns);
  scanPtsTf_.resize(numPts);

  tbb::parallel_for(
      tbb::blocked_range<int>(0, numPts), [&](tbb::blocked_range<int> r) {
        for (unsigned int i = r.begin(); i < r.end(); i++) {
          int c = i * numColumns;

          // Save the original point.
          scanPts_.coeffRef(i, 0) = ptsFromFile[c];
          scanPts_.coeffRef(i, 1) = ptsFromFile[c + 1];
          scanPts_.coeffRef(i, 2) = ptsFromFile[c + 2];
          scanPts_.coeffRef(i, 3) = 1;

          // Transform the scan points by the sensor pose.
          scanPtsTf_[i] =
              ((sensorPose * scanPts_.row(i).transpose()).topRows(3));
        }
      });

  file.close();
}

void Scan::readScanPCD(const pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                       const Eigen::Isometry3d &pose) {
  // pose를 Eigen::Matrix4d로 변환
  Eigen::Matrix4d pose_matrix = Eigen::Matrix4d::Identity();
  pose_matrix.block<3, 3>(0, 0) = pose.rotation();
  pose_matrix.block<3, 1>(0, 3) = pose.translation();
  // sensorPose에 할당
  sensorPose = pose_matrix;

  unsigned int numPts = cloud->points.size();
  unsigned int numColumns = 4;
  scanPts_.resize(numPts, numColumns);
  scanPtsTf_.resize(numPts);

  tbb::parallel_for(
      tbb::blocked_range<int>(0, numPts), [&](tbb::blocked_range<int> r) {
        for (unsigned int i = r.begin(); i < r.end(); i++) {
          scanPts_.coeffRef(i, 0) = cloud->points[i].x;
          scanPts_.coeffRef(i, 1) = cloud->points[i].y;
          scanPts_.coeffRef(i, 2) = cloud->points[i].z;
          scanPts_.coeffRef(i, 3) = 1;

          scanPtsTf_[i] =
              ((sensorPose * scanPts_.row(i).transpose()).topRows(3));
        }
      });
}

// 1-3. removeVoxelsOutsideMaxRange()
void Scan::removeVoxelsOutsideMaxRange() {
  double ptNormSquared, ptDiffX, ptDiffY, ptDiffZ;
  std::vector<Eigen::Vector3i> voxToRemove;
  for (const auto &[pt, state] : scan_) {
    ptDiffX = pt.coeffRef(0) * voxelSize_ - sensorPose.coeffRef(0, 3);
    ptDiffY = pt.coeffRef(1) * voxelSize_ - sensorPose.coeffRef(1, 3);
    ptDiffZ = pt.coeffRef(2) * voxelSize_ - sensorPose.coeffRef(2, 3);
    ptNormSquared =
        (ptDiffX * ptDiffX) + (ptDiffY * ptDiffY) + (ptDiffZ * ptDiffZ);
    if (ptNormSquared > (maxRangeSqr_)) {
      voxToRemove.push_back(pt);
    }
  }

  for (const auto &x : voxToRemove) {
    scan_.erase(x);
  }

  // Update the occupied voxels.
  occupiedVoxels.clear();
  for (const auto &[vox, state] : scan_) {
    occupiedVoxels.push_back(vox);
  }
}

void Scan::voxelizeScan() {
  ptsOccupied.clear();
  occupiedVoxels.clear();
  observedVoxels.clear();
  scan_.clear();

  // Voxelize the scan.
  addPointsWithIndex();

  // Raycast to find all observed voxels.
  findObservedVoxels();

  // Remove voxelized measurements that are out of range.
  removeVoxelsOutsideMaxRange();
}

void Scan::writeFile(std::ofstream &outFile, unsigned int scanNum) {
  std::stringstream outConstructor;
  std::string out;
  outConstructor << scanNum + 1;
  for (auto &[vox, state] : scan_) {
    if (state.isDynamic && dynThreshold > minOtsu_) {
      for (auto pt : state.pointIndicies) {
        outConstructor << "," << pt;
      }
    }
  }
  outConstructor >> out;
  outFile << out;
  outFile << std::endl;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Scan::getGlobalStaticScan() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::set<int> staticInds;
  if (dynThreshold > minOtsu_) {
    for (auto &[vox, state] : scan_) {
      if (!state.isDynamic) {
        for (auto pt : state.pointIndicies) staticInds.insert(pt);
      }
    }
  } else {
    for (auto &[vox, state] : scan_) {
      staticInds.insert(state.pointIndicies.begin(), state.pointIndicies.end());
    }
  }

  for (int i = 0; i < scanPtsTf_.size(); i++) {
    if (staticInds.find(i) != staticInds.end()) {
      pcl::PointXYZI point;
      point.x = scanPtsTf_[i](0);
      point.y = scanPtsTf_[i](1);
      point.z = scanPtsTf_[i](2);
      point.intensity = 0;
      cloud->push_back(point);
    }
  }

  return cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr Scan::getGlobalDynamicScan() {
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(
      new pcl::PointCloud<pcl::PointXYZI>);

  std::set<int> dynamicInds;
  if (dynThreshold > minOtsu_) {
    for (auto &[vox, state] : scan_) {
      if (state.isDynamic) {
        for (auto pt : state.pointIndicies) dynamicInds.insert(pt);
      }
    }
  }

  for (int i = 0; i < scanPtsTf_.size(); i++) {
    if (dynamicInds.find(i) != dynamicInds.end()) {
      pcl::PointXYZI point;
      point.x = scanPtsTf_[i](0);
      point.y = scanPtsTf_[i](1);
      point.z = scanPtsTf_[i](2);
      point.intensity = 1;
      cloud->push_back(point);
    }
  }

  return cloud;
}

void Scan::writeLabel(const std::string &outputLabelFolder,
                      unsigned int scanNum) {
  std::set<int> dynInds;
  if (dynThreshold > minOtsu_) {
    for (auto &[vox, state] : scan_) {
      if (state.isDynamic) {
        for (auto pt : state.pointIndicies) dynInds.insert(pt);
      }
    }
  }

  // Write the dynamic indicies.
  int ptCloudSize = 0;
  ptCloudSize = scanPts_.rows();
  Eigen::VectorXi outputLabels = Eigen::VectorXi::Ones(ptCloudSize);
  outputLabels = outputLabels * 9;  // Static.

  // Overwrite the static labels.
  for (auto x : dynInds) {
    outputLabels(x) = 251;  // Dynamic.
  }

  std::stringstream fNameMaker;
  fNameMaker << std::setw(6) << std::setfill('0') << std::to_string(scanNum)
             << ".label";

  std::string fName;
  fNameMaker >> fName;

  std::ofstream outFile;
  outFile.open(outputLabelFolder_ + fName, std::ios::binary | std::ios::out);
  for (unsigned int j = 0; j < outputLabels.rows(); j++) {
    uint32_t label = uint32_t(outputLabels(j)) & 0xFFFF;
    outFile.write(reinterpret_cast<const char *>(&label), sizeof(label));
  }
  outFile.close();
}