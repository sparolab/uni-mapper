#pragma once
#include <tqdmcpp/tqdmcpp.hpp>


#include <filesystem>
#include <fstream>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/pose_conversion.hpp>

#include "data_loader_base.hpp"

namespace open_lmm {

struct DataLoaderFileParam {
  std::string data_loader_type;
  std::string pose_file_name;
  std::string pose_format;
  std::string scan_type;
  std::string scan_dir_name;
  Eigen::Isometry3d extrinsic;
  float voxel_size;
  float min_range;
  float max_range;
  std::string delimiter;
};

class DataLoaderFile : public DataLoaderBase {
 public:
  explicit DataLoaderFile(Config config);
  ~DataLoaderFile() override = default;
  void parseConfig(Config config) override;
  AgentRawData Process(const AgentContext& ctx,
                       const fs::path& data_dir) override;
  PoseVec loadPoseData(fs::path data_dir_path);
  ScanVec loadRawScanData(fs::path data_dir_path) override;
  ScanVec loadFilteredScanData(fs::path data_dir_path);
  std::function<Eigen::Isometry3d(std::vector<double>&)> transformFunctor;
  std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::string)>
      convertScanFunctor;

 private:
  DataLoaderFileParam param_;
};

}  // namespace open_lmm