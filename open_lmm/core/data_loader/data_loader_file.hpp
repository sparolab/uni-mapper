#pragma once
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/pose_conversion.hpp>
#include <tqdmcpp/tqdmcpp.hpp>

#include <filesystem>
#include <fstream>

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
  std::tuple<PoseVec, ScanVec, ScanVec> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      fs::path data_dir_path) override;
  PoseVec loadPoseData(fs::path data_dir_path) override;
  ScanVec loadRawScanData(fs::path data_dir_path) override;
  ScanVec loadFilteredScanData(fs::path data_dir_path) override;
  std::function<Eigen::Isometry3d(std::vector<double>&)> transformFunctor;
  std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::string)>
      convertScanFunctor;

 private:
  DataLoaderFileParam param_;
};

}  // namespace open_lmm