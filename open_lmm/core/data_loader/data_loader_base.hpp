#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/utils/config.hpp>
#include <string>
namespace fs = std::filesystem;

namespace open_lmm {

class DataLoaderBase {
 public:
  DataLoaderBase() = default;
  explicit DataLoaderBase(Config config);
  virtual ~DataLoaderBase() = default;
  virtual std::tuple<PoseVec, ScanVec, ScanVec> process(
      std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
      fs::path data_dir_path) = 0;
  virtual PoseVec loadPoseData(fs::path data_dir_path) = 0;
  virtual ScanVec loadRawScanData(fs::path data_dir_path) = 0;
  virtual ScanVec loadFilteredScanData(fs::path data_dir_path) = 0;

  virtual void parseConfig(Config config) = 0;
  static std::unique_ptr<DataLoaderBase> createInstance(Config config);
};

}  // namespace open_lmm