#pragma once
#include <filesystem>
#include <fstream>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/pose_conversion.hpp>

#include "data_loader_base.hpp"

namespace open_lmm {

using DataLoaderFileParam = DataLoaderConfig;

class DataLoaderFile : public DataLoaderBase {
 public:
  explicit DataLoaderFile(DataLoaderConfig config);
  ~DataLoaderFile() override = default;
  Result<AgentRawData> Process(const AgentContext& ctx,
                               const fs::path& data_dir) override;
  Result<PoseVec> loadPoseData(fs::path data_dir_path);
  Result<ScanVec> loadRawScanData(fs::path data_dir_path) override;
  Result<std::size_t> VisitRawScanData(
      const fs::path& data_dir_path,
      const RawScanVisitor& visitor) override;
  Result<ScanVec> loadFilteredScanData(fs::path data_dir_path);
  std::function<Eigen::Isometry3d(std::vector<double>&)> transformFunctor;
  std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::string)>
      convertScanFunctor;

 private:
  DataLoaderFileParam param_;
  std::optional<Error> initialization_error_;
};

}  // namespace open_lmm
