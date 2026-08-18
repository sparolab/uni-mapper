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
  using DataLoaderBase::Process;
  explicit DataLoaderFile(DataLoaderConfig config);
  ~DataLoaderFile() override = default;
  Result<AgentRawData> Process(
      const AlgorithmExecutionContext& context,
      const DataLoaderInput& input) override;
  Result<std::size_t> VisitRawScanData(
      const AlgorithmExecutionContext& context,
      const fs::path& data_dir_path,
      const RawScanVisitor& visitor) override;
  std::function<Eigen::Isometry3d(std::vector<double>&)> transformFunctor;
  std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::string)>
      convertScanFunctor;

 private:
  Result<ScanVec> LoadRawScans(fs::path data_dir_path);
  Result<PoseVec> loadPoseData(const AlgorithmExecutionContext& context,
                               fs::path data_dir_path);
  Result<ScanVec> loadFilteredScanData(
      const AlgorithmExecutionContext& context, fs::path data_dir_path);
  Result<AgentRawData> Load(const AlgorithmExecutionContext& context,
                            const fs::path& data_directory);
  DataLoaderFileParam param_;
  std::optional<Error> initialization_error_;
};

}  // namespace open_lmm
