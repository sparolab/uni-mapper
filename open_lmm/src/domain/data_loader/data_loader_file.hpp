#pragma once
#include <filesystem>
#include <fstream>
#include <open_lmm/common/data_types.hpp>
#include <domain/support/pointcloud_utils.hpp>
#include <domain/support/pose_conversion.hpp>

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
      const RawScanVisitor& visitor,
      AlgorithmProgressPhase phase =
          AlgorithmProgressPhase::kLoadRemoverInput) override;
  std::function<Eigen::Isometry3d(std::vector<double>&)> transformFunctor;
  std::function<pcl::PointCloud<pcl::PointXYZI>::Ptr(std::string)>
      convertScanFunctor;

 private:
  using ScanConsumer = RawScanVisitor;
  Result<std::size_t> ForEachScanFile(
      const AlgorithmExecutionContext& context,
      const fs::path& data_dir_path,
      std::optional<std::size_t> expected_count,
      AlgorithmProgressPhase phase, bool filter,
      const ScanConsumer& consumer);
  Result<PoseVec> loadPoseData(const AlgorithmExecutionContext& context,
                               fs::path data_dir_path);
  Result<AgentRawData> Load(const AlgorithmExecutionContext& context,
                            const DataLoaderInput& input);
  DataLoaderFileParam param_;
  std::optional<Error> initialization_error_;
};

}  // namespace open_lmm
