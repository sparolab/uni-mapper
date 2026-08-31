#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <string>

namespace fs = std::filesystem;
namespace open_lmm {

struct DataLoaderInput {
  fs::path data_directory;
};

class DataLoaderBase {
 public:
  using RawScanVisitor = std::function<Result<void>(
      std::size_t, const pcl::PointCloud<pcl::PointXYZI>::Ptr&)>;

  DataLoaderBase() = default;
  virtual ~DataLoaderBase() = default;

  // Result-only production boundary.
  virtual Result<AgentRawData> Process(
      const AlgorithmExecutionContext& context,
      const DataLoaderInput& input) = 0;

  // Context-aware streaming is the only public raw-scan boundary.
  virtual Result<std::size_t> VisitRawScanData(
      const AlgorithmExecutionContext& context,
      const fs::path& data_dir_path, const RawScanVisitor& visitor,
      AlgorithmProgressPhase phase =
          AlgorithmProgressPhase::kLoadRemoverInput) = 0;

};

}  // namespace open_lmm
