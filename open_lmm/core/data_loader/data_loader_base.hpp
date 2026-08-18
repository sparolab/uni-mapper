#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <string>

namespace fs = std::filesystem;
namespace open_lmm {

class DataLoaderBase {
 public:
  using RawScanVisitor = std::function<Result<void>(
      std::size_t, const pcl::PointCloud<pcl::PointXYZI>::Ptr&)>;

  DataLoaderBase() = default;
  virtual ~DataLoaderBase() = default;

  // 순수 함수 — SharedDatabase 없이 AgentRawData 반환
  virtual Result<AgentRawData> Process(const AgentContext& ctx,
                                       const fs::path& data_dir) = 0;

  // MapUpdater가 raw 스캔만 다시 읽을 때 사용
  virtual Result<ScanVec> loadRawScanData(fs::path data_dir_path) = 0;
  virtual Result<std::size_t> VisitRawScanData(
      const fs::path& data_dir_path, const RawScanVisitor& visitor);

  static Result<std::unique_ptr<DataLoaderBase>> createInstance(
      const DataLoaderConfig& config);
};

}  // namespace open_lmm
