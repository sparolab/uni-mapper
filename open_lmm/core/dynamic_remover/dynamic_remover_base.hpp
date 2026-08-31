#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <string>
#include <open_lmm/utils/load_module.hpp>

namespace fs = std::filesystem;

namespace open_lmm {

struct DynamicRemoverInput {
  std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans;
  std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses;
};

using DynamicRemoverPointCloud = pcl::PointCloud<pcl::PointXYZI>;
using DynamicRemoverRawScanVisitor = std::function<Result<void>(
    std::size_t, const DynamicRemoverPointCloud::Ptr&)>;
using DynamicRemoverRawScanSource =
    std::function<Result<std::size_t>(const DynamicRemoverRawScanVisitor&)>;
using DynamicRemoverHeavyPhaseAdmission =
    std::function<Result<std::shared_ptr<void>>() >;

struct DynamicRemoverStreamingInput {
  const DynamicRemoverRawScanSource& source;
  const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses;
  DynamicRemoverHeavyPhaseAdmission heavy_phase_admission;
};

enum class DynamicRemoverStreamingMode : uint8_t {
  kBuffered,
  kDirect,
};

class DynamicRemoverBase {
 public:
  using PointCloud = DynamicRemoverPointCloud;
  using RawScanVisitor = DynamicRemoverRawScanVisitor;
  using RawScanSource = DynamicRemoverRawScanSource;
  using HeavyPhaseAdmission = DynamicRemoverHeavyPhaseAdmission;

  DynamicRemoverBase() = default;
  virtual ~DynamicRemoverBase() = default;
  virtual Result<PointCloud::Ptr> Process(
      const AlgorithmExecutionContext& context,
      DynamicRemoverInput input) = 0;
  virtual Result<PointCloud::Ptr> ProcessStreaming(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input);
  [[nodiscard]] virtual DynamicRemoverStreamingMode StreamingMode() const {
    return DynamicRemoverStreamingMode::kBuffered;
  }
};

}  // namespace open_lmm
