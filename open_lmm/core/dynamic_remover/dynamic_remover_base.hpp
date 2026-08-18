#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <open_lmm/core/algorithm_config.hpp>
#include <string>
#include <open_lmm/utils/load_module.hpp>

namespace fs = std::filesystem;

namespace open_lmm {

class DynamicRemoverBase {
 public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZI>;
  using RawScanVisitor =
      std::function<Result<void>(std::size_t, const PointCloud::Ptr&)>;
  using RawScanSource =
      std::function<Result<std::size_t>(const RawScanVisitor&)>;
  using HeavyPhaseAdmission =
      std::function<Result<std::shared_ptr<void>>() >;

  DynamicRemoverBase() = default;
  virtual ~DynamicRemoverBase() = default;
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr
  process(std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
          std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) = 0;
  virtual Result<PointCloud::Ptr> processStreaming(
      const RawScanSource& source,
      const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
      const HeavyPhaseAdmission& heavy_phase_admission = {});
  static Result<std::shared_ptr<DynamicRemoverBase>> createInstance(
      const DynamicRemoverConfig& config);
};

}  // namespace open_lmm
