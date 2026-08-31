#include "dynamic_remover_online.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/core/algorithm_invariants.hpp>
#include <open_lmm/utils/logging.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>

namespace open_lmm {

DynamicRemoverOnline::DynamicRemoverOnline(
    const OnlineParams& params, std::shared_ptr<IOnlineRemoverPlugin> model)
    : params_(params), online_model_(std::move(model)) {}

Result<DynamicRemoverBase::PointCloud::Ptr> DynamicRemoverOnline::Process(
    const AlgorithmExecutionContext& context, DynamicRemoverInput input) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation = CheckAlgorithmCancellation(context, "before map removal");
  if (!cancellation) {
    return Result<PointCloud::Ptr>::Failure(cancellation.GetError());
  }
  try {
    auto processed = ProcessImpl(context, std::move(input.scans),
                                 std::move(input.optimized_poses));
    if (!processed) return processed;
    auto output = std::move(processed).Value();
    if (!output) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("dynamic remover returned a null point cloud"),
          context));
    }
    cancellation = CheckAlgorithmCancellation(context, "after map removal");
    if (!cancellation) {
      return Result<PointCloud::Ptr>::Failure(cancellation.GetError());
    }
    return Result<PointCloud::Ptr>::Ok(std::move(output));
  } catch (const std::exception& error) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(std::string("dynamic remover exception: ") +
                               error.what()),
        context));
  } catch (...) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown dynamic remover exception"), context));
  }
}

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverOnline::ProcessImpl(
    const AlgorithmExecutionContext& context,
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  auto ordered_result = OrderOptimizedPosesByFrameId(
      optimized_poses, scans.size(), "online dynamic remover");
  if (!ordered_result) {
    return Result<PointCloud::Ptr>::Failure(ordered_result.GetError());
  }
  PoseVec ordered_poses = std::move(ordered_result).Value();
  for (std::size_t index = 0; index < scans.size(); ++index) {
    auto valid = ValidatePointCloud(
        scans[index], "online dynamic remover scan " + std::to_string(index));
    if (!valid) return Result<PointCloud::Ptr>::Failure(valid.GetError());
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_scan(
      new pcl::PointCloud<pcl::PointXYZI>);

  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kRunRemover, 0,
                          scans.size());
  for (std::size_t idx = 0; idx < scans.size(); ++idx) {
    auto active = CheckAlgorithmCancellation(
        context, "while running online dynamic remover");
    if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
    static_scan = online_model_->run(scans[idx], ordered_poses[idx]);
    ReportAlgorithmProgress(context, AlgorithmProgressPhase::kRunRemover,
                            idx + 1, scans.size());
  }

  LogInfo("[dynamic_remover] building static map from " +
          std::to_string(scans.size()) + " scans");
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kBuildStaticMap,
                          0);
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      online_model_->getStaticMap();

  // TODO(gil) : hardcoded voxel leaf size
  static_map = downsampleWithRangeFilter(static_map, 0.2F, 0.0F, 0.0F, false);

  return Result<PointCloud::Ptr>::Ok(std::move(static_map));
}

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverOnline::ProcessStreaming(
    const AlgorithmExecutionContext& context,
    const DynamicRemoverStreamingInput& input) {
  AlgorithmExecutionTimer timer(context);
  auto active =
      CheckAlgorithmCancellation(context, "before online map removal");
  if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
  try {
    auto ordered = OrderOptimizedPosesByFrameId(
        input.optimized_poses, input.optimized_poses.size(),
        "online streaming dynamic remover");
    if (!ordered) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(ordered.GetError(), context));
    }
    PoseVec poses = std::move(ordered).Value();
    std::size_t visited = 0;
    auto loaded = input.source(
        [&](std::size_t index, const PointCloud::Ptr& source_scan) {
          auto current = CheckAlgorithmCancellation(
              context, "while streaming online map-removal input");
          if (!current) return current;
          if (index != visited) {
            return Result<void>::Failure(Error::InvalidArgument(
                "online dynamic remover source expected frame " +
                std::to_string(visited) + ", got " +
                std::to_string(index)));
          }
          if (index >= poses.size()) {
            return Result<void>::Failure(Error::InvalidArgument(
                "online dynamic remover source has more scans than poses"));
          }
          auto valid = ValidatePointCloud(
              source_scan, "online dynamic remover source frame " +
                               std::to_string(index));
          if (!valid) return valid;
          PointCloud::Ptr scan = source_scan;
          Eigen::Isometry3d pose = poses[index];
          (void)online_model_->run(scan, pose);
          ++visited;
          return Result<void>::Ok();
        });
    if (!loaded) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(loaded.GetError(), context));
    }
    if (loaded.Value() != visited || visited != poses.size()) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument(
              "online dynamic remover scan/pose count mismatch: " +
              std::to_string(visited) + " scans, " +
              std::to_string(poses.size()) + " poses"),
          context));
    }
    ReportAlgorithmProgress(context, AlgorithmProgressPhase::kBuildStaticMap,
                            0);
    auto static_map = online_model_->getStaticMap();
    auto valid = ValidatePointCloud(static_map, "online remover output");
    if (!valid) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(valid.GetError(), context));
    }
    static_map = downsampleWithRangeFilter(
        static_map, 0.2F, 0.0F, 0.0F, false);
    return Result<PointCloud::Ptr>::Ok(std::move(static_map));
  } catch (const std::exception& error) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(
            std::string("online dynamic remover streaming exception: ") +
            error.what()),
        context));
  } catch (...) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(
            "unknown online dynamic remover streaming exception"),
        context));
  }
}

Result<std::shared_ptr<IOnlineRemoverPlugin>> DynamicRemoverOnline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOnlineRemoverPlugin>(
      so_name, "dynamic_remover_online", config_json);
}

}  // namespace open_lmm
