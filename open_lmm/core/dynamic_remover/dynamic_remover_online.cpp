#include "dynamic_remover_online.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tqdmcpp/tqdmcpp.hpp>

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
    auto output = ProcessImpl(std::move(input.scans),
                              std::move(input.optimized_poses));
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

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOnline::ProcessImpl(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  auto ordered_result = OrderOptimizedPosesByFrameId(
      optimized_poses, scans.size(), "online dynamic remover");
  if (!ordered_result) {
    throw std::invalid_argument(ordered_result.GetError().Message());
  }
  PoseVec ordered_poses = std::move(ordered_result).Value();
  for (std::size_t index = 0; index < scans.size(); ++index) {
    auto valid = ValidatePointCloud(
        scans[index], "online dynamic remover scan " + std::to_string(index));
    if (!valid) throw std::invalid_argument(valid.GetError().Message());
  }
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_scan(
      new pcl::PointCloud<pcl::PointXYZI>);

  auto T = tq::tqdm(scans);
  T.set_prefix("Dynamic Remover");
  int idx = 0;
  for (auto scan : T) {
    static_scan = online_model_->run(scans[idx], ordered_poses[idx]);
    idx++;
  }
  T.finish();

  LogInfo("[dynamic_remover] building static map from " +
          std::to_string(scans.size()) + " scans");
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      online_model_->getStaticMap();

  // TODO(gil) : hardcoded voxel leaf size
  static_map = downsampleWithRangeFilter(static_map, 0.2F, 0.0F, 0.0F, false);

  return static_map;
}

Result<std::shared_ptr<IOnlineRemoverPlugin>> DynamicRemoverOnline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOnlineRemoverPlugin>(
      so_name, "dynamic_remover_online", config_json);
}

}  // namespace open_lmm
