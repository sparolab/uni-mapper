#include "map_update_node.hpp"

#include <filesystem>
#include <utility>

#include <pcl/io/pcd_io.h>

#include <domain/support/pointcloud_utils.hpp>
#include <foundation/diagnostics/profiling.hpp>
#include <domain/data_loader/data_loader_base.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <foundation/logging/logging.hpp>

namespace open_lmm {

MapUpdateNode::MapUpdateNode(std::unique_ptr<DataLoaderBase> loader,
                             RemoverFactory remover_factory,
                             const std::string& save_dir,
                             double save_voxel_size, bool defer_commit,
                             HeavyPhaseAdmission heavy_phase_admission,
                             AlgorithmExecutionContext data_loader_context,
                             AlgorithmExecutionContext remover_context)
    : loader_(std::move(loader)),
      remover_factory_(std::move(remover_factory)),
      save_dir_(save_dir),
      save_voxel_size_(save_voxel_size),
      defer_commit_(defer_commit),
      heavy_phase_admission_(std::move(heavy_phase_admission)),
      data_loader_context_(std::move(data_loader_context)),
      remover_context_(std::move(remover_context)) {}

MapUpdateNode::~MapUpdateNode() = default;

Result<ControlFlow> MapUpdateNode::Process(AgentPipelineCtx& ctx,
                                           SharedDatabase& db) {
  OPEN_LMM_ZONE_N("MapUpdate.Process");
  auto it = db.optimized_data.find(ctx.agent.id);
  if (it == db.optimized_data.end()) {
    LogWarning("[MapUpdateNode] No optimized data for agent " +
               ctx.agent.id.Value() + ". Skipping.");
    return Result<ControlFlow>::Ok(ControlFlow::kSkip);
  }

  auto remover_result = remover_factory_();
  if (!remover_result) {
    AlgorithmExecutionContext remover_context = remover_context_;
    remover_context.agent = ctx.agent;
    remover_context.cancellation = ctx.cancellation;
    return Result<ControlFlow>::Failure(
        WithAlgorithmContext(remover_result.GetError(), remover_context));
  }
  auto dynamic_remover = std::move(remover_result).Value();
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map;
  {
    OPEN_LMM_ZONE_N("MapUpdate.RemoverProcess");
    AlgorithmExecutionContext data_loader_context = data_loader_context_;
    data_loader_context.agent = ctx.agent;
    data_loader_context.cancellation = ctx.cancellation;
    AlgorithmExecutionContext remover_context = remover_context_;
    remover_context.agent = ctx.agent;
    remover_context.cancellation = ctx.cancellation;
    const auto input_progress_phase =
        dynamic_remover->StreamingMode() ==
                DynamicRemoverStreamingMode::kDirect
            ? AlgorithmProgressPhase::kReadAndRunRemover
            : AlgorithmProgressPhase::kLoadRemoverInput;
    auto processed = dynamic_remover->ProcessStreaming(
        remover_context,
        DynamicRemoverStreamingInput{
        [&](const DynamicRemoverBase::RawScanVisitor& visitor) {
          return loader_->VisitRawScanData(
              data_loader_context, ctx.data_dir,
              [&](std::size_t index,
                  const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan) {
                if (ctx.cancellation &&
                    ctx.cancellation->IsCancellationRequested()) {
                  return Result<void>::Failure(
                      Error::Cancelled("during raw scan streaming"));
                }
                return visitor(index, scan);
              }, input_progress_phase);
        },
        it->second->optimized_poses, heavy_phase_admission_});
    if (!processed) {
      return Result<ControlFlow>::Failure(processed.GetError());
    }
    static_map = std::move(processed).Value();
  }
  if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
    AlgorithmExecutionContext remover_context = remover_context_;
    remover_context.agent = ctx.agent;
    return Result<ControlFlow>::Failure(WithAlgorithmContext(
        Error::Cancelled("after dynamic remover"), remover_context));
  }
  if (!static_map || static_map->empty()) {
    AlgorithmExecutionContext remover_context = remover_context_;
    remover_context.agent = ctx.agent;
    return Result<ControlFlow>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("Dynamic remover produced an empty map for agent " +
                               ctx.agent.id.Value()),
        remover_context));
  }

  pcl::PointCloud<pcl::PointXYZI>::Ptr ds_map;
  {
    OPEN_LMM_ZONE_N("MapUpdate.FinalDownsample");
    ds_map = downsampleWithRangeFilter(
        static_map, static_cast<float>(save_voxel_size_), 0.0f, 0.0f, false);
  }
  OPEN_LMM_PLOT("map_update.point_count", ds_map ? ds_map->size() : 0);
  if (!ds_map || ds_map->empty()) {
    return Result<ControlFlow>::Failure(Error::InvalidArgument(
        "Downsampled map is empty for agent " + ctx.agent.id.Value()));
  }

  fs::path map_file = fs::path(save_dir_) /
                      ("global_map_" + ctx.agent.id.Value() + ".pcd");
  fs::path temp_file = map_file;
  temp_file += ".tmp";
  std::error_code cleanup_error;
  fs::remove(temp_file, cleanup_error);
  int save_result = 0;
  {
    OPEN_LMM_ZONE_N("MapUpdate.PCDWrite");
    AlgorithmExecutionContext output_context = remover_context_;
    output_context.agent = ctx.agent;
    output_context.cancellation = ctx.cancellation;
    output_context.operation = "map_update_write";
    ReportAlgorithmProgress(output_context,
                            AlgorithmProgressPhase::kWriteOutput, 0);
    save_result = pcl::io::savePCDFileBinaryCompressed(temp_file, *ds_map);
  }
  if (save_result != 0) {
    fs::remove(temp_file, cleanup_error);
    return Result<ControlFlow>::Failure(Error::IoError(
        "failed to save temporary map file: " + temp_file.string()));
  }
  if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
    fs::remove(temp_file, cleanup_error);
    return Result<ControlFlow>::Failure(
        Error::Cancelled("before PCD commit"));
  }
  if (defer_commit_) {
    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }
  std::error_code rename_error;
  fs::rename(temp_file, map_file, rename_error);
  if (rename_error) {
    fs::remove(temp_file, cleanup_error);
    return Result<ControlFlow>::Failure(Error::IoError(
        "failed to commit map file: " + rename_error.message()));
  }
  return Result<ControlFlow>::Ok(ControlFlow::kContinue);
}

}  // namespace open_lmm
