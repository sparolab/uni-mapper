#pragma once
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/validation.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

namespace open_lmm {

class MapUpdateNode : public PipelineNodeBase {
 public:
  using RemoverFactory =
      std::function<Result<std::shared_ptr<DynamicRemoverBase>>() >;

  MapUpdateNode(std::unique_ptr<DataLoaderBase> loader,
                RemoverFactory                  remover_factory,
                const std::string&              save_dir,
                double                          save_voxel_size)
      : loader_(std::move(loader)),
        remover_factory_(std::move(remover_factory)),
        save_dir_(save_dir),
        save_voxel_size_(save_voxel_size) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    auto it = db.optimized_data.find(ctx.agent.id);
    if (it == db.optimized_data.end()) {
      spdlog::warn("[MapUpdateNode] No optimized data for agent {}. Skipping.",
                   ctx.agent.id);
      return Result<ControlFlow>::Ok(ControlFlow::kSkip);
    }

    auto raw_result = loader_->loadRawScanData(ctx.data_dir);
    if (!raw_result) return Result<ControlFlow>::Failure(raw_result.GetError());
    auto raw_scans = std::move(raw_result).Value();
    auto count_result = ValidateScanPoseCount(
        raw_scans.size(), it->second.optimized_poses.size(), ctx.data_dir.string());
    if (!count_result) return Result<ControlFlow>::Failure(count_result.GetError());
    auto remover_result = remover_factory_();
    if (!remover_result) {
      return Result<ControlFlow>::Failure(remover_result.GetError());
    }
    auto dynamic_remover = std::move(remover_result).Value();
    auto static_map = dynamic_remover->process(raw_scans, it->second.optimized_poses);

    auto ds_map = downsampleWithRangeFilter(
        static_map, static_cast<float>(save_voxel_size_), 0.0f, 0.0f, false);

    fs::path map_file = fs::path(save_dir_) /
                        ("global_map_" + std::string{ctx.agent.id} + ".pcd");
    if (pcl::io::savePCDFileBinaryCompressed(map_file, *ds_map) != 0) {
      return Result<ControlFlow>::Failure(Error::InvalidArgument(
          "Failed to save map file: " + map_file.string()));
    }

    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "MapUpdate"; }

 private:
  std::unique_ptr<DataLoaderBase>      loader_;
  RemoverFactory                       remover_factory_;
  std::string                          save_dir_;
  double                               save_voxel_size_;
};

}  // namespace open_lmm
