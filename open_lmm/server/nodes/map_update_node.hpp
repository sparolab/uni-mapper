#pragma once
#include <filesystem>
#include <functional>
#include <memory>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

namespace open_lmm {

class MapUpdateNode : public PipelineNodeBase {
 public:
  using RemoverFactory = std::function<std::shared_ptr<DynamicRemoverBase>()>;

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

    auto raw_scans = loader_->loadRawScanData(ctx.data_dir);
    auto dynamic_remover = remover_factory_();
    auto static_map = dynamic_remover->process(raw_scans, it->second.optimized_poses);

    // voxel downsampling
    float vs = static_cast<float>(save_voxel_size_);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ds_map(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    vg.setInputCloud(static_map);
    vg.setLeafSize(vs, vs, vs);
    vg.filter(*ds_map);

    fs::path map_file = fs::path(save_dir_) /
                        ("global_map_" + std::string{ctx.agent.id} + ".pcd");
    pcl::io::savePCDFileBinaryCompressed(map_file, *ds_map);

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
