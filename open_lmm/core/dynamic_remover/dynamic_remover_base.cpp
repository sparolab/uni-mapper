#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"

namespace open_lmm {

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverBase::processStreaming(
    const RawScanSource& source,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
    const HeavyPhaseAdmission&) {
  ScanVec scans;
  auto loaded = source([&](std::size_t, const PointCloud::Ptr& scan) {
    scans.push_back(scan);
    return Result<void>::Ok();
  });
  if (!loaded) return Result<PointCloud::Ptr>::Failure(loaded.GetError());
  if (loaded.Value() != optimized_poses.size()) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "raw scan and optimized pose counts differ"));
  }
  return Result<PointCloud::Ptr>::Ok(process(scans, optimized_poses));
}

Result<std::shared_ptr<DynamicRemoverBase>> DynamicRemoverBase::createInstance(
    const DynamicRemoverConfig& config) {
  if (config.type == "offline") {
    auto module = DynamicRemoverOffline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOffline>(config,
                                                std::move(module).Value()));
  } else if (config.type == "online") {
    auto module = DynamicRemoverOnline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOnline>(config,
                                               std::move(module).Value()));
  }
  return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
      Error::InvalidArgument("Unknown dynamic_remover_type: '" +
          config.type + "'. Supported: offline, online"));
};

}  // namespace open_lmm
