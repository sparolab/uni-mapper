#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"

#include <open_lmm/common/validation.hpp>

#include <algorithm>
#include <set>

namespace open_lmm {

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverBase::processStreaming(
    const RawScanSource& source,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
    const HeavyPhaseAdmission&) {
  std::vector<std::pair<std::size_t, PointCloud::Ptr>> indexed_scans;
  std::set<std::size_t> seen;
  auto loaded = source([&](std::size_t index, const PointCloud::Ptr& scan) {
    if (!seen.insert(index).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "dynamic remover source returned duplicate frame " +
          std::to_string(index)));
    }
    auto valid = ValidatePointCloud(
        scan, "dynamic remover source frame " + std::to_string(index));
    if (!valid) return valid;
    indexed_scans.emplace_back(index, scan);
    return Result<void>::Ok();
  });
  if (!loaded) return Result<PointCloud::Ptr>::Failure(loaded.GetError());
  if (loaded.Value() != indexed_scans.size()) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "dynamic remover source count differs from visited scan count"));
  }
  std::sort(indexed_scans.begin(), indexed_scans.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first < rhs.first;
            });
  ScanVec scans;
  scans.reserve(indexed_scans.size());
  for (std::size_t expected = 0; expected < indexed_scans.size(); ++expected) {
    if (indexed_scans[expected].first != expected) {
      return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
          "dynamic remover source is missing frame " +
          std::to_string(expected)));
    }
    scans.push_back(std::move(indexed_scans[expected].second));
  }
  try {
    return Result<PointCloud::Ptr>::Ok(process(scans, optimized_poses));
  } catch (const std::exception& error) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        std::string("dynamic remover process failed: ") + error.what()));
  } catch (...) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "dynamic remover process failed with an unknown exception"));
  }
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
