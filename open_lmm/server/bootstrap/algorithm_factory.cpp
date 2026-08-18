#include "algorithm_factory.hpp"

#include <exception>
#include <string>

#include <open_lmm/core/backend_optimizer/backend_optimizer_incremental.hpp>
#include <open_lmm/core/data_loader/data_loader_file.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_offline.hpp>
#include <open_lmm/core/dynamic_remover/dynamic_remover_online.hpp>
#include <open_lmm/core/loop_detector/loop_detector_base.hpp>
#include <open_lmm/core/loop_detector/loop_detector_kdtree.hpp>
#include <open_lmm/core/loop_detector/scan_context_v2_adapter.hpp>
#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/logging.hpp>

namespace open_lmm {
namespace {

template <typename Pointer, typename Factory>
Result<Pointer> NormalizeFactoryResult(const char* name, Factory&& factory) {
  try {
    auto result = factory();
    if (!result) return Result<Pointer>::Failure(result.GetError());
    auto value = std::move(result).Value();
    if (!value) {
      return Result<Pointer>::Failure(Error::InvalidArgument(
          std::string(name) + " factory returned a null implementation"));
    }
    return Result<Pointer>::Ok(std::move(value));
  } catch (const std::exception& error) {
    return Result<Pointer>::Failure(Error::InvalidArgument(
        std::string(name) + " factory exception: " + error.what()));
  } catch (...) {
    return Result<Pointer>::Failure(Error::InvalidArgument(
        std::string(name) + " factory unknown exception"));
  }
}

}  // namespace

Result<void> AlgorithmFactory::Preflight(
    const LoopDetectorConfig& loop_detector,
    const DynamicRemoverConfig& remover) const {
  auto descriptor = PreflightDescriptor(loop_detector);
  if (!descriptor) return descriptor;
  return PreflightRemover(remover);
}

Result<void> AlgorithmFactory::PreflightDescriptor(
    const LoopDetectorConfig& loop_detector) const {
  return InspectDescriptorPlugin(loop_detector);
}

Result<void> AlgorithmFactory::PreflightRemover(
    const DynamicRemoverConfig& remover) const {
  const std::string remover_kind = remover.type == "online"
                                       ? "dynamic_remover_online"
                                       : "dynamic_remover_offline";
  auto remover_plugin =
      inspect_plugin_v1("libcreate_" + remover.model + ".so", remover_kind);
  if (!remover_plugin) {
    return Result<void>::Failure(remover_plugin.GetError());
  }
  return Result<void>::Ok();
}

Result<std::unique_ptr<DataLoaderBase>> AlgorithmFactory::CreateDataLoader(
    const DataLoaderConfig& config) const {
  return NormalizeFactoryResult<std::unique_ptr<DataLoaderBase>>(
      "data loader", [&] { return CreateDataLoaderImpl(config); });
}

Result<std::unique_ptr<DataLoaderBase>> AlgorithmFactory::CreateDataLoaderImpl(
    const DataLoaderConfig& config) const {
  if (config.type == "file_based") {
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(
        std::make_unique<DataLoaderFile>(config));
  }
  return Result<std::unique_ptr<DataLoaderBase>>::Failure(
      Error::InvalidArgument("Unknown data_loader_type: '" + config.type +
                             "'. Supported: file_based"));
}

Result<std::unique_ptr<LoopDetectorBase>> AlgorithmFactory::CreateLoopDetector(
    const LoopDetectorConfig& config) const {
  return NormalizeFactoryResult<std::unique_ptr<LoopDetectorBase>>(
      "loop detector", [&] { return CreateLoopDetectorImpl(config); });
}

Result<std::unique_ptr<LoopDetectorBase>>
AlgorithmFactory::CreateLoopDetectorImpl(
    const LoopDetectorConfig& config) const {
  if (config.type != "kdtree") {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        Error::InvalidArgument("Unknown loop_detector_type: '" + config.type +
                               "'. Supported: kdtree"));
  }
  const std::string library = "libcreate_" + config.model + ".so";
  if (config.model == "scan_context" && config.plugin_abi != "v1") {
    auto v2 = LoadScanContextV2Adapter(library, config.plugin_config_json);
    if (v2) {
      LogInfo("[plugin ABI v2] descriptor:scan_context");
      return Result<std::unique_ptr<LoopDetectorBase>>::Ok(
          std::make_unique<LoopDetectorKdtree>(config,
                                                std::move(v2).Value()));
    }
    if (config.plugin_abi == "v2") {
      return Result<std::unique_ptr<LoopDetectorBase>>::Failure(v2.GetError());
    }
    LogWarning(
        "[plugin ABI] Scan Context v2 unavailable; falling back to v1: " +
        v2.GetError().Message());
  } else if (config.plugin_abi == "v2") {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        Error::InvalidArgument("descriptor plugin '" + config.model +
                               "' does not provide an ABI-v2 adapter"));
  }
  auto module = LoopDetectorKdtree::loadModule(
      library, config.plugin_config_json);
  if (!module) {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        module.GetError());
  }
  return Result<std::unique_ptr<LoopDetectorBase>>::Ok(
      std::make_unique<LoopDetectorKdtree>(config,
                                            std::move(module).Value()));
}

Result<std::shared_ptr<BackendOptimizerBase>> AlgorithmFactory::CreateOptimizer(
    const OptimizerConfig& config) const {
  return NormalizeFactoryResult<std::shared_ptr<BackendOptimizerBase>>(
      "optimizer", [&] { return CreateOptimizerImpl(config); });
}

Result<std::shared_ptr<BackendOptimizerBase>>
AlgorithmFactory::CreateOptimizerImpl(
    const OptimizerConfig& config) const {
  if (config.type == "incremental") {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Ok(
        std::make_shared<BackendOptimizerIncremental>(config));
  }
  return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
      Error::InvalidArgument("Unknown backend_optimizer_type: '" +
                             config.type + "'. Supported: incremental"));
}

Result<std::shared_ptr<DynamicRemoverBase>>
AlgorithmFactory::CreateDynamicRemover(
    const DynamicRemoverConfig& config) const {
  return NormalizeFactoryResult<std::shared_ptr<DynamicRemoverBase>>(
      "dynamic remover", [&] { return CreateDynamicRemoverImpl(config); });
}

Result<std::shared_ptr<DynamicRemoverBase>>
AlgorithmFactory::CreateDynamicRemoverImpl(
    const DynamicRemoverConfig& config) const {
  if (config.type == "offline") {
    auto module = DynamicRemoverOffline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) {
      return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
          module.GetError());
    }
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOffline>(config,
                                                std::move(module).Value()));
  }
  if (config.type == "online") {
    auto module = DynamicRemoverOnline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) {
      return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
          module.GetError());
    }
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOnline>(config,
                                               std::move(module).Value()));
  }
  return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
      Error::InvalidArgument("Unknown dynamic_remover_type: '" + config.type +
                             "'. Supported: offline, online"));
}

}  // namespace open_lmm
