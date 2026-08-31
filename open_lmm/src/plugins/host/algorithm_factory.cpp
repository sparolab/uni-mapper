#include "algorithm_factory.hpp"

#include <exception>
#include <string>

#include <domain/optimization/backend_optimizer_incremental.hpp>
#include <domain/data_loader/data_loader_file.hpp>
#include <domain/dynamic_removal/dynamic_remover_offline.hpp>
#include <domain/dynamic_removal/dynamic_remover_online.hpp>
#include <domain/loop_detection/loop_detector_base.hpp>
#include <domain/loop_detection/loop_detector_kdtree.hpp>
#include <domain/descriptor/built_in_descriptor_engine.hpp>
#include <plugins/host/load_module.hpp>
#include <foundation/logging/logging.hpp>

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

Result<std::unique_ptr<LoopDetectorBase>> MakeLoopDetector(
    const LoopDetectorConfig& config,
    std::shared_ptr<IDescriptorKdtree> descriptor) {
  auto engine = BuiltInDescriptorEngine::Create(
      config.model, config.model, 1, descriptor);
  if (!engine) {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        engine.GetError());
  }
  return Result<std::unique_ptr<LoopDetectorBase>>::Ok(
      std::make_unique<LoopDetectorKdtree>(
          config, std::move(engine).Value()));
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
  auto module = LoopDetectorKdtree::loadModule(
      library, config.plugin_config_json);
  if (!module) {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        module.GetError());
  }
  return MakeLoopDetector(config, std::move(module).Value());
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
