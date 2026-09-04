#include <plugins/host/algorithm_factory.hpp>

#include <domain/data_loader/data_loader_base.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <domain/loop_detection/loop_detector_base.hpp>
#include <domain/optimization/backend_optimizer_base.hpp>
#include "support/check.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>

namespace {
using namespace open_lmm;

class NullFactory final : public AlgorithmFactory {
 protected:
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoaderImpl(
      const DataLoaderConfig&) const override {
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(nullptr);
  }
  Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetectorImpl(
      const LoopDetectorConfig&) const override {
    return Result<std::unique_ptr<LoopDetectorBase>>::Ok(nullptr);
  }
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizerImpl(
      const OptimizerConfig&) const override {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Ok(nullptr);
  }
  Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemoverImpl(
      const DynamicRemoverConfig&) const override {
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(nullptr);
  }
};

class ThrowFactory final : public AlgorithmFactory {
 protected:
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoaderImpl(
      const DataLoaderConfig&) const override {
    throw std::runtime_error("loader hook");
  }
  Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetectorImpl(
      const LoopDetectorConfig&) const override {
    throw std::runtime_error("detector hook");
  }
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizerImpl(
      const OptimizerConfig&) const override {
    throw std::runtime_error("optimizer hook");
  }
  Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemoverImpl(
      const DynamicRemoverConfig&) const override {
    throw std::runtime_error("remover hook");
  }
};

class PreflightFactory final : public AlgorithmFactory {
 public:
  mutable int descriptor_calls = 0;
  mutable int remover_calls = 0;
  bool descriptor_fails = false;

  Result<void> PreflightDescriptor(const LoopDetectorConfig&) const override {
    ++descriptor_calls;
    return descriptor_fails
               ? Result<void>::Failure(
                     Error::PluginLoadFailed("descriptor preflight fixture")
                         .WithPlugin("descriptor-fixture"))
               : Result<void>::Ok();
  }
  Result<void> PreflightRemover(const DynamicRemoverConfig&) const override {
    ++remover_calls;
    return Result<void>::Failure(
        Error::PluginLoadFailed("remover preflight fixture")
            .WithPlugin("remover-fixture"));
  }
};

void TestNullAndThrowNormalization() {
  NullFactory null_factory;
  Check(!null_factory.CreateDataLoader({}) &&
            !null_factory.CreateLoopDetector({}) &&
            !null_factory.CreateOptimizer({}) &&
            !null_factory.CreateDynamicRemover({}),
        "successful-null hooks normalize to failures for every algorithm kind");

  ThrowFactory throw_factory;
  const auto loader = throw_factory.CreateDataLoader({});
  const auto detector = throw_factory.CreateLoopDetector({});
  const auto optimizer = throw_factory.CreateOptimizer({});
  const auto remover = throw_factory.CreateDynamicRemover({});
  Check(!loader && loader.GetError().Message().find("loader hook") !=
                       std::string::npos &&
            !detector && detector.GetError().Message().find("detector hook") !=
                             std::string::npos &&
            !optimizer && optimizer.GetError().Message().find("optimizer hook") !=
                              std::string::npos &&
            !remover && remover.GetError().Message().find("remover hook") !=
                            std::string::npos,
        "hook exceptions normalize with algorithm-specific context");
}

void TestUnknownTypesAndPreflightOrdering() {
  AlgorithmFactory factory;
  DataLoaderConfig loader;
  loader.type = "unknown";
  LoopDetectorConfig detector;
  detector.type = "unknown";
  OptimizerConfig optimizer;
  optimizer.type = "unknown";
  DynamicRemoverConfig remover;
  remover.type = "unknown";
  Check(!factory.CreateDataLoader(loader) &&
            !factory.CreateLoopDetector(detector) &&
            !factory.CreateOptimizer(optimizer) &&
            !factory.CreateDynamicRemover(remover),
        "every unknown algorithm type is rejected before construction");

  PreflightFactory preflight;
  preflight.descriptor_fails = true;
  const auto descriptor_failure = preflight.Preflight(detector, remover);
  Check(!descriptor_failure && preflight.descriptor_calls == 1 &&
            preflight.remover_calls == 0 &&
            descriptor_failure.GetError().context.plugin ==
                "descriptor-fixture",
        "descriptor preflight failure short-circuits and preserves context");
  preflight.descriptor_fails = false;
  const auto remover_failure = preflight.Preflight(detector, remover);
  Check(!remover_failure && preflight.descriptor_calls == 2 &&
            preflight.remover_calls == 1 &&
            remover_failure.GetError().context.plugin == "remover-fixture",
        "successful descriptor preflight advances to remover with context");
}

}  // namespace

int main() {
  TestNullAndThrowNormalization();
  TestUnknownTypesAndPreflightOrdering();
  std::cout << "algorithm factory contract tests passed\n";
  return 0;
}
