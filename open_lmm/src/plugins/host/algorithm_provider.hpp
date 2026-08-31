#pragma once

#include <memory>

#include <open_lmm/common/result.hpp>
#include <config/domain/algorithm_config.hpp>

namespace open_lmm {

class BackendOptimizerBase;
class DataLoaderBase;
class DynamicRemoverBase;
class LoopDetectorBase;

// Private runtime port for algorithm construction and plugin preflight.
// Execution and reconfiguration depend on this contract; only composition
// owns a concrete plugin-aware implementation.
class AlgorithmProvider {
 public:
  virtual ~AlgorithmProvider() = default;

  virtual Result<void> Preflight(
      const LoopDetectorConfig& loop_detector,
      const DynamicRemoverConfig& remover) const = 0;
  virtual Result<void> PreflightDescriptor(
      const LoopDetectorConfig& loop_detector) const = 0;
  virtual Result<void> PreflightRemover(
      const DynamicRemoverConfig& remover) const = 0;
  virtual Result<std::unique_ptr<DataLoaderBase>> CreateDataLoader(
      const DataLoaderConfig& config) const = 0;
  virtual Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetector(
      const LoopDetectorConfig& config) const = 0;
  virtual Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizer(
      const OptimizerConfig& config) const = 0;
  virtual Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemover(
      const DynamicRemoverConfig& config) const = 0;
};

}  // namespace open_lmm
