#pragma once

#include <memory>

#include <open_lmm/common/result.hpp>
#include <open_lmm/core/algorithm_config.hpp>

namespace open_lmm {

class BackendOptimizerBase;
class DataLoaderBase;
class DynamicRemoverBase;
class LoopDetectorBase;

// Normalizes construction failures and successful-null results at one
// injectable boundary. Implementations override only the protected hooks.
class AlgorithmFactory {
 public:
  virtual ~AlgorithmFactory() = default;

  virtual Result<void> Preflight(const LoopDetectorConfig& loop_detector,
                                 const DynamicRemoverConfig& remover) const;
  virtual Result<void> PreflightDescriptor(
      const LoopDetectorConfig& loop_detector) const;
  virtual Result<void> PreflightRemover(
      const DynamicRemoverConfig& remover) const;
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoader(
      const DataLoaderConfig& config) const;
  Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetector(
      const LoopDetectorConfig& config) const;
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizer(
      const OptimizerConfig& config) const;
  Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemover(
      const DynamicRemoverConfig& config) const;

 protected:
  virtual Result<std::unique_ptr<DataLoaderBase>> CreateDataLoaderImpl(
      const DataLoaderConfig& config) const;
  virtual Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetectorImpl(
      const LoopDetectorConfig& config) const;
  virtual Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizerImpl(
      const OptimizerConfig& config) const;
  virtual Result<std::shared_ptr<DynamicRemoverBase>>
  CreateDynamicRemoverImpl(const DynamicRemoverConfig& config) const;
};

}  // namespace open_lmm
