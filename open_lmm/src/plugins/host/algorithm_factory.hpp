#pragma once

#include <plugins/host/algorithm_provider.hpp>

namespace open_lmm {

// Normalizes construction failures and successful-null results at one
// injectable boundary. Implementations override only the protected hooks.
class AlgorithmFactory : public AlgorithmProvider {
 public:
  virtual ~AlgorithmFactory() = default;

  virtual Result<void> Preflight(
      const LoopDetectorConfig& loop_detector,
      const DynamicRemoverConfig& remover) const override;
  virtual Result<void> PreflightDescriptor(
      const LoopDetectorConfig& loop_detector) const override;
  virtual Result<void> PreflightRemover(
      const DynamicRemoverConfig& remover) const override;
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoader(
      const DataLoaderConfig& config) const override;
  Result<std::unique_ptr<LoopDetectorBase>> CreateLoopDetector(
      const LoopDetectorConfig& config) const override;
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizer(
      const OptimizerConfig& config) const override;
  Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemover(
      const DynamicRemoverConfig& config) const override;

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
