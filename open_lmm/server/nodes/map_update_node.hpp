#pragma once
#include <functional>
#include <memory>
#include <string>

#include <open_lmm/common/pipeline.hpp>

namespace open_lmm {

class DataLoaderBase;
class DynamicRemoverBase;

class MapUpdateNode : public PipelineNodeBase {
 public:
  using RemoverFactory =
      std::function<Result<std::shared_ptr<DynamicRemoverBase>>() >;
  using HeavyPhaseAdmission =
      std::function<Result<std::shared_ptr<void>>() >;

  MapUpdateNode(std::unique_ptr<DataLoaderBase> loader,
                RemoverFactory                  remover_factory,
                const std::string&              save_dir,
                double                          save_voxel_size,
                bool                            defer_commit = false,
                HeavyPhaseAdmission heavy_phase_admission = {});
  ~MapUpdateNode() override;

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                              SharedDatabase& db) override;

  const char* Name() const override { return "MapUpdate"; }

 private:
  std::unique_ptr<DataLoaderBase>      loader_;
  RemoverFactory                       remover_factory_;
  std::string                          save_dir_;
  double                               save_voxel_size_;
  bool                                 defer_commit_;
  HeavyPhaseAdmission                  heavy_phase_admission_;
};

}  // namespace open_lmm
