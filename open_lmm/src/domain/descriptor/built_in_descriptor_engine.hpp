#pragma once

#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/core/descriptor/descriptor_engine.hpp>

#include <cstdint>
#include <memory>
#include <string>

namespace open_lmm {

// Wraps ABI-v1 descriptor objects. Artifacts retain the descriptor object and
// therefore its plugin/library lifetime.
class BuiltInDescriptorEngine final : public DescriptorEngine {
 public:
  static Result<std::shared_ptr<BuiltInDescriptorEngine>> Create(
      std::string plugin_id, std::string format_id, uint32_t format_version,
      std::shared_ptr<IDescriptorKdtree> prototype);

  [[nodiscard]] const DescriptorIndexMetadata& IndexMetadata()
      const override {
    return metadata_;
  }

  Result<DescriptorArtifact> Make(
      const AlgorithmExecutionContext& context,
      const DescriptorPointView& points) const override;
  Result<DescriptorMatch> Compare(
      const AlgorithmExecutionContext& context,
      const DescriptorArtifact& lhs,
      const DescriptorArtifact& rhs) const override;

 private:
  BuiltInDescriptorEngine(std::string plugin_id, std::string format_id,
                          uint32_t format_version,
                          std::shared_ptr<IDescriptorKdtree> prototype);
  AlgorithmExecutionContext Context(
      const AlgorithmExecutionContext& context,
      const char* operation) const;

  DescriptorIndexMetadata metadata_;
  std::shared_ptr<IDescriptorKdtree> prototype_;
};

}  // namespace open_lmm
