#include "descriptor_artifact.hpp"

#include <cmath>
#include <utility>

namespace open_lmm {

DescriptorArtifact::DescriptorArtifact(
    std::string plugin_id, std::string format_id, uint32_t format_version,
    std::vector<double> index_key,
    std::shared_ptr<const DescriptorOpaquePayload> payload)
    : plugin_id_(std::move(plugin_id)),
      format_id_(std::move(format_id)),
      format_version_(format_version),
      index_key_(std::move(index_key)),
      opaque_payload_(std::move(payload)) {}

Result<DescriptorArtifact> DescriptorArtifact::Create(
    std::string plugin_id, std::string format_id, uint32_t format_version,
    std::vector<double> index_key,
    std::shared_ptr<const DescriptorOpaquePayload> opaque_payload) {
  if (plugin_id.empty()) {
    return Result<DescriptorArtifact>::Failure(
        Error::InvalidArgument("descriptor artifact plugin_id is empty"));
  }
  if (format_id.empty() || format_version == 0) {
    return Result<DescriptorArtifact>::Failure(Error::InvalidArgument(
        "descriptor artifact format identity is invalid"));
  }
  if (index_key.empty()) {
    return Result<DescriptorArtifact>::Failure(
        Error::InvalidArgument("descriptor artifact index_key is empty"));
  }
  for (double value : index_key) {
    if (!std::isfinite(value)) {
      return Result<DescriptorArtifact>::Failure(Error::InvalidArgument(
          "descriptor artifact index_key contains a non-finite value"));
    }
  }
  if (!opaque_payload) {
    return Result<DescriptorArtifact>::Failure(
        Error::InvalidArgument("descriptor artifact payload is null"));
  }
  return Result<DescriptorArtifact>::Ok(DescriptorArtifact(
      std::move(plugin_id), std::move(format_id), format_version,
      std::move(index_key), std::move(opaque_payload)));
}

}  // namespace open_lmm
