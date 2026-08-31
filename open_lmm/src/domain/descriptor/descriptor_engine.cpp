#include "built_in_descriptor_engine.hpp"

#include <open_lmm/common/alignment_types.hpp>
#include <domain/support/validation.hpp>

#include <cmath>
#include <exception>
#include <utility>
#include <vector>

namespace open_lmm {
namespace {

class BuiltInDescriptorPayload final : public DescriptorOpaquePayload {
 public:
  explicit BuiltInDescriptorPayload(
      std::shared_ptr<IDescriptorKdtree> descriptor,
      std::shared_ptr<IDescriptorKdtree> plugin_owner)
      : plugin_owner(std::move(plugin_owner)),
        descriptor(std::move(descriptor)) {}

  // ABI-v1 descriptors contain code-backed vtables but do not necessarily
  // retain the loader-owned prototype. Keep it beside every artifact so the
  // DSO cannot unload before the opaque descriptor is destroyed.
  std::shared_ptr<IDescriptorKdtree> plugin_owner;
  // Members are destroyed in reverse declaration order: the produced
  // descriptor dies while plugin_owner still keeps its DSO loaded.
  std::shared_ptr<IDescriptorKdtree> descriptor;
};

}  // namespace

BuiltInDescriptorEngine::BuiltInDescriptorEngine(
    std::string plugin_id, std::string format_id, uint32_t format_version,
    std::shared_ptr<IDescriptorKdtree> prototype)
    : metadata_{std::move(plugin_id), std::move(format_id), format_version,
                static_cast<std::size_t>(
                    prototype->getDescriptorKey().size())},
      prototype_(std::move(prototype)) {}

Result<std::shared_ptr<BuiltInDescriptorEngine>>
BuiltInDescriptorEngine::Create(
    std::string plugin_id, std::string format_id, uint32_t format_version,
    std::shared_ptr<IDescriptorKdtree> prototype) {
  if (plugin_id.empty() || format_id.empty() || format_version == 0 ||
      !prototype || prototype->getDescriptorKey().size() <= 0) {
    return Result<std::shared_ptr<BuiltInDescriptorEngine>>::Failure(
        Error::InvalidArgument("built-in descriptor engine identity is invalid"));
  }
  return Result<std::shared_ptr<BuiltInDescriptorEngine>>::Ok(
      std::shared_ptr<BuiltInDescriptorEngine>(new BuiltInDescriptorEngine(
          std::move(plugin_id), std::move(format_id), format_version,
          std::move(prototype))));
}

AlgorithmExecutionContext BuiltInDescriptorEngine::Context(
    const AlgorithmExecutionContext& context, const char* operation) const {
  AlgorithmExecutionContext enriched = context;
  enriched.operation = operation;
  enriched.plugin_id = metadata_.plugin_id;
  return enriched;
}

Result<DescriptorArtifact> BuiltInDescriptorEngine::Make(
    const AlgorithmExecutionContext& supplied_context,
    const DescriptorPointView& points) const {
  const auto context = Context(supplied_context, "descriptor.make");
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before descriptor make");
  if (!cancellation) {
    return Result<DescriptorArtifact>::Failure(cancellation.GetError());
  }
  try {
    if (points.points.empty()) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor point view is empty"), context));
    }
    for (std::size_t index = 0; index < points.points.size(); ++index) {
      auto valid = ValidateFinitePoint(
          points.points[index],
          "descriptor point view point " + std::to_string(index));
      if (!valid) {
        return Result<DescriptorArtifact>::Failure(
            WithAlgorithmContext(valid.GetError(), context));
      }
    }
    auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->assign(points.points.begin(), points.points.end());
    auto descriptor = prototype_->makeDescriptor(cloud);
    if (!descriptor) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor engine returned a null payload"),
          context));
    }
    const Eigen::VectorXd& key = descriptor->getDescriptorKey();
    std::vector<double> index_key(key.data(), key.data() + key.size());
    auto artifact = DescriptorArtifact::Create(
        metadata_.plugin_id, metadata_.format_id, metadata_.format_version,
        std::move(index_key),
        std::make_shared<const BuiltInDescriptorPayload>(descriptor,
                                                         prototype_));
    if (!artifact) {
      return Result<DescriptorArtifact>::Failure(
          WithAlgorithmContext(artifact.GetError(), context));
    }
    cancellation =
        CheckAlgorithmCancellation(context, "after descriptor make");
    if (!cancellation) {
      return Result<DescriptorArtifact>::Failure(cancellation.GetError());
    }
    return artifact;
  } catch (const std::exception& error) {
    return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(std::string("descriptor make exception: ") +
                               error.what()),
        context));
  } catch (...) {
    return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown descriptor make exception"), context));
  }
}

Result<DescriptorMatch> BuiltInDescriptorEngine::Compare(
    const AlgorithmExecutionContext& supplied_context,
    const DescriptorArtifact& lhs, const DescriptorArtifact& rhs) const {
  const auto context = Context(supplied_context, "descriptor.compare");
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before descriptor compare");
  if (!cancellation) {
    return Result<DescriptorMatch>::Failure(cancellation.GetError());
  }
  try {
    const auto compatible = [&](const DescriptorArtifact& artifact) {
      return artifact.plugin_id() == metadata_.plugin_id &&
             artifact.format_id() == metadata_.format_id &&
             artifact.format_version() == metadata_.format_version &&
             artifact.index_key().size() == metadata_.index_dimension;
    };
    if (!compatible(lhs) || !compatible(rhs)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor artifact identity mismatch"),
          context));
    }
    auto lhs_payload =
        std::dynamic_pointer_cast<const BuiltInDescriptorPayload>(
            lhs.opaque_payload_);
    auto rhs_payload =
        std::dynamic_pointer_cast<const BuiltInDescriptorPayload>(
            rhs.opaque_payload_);
    if (!lhs_payload || !rhs_payload || !lhs_payload->descriptor ||
        !rhs_payload->descriptor) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument(
              "descriptor artifact payload is incompatible with built-in engine"),
          context));
    }
    auto [score, relative_pose] =
        lhs_payload->descriptor->distance(rhs_payload->descriptor);
    auto valid_pose = ValidateRigidTransform(relative_pose,
                                             "descriptor match output");
    if (!std::isfinite(score) || !valid_pose) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor match output is invalid"),
          context));
    }
    cancellation =
        CheckAlgorithmCancellation(context, "after descriptor compare");
    if (!cancellation) {
      return Result<DescriptorMatch>::Failure(cancellation.GetError());
    }
    return Result<DescriptorMatch>::Ok(
        DescriptorMatch{score, std::move(relative_pose)});
  } catch (const std::exception& error) {
    return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(std::string("descriptor compare exception: ") +
                               error.what()),
        context));
  } catch (...) {
    return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown descriptor compare exception"),
        context));
  }
}

}  // namespace open_lmm
