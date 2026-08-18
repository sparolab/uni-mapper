#pragma once

#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/core/descriptor/descriptor_artifact.hpp>

#include <Eigen/Geometry>
#include <pcl/point_types.h>

#include <cstddef>
#include <cstdint>
#include <span>
#include <string>

namespace open_lmm {

struct DescriptorPointView {
  std::span<const pcl::PointXYZI> points;
};

struct DescriptorMatch {
  double score = 0.0;
  Eigen::Isometry3d relative_pose = Eigen::Isometry3d::Identity();
};

struct DescriptorIndexMetadata {
  std::string plugin_id;
  std::string format_id;
  uint32_t format_version = 0;
  std::size_t index_dimension = 0;
  std::string engine_identity;

  friend bool operator==(const DescriptorIndexMetadata&,
                         const DescriptorIndexMetadata&) = default;
};

class DescriptorEngine {
 public:
  virtual ~DescriptorEngine() = default;
  [[nodiscard]] virtual const DescriptorIndexMetadata& IndexMetadata()
      const = 0;
  virtual Result<DescriptorArtifact> Make(
      const AlgorithmExecutionContext& context,
      const DescriptorPointView& points) const = 0;
  virtual Result<DescriptorMatch> Compare(
      const AlgorithmExecutionContext& context,
      const DescriptorArtifact& lhs,
      const DescriptorArtifact& rhs) const = 0;
};

}  // namespace open_lmm
