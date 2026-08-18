#pragma once

#include <open_lmm/common/agent_id.hpp>

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <memory>
#include <optional>
#include <tuple>
#include <utility>
#include <vector>

// Descriptor implementations are runtime plugins, but the value and index
// contracts belong to the dependency-neutral common layer.
class IDescriptorKdtree {
 public:
  virtual ~IDescriptorKdtree() = default;
  virtual const Eigen::MatrixXd& getDescriptor() const = 0;
  virtual const Eigen::VectorXd& getDescriptorKey() const = 0;
  virtual std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const = 0;
  virtual std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) = 0;
};

class DescriptorIndex {
 public:
  virtual ~DescriptorIndex() = default;
  [[nodiscard]] virtual std::size_t getSize() const = 0;
  [[nodiscard]] virtual std::unique_ptr<DescriptorIndex> Clone() const = 0;
  virtual void clear() = 0;
  virtual void merge(const DescriptorIndex& other) = 0;
  virtual void insert(
      open_lmm::AgentId agent_id, std::size_t key,
      const std::shared_ptr<IDescriptorKdtree>& descriptor) = 0;
  [[nodiscard]] virtual std::optional<
      std::tuple<open_lmm::AgentId, std::size_t, Eigen::Isometry3d>>
  query(const std::shared_ptr<IDescriptorKdtree>& query) const = 0;
  [[nodiscard]] virtual std::vector<
      std::tuple<open_lmm::AgentId, std::size_t, Eigen::Isometry3d>>
  queryK(const std::shared_ptr<IDescriptorKdtree>& query,
         std::size_t count) const = 0;
};

using DescriptorIndexHandle = std::shared_ptr<const DescriptorIndex>;
