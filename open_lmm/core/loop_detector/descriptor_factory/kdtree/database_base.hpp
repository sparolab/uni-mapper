#pragma once

#include <open_lmm/utils/config.hpp>
// #include <string>
// #include <memory>
// #include <Eigen/Core>
#include <Eigen/Geometry>
#include "interface_descriptor_kdtree.hpp"



class DatabaseBase
{
public:
  DatabaseBase() = default;
  // explicit DatabaseBase(openConfig config);
  virtual ~DatabaseBase() = default;
  virtual size_t getSize() const = 0;
  virtual void insert(char agent_id, size_t key, const std::shared_ptr<IDescriptorKdtree>& descriptor) = 0;
  virtual std::optional<std::tuple<char, size_t, Eigen::Isometry3d>> query(const std::shared_ptr<IDescriptorKdtree>& query) const = 0;
  virtual std::vector<std::tuple<char, size_t, Eigen::Isometry3d>> queryK(const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const = 0;
};