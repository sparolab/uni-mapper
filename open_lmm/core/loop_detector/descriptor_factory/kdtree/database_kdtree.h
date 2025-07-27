#pragma once

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <nanoflann.hpp>
#include <open_lmm/utils/config.hpp>
#include <optional>

#include "database_base.hpp"

struct KdtreeParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit KdtreeParams();
  // explicit ModelOnlineParams(Config config);
  ~KdtreeParams() = default;

 public:
  // typename DescriptorType::Params descriptor_params;
  size_t descriptor_vector_dim;
  size_t num_candidates{5};
  double distance_threshold{0.2};
  size_t kdtree_rebuild_threshold{50};
};

class DatabaseKdtree : public DatabaseBase {
 public:
  /** TYPES **/

  DatabaseKdtree() = default;
  /** Interface **/
  DatabaseKdtree(const KdtreeParams& params);

  // Copy constructor
  DatabaseKdtree(const DatabaseKdtree& other)
      : params_(other.params_),
        tree_descriptor_keys_(other.tree_descriptor_keys_),
        database_(other.database_) {
    kd_tree_ = std::make_shared<KDTree>(
        params_.descriptor_vector_dim, tree_descriptor_keys_, KDTreeParams(20));
  }

  // Copy operator
  DatabaseKdtree& operator=(const DatabaseKdtree& other) {
    if (this != &other) {
      params_ = other.params_;
      tree_descriptor_keys_ = other.tree_descriptor_keys_;
      database_ = other.database_;
      kd_tree_ =
          std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                   tree_descriptor_keys_, KDTreeParams(20));
    }
    return *this;
  }

  // Move constructor
  DatabaseKdtree(DatabaseKdtree&& other) noexcept
      : params_(std::move(other.params_)),
        tree_descriptor_keys_(std::move(other.tree_descriptor_keys_)),
        database_(std::move(other.database_)) {
    kd_tree_ = std::make_shared<KDTree>(
        params_.descriptor_vector_dim, tree_descriptor_keys_, KDTreeParams(20));
  }

  // Move assignment operator
  DatabaseKdtree& operator=(DatabaseKdtree&& other) noexcept {
    if (this != &other) {
      params_ = std::move(other.params_);
      tree_descriptor_keys_ = std::move(other.tree_descriptor_keys_);
      database_ = std::move(other.database_);
      kd_tree_ =
          std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                   tree_descriptor_keys_, KDTreeParams(20));
    }
    return *this;
  }

  size_t getSize() const { return database_.size(); }

  // TODO(gil) : need this?
  void setAgentId(const char agent_id) { agent_id_ = agent_id; }

  void merge(const DatabaseKdtree& other);

  void insert(char agent_id, size_t key,
              const std::shared_ptr<IDescriptorKdtree>& descriptor);

  std::optional<std::tuple<char, size_t, Eigen::Isometry3d>> query(
      const std::shared_ptr<IDescriptorKdtree>& query) const;

  std::vector<std::tuple<char, size_t, Eigen::Isometry3d>> queryK(
      const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const;

  /** HELPERS **/
  void tryRebuild();

  std::vector<std::pair<size_t, double>> findDescriptorKeyNeighborsSafe(
      const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const;

  static bool keyDistPairComp(const std::pair<size_t, double>& lhs,
                              const std::pair<size_t, double>& rhs) {
    return lhs.second < rhs.second;
  }

  static bool keyDistPoseTupleComp(
      const std::tuple<size_t, double, Eigen::Isometry3d>& lhs,
      const std::tuple<size_t, double, Eigen::Isometry3d>& rhs) {
    return std::get<1>(lhs) < std::get<1>(rhs);
  }

 protected:
  struct KDTreeAdaptor {
    std::vector<Eigen::VectorXd> data;
    // Interface required by nanoflann
    size_t kdtree_get_point_count() const { return data.size(); }
    double kdtree_get_pt(const size_t idx, const size_t dim) const {
      return data.at(idx)(dim);
    }
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
      return false;
    }
    KDTreeAdaptor(const std::vector<Eigen::VectorXd> data = {}) : data(data) {}
  };

  /// @brief Type for the KDTree distance metric [required by nanoflann]
  typedef nanoflann::L2_Simple_Adaptor<double, KDTreeAdaptor> KDTreeDistance;
  /// @brief Type for the KDTree using the ring size configured at runtime
  typedef nanoflann::KDTreeSingleIndexAdaptor<KDTreeDistance, KDTreeAdaptor>
      KDTree;
  typedef nanoflann::KDTreeSingleIndexAdaptorParams KDTreeParams;

  /** Fields **/
  KdtreeParams params_;
  std::shared_ptr<KDTree> kd_tree_;
  KDTreeAdaptor tree_descriptor_keys_;
  std::vector<std::tuple<char, size_t, std::shared_ptr<IDescriptorKdtree>>>
      database_;
  char agent_id_;
};