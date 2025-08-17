/** @brief This file implements the ScanContext Database interface defined in
 * database.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */

#include "database_kdtree.h"

KdtreeParams::KdtreeParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_loop_detector"));
  num_candidates = config.param<int>("database", "num_candidates", 5);
  distance_threshold =
      config.param<double>("database", "distance_threshold", 0.13);
  kdtree_rebuild_threshold =
      config.param<int>("database", "rebuild_threshold", 50);
  descriptor_vector_dim =
      config.param<int>("database", "descriptor_vector_dim", 128);
}

DatabaseKdtree::DatabaseKdtree(const KdtreeParams& params) : params_(params) {
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
}

/*********************************************************************************************************************/
void DatabaseKdtree::insert(
    char agent_id, size_t key,
    const std::shared_ptr<IDescriptorKdtree>& descriptor) {
  // Guard Code
  // if (!params_.descriptor_params.equals(descriptor.params())) {
  //   throw std::invalid_argument(
  //       "DescriptorDatabase::insert provided descriptor with mis-matched
  //       parameters to database");
  // }
  // Add the new descriptor to the insertion queue
  database_.emplace_back(std::make_tuple(agent_id, key, descriptor));
  // Rebuild the database if necessary
  tree_descriptor_keys_.data.emplace_back(descriptor->getDescriptorKey());
  tryRebuild();
}

// TODO(gil) : refactor query/queryK/sc-distance more readable
/*********************************************************************************************************************/
std::optional<std::tuple<char, size_t, Eigen::Isometry3d>>
DatabaseKdtree::query(const std::shared_ptr<IDescriptorKdtree>& query) const {
  std::vector<std::tuple<char, size_t, Eigen::Isometry3d>> matches =
      queryK(query, 1);
  if (matches.size()) {
    return std::make_tuple(std::get<0>(matches[0]), std::get<1>(matches[0]),
                           std::get<2>(matches[0]));
  } else {
    return std::nullopt;
  }
}

/*********************************************************************************************************************/
std::vector<std::tuple<char, size_t, Eigen::Isometry3d>> DatabaseKdtree::queryK(
    const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const {
  // Determine the number of descriptor-key nearest neighbors to retrieve
  size_t number_rink_key_nn = std::max(k, params_.num_candidates);

  // First find the K nearest rink key neighbors accounting for the removal
  // queue
  std::vector<std::pair<size_t, double>> descriptor_key_neighbors =
      findDescriptorKeyNeighborsSafe(query, number_rink_key_nn);

  // Sort all the descriptor key neighbors according to their distance
  std::sort(descriptor_key_neighbors.begin(), descriptor_key_neighbors.end(),
            keyDistPairComp);
  // For the top number_rink_key_nn compute their ScanContext Distance
  std::vector<std::tuple<size_t, double, Eigen::Isometry3d>>
      descriptor_neighbors;
  for (size_t i = 0;
       i < std::min(number_rink_key_nn, descriptor_key_neighbors.size()); i++) {
    const size_t key = descriptor_key_neighbors[i].first;
    const std::shared_ptr<IDescriptorKdtree> candidate =
        std::get<2>(database_[key]);
    const auto [distance, rel_pose] = query->distance(candidate);
    if (distance < params_.distance_threshold) {
      descriptor_neighbors.push_back(std::make_tuple(key, distance, rel_pose));
    }
  }

  // Sort all the descriptor neighbors according to their distance
  std::sort(descriptor_neighbors.begin(), descriptor_neighbors.end(),
            keyDistPoseTupleComp);

  // Accumulate the final results
  std::vector<std::tuple<char, size_t, Eigen::Isometry3d>> knn;
  for (size_t i = 0; i < std::min(k, descriptor_neighbors.size()); i++) {
    const size_t global_key = std::get<0>(descriptor_neighbors[i]);
    const char agent_id = std::get<0>(database_[global_key]);
    const size_t local_key = std::get<1>(database_[global_key]);
    const Eigen::Isometry3d init_rel_pose =
        std::get<2>(descriptor_neighbors[i]);
    knn.push_back(std::make_tuple(agent_id, local_key, init_rel_pose));
  }
  return knn;
}

/*********************************************************************************************************************/
// TODO(gil) : refactor rebuild function
void DatabaseKdtree::tryRebuild() {
  // Do not rebuild until we have sufficient modifications [early exit]
  if (database_.size() % params_.kdtree_rebuild_threshold != 0) return;
  // Construct the index for the KDTree and organize the descriptorKeys
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
}

/*********************************************************************************************************************/
std::vector<std::pair<size_t, double>>
DatabaseKdtree::findDescriptorKeyNeighborsSafe(
    const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const {
  std::vector<std::pair<size_t, double>> neighbors;
  size_t num_results = k;
  // Setup the KNN search
  std::vector<size_t> knn_indicies(num_results);
  std::vector<double> knn_distances_sq(num_results);
  nanoflann::KNNResultSet<double> result_set(num_results);
  result_set.init(&knn_indicies[0], &knn_distances_sq[0]);

  // Do the KNN search
  kd_tree_->findNeighbors(result_set, &query->getDescriptorKey()[0]);
  // kd_tree_->findNeighbors();
  // Compose the results accounting for removed keys
  for (size_t i = 0; i < result_set.size(); i++) {
    size_t key = knn_indicies[i];
    neighbors.push_back(std::make_pair(key, std::sqrt(knn_distances_sq[i])));
  }

  return neighbors;
}

void DatabaseKdtree::merge(const DatabaseKdtree& other) {
  database_.insert(database_.end(), other.database_.begin(),
                   other.database_.end());
  tree_descriptor_keys_.data.insert(tree_descriptor_keys_.data.end(),
                                    other.tree_descriptor_keys_.data.begin(),
                                    other.tree_descriptor_keys_.data.end());
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
}
