/** @brief This file implements the ScanContext Database interface defined in
 * database.h
 *
 *  @author Dan McGann
 *  @date March 2024
 */

#include <domain/loop_detection/database/database_kdtree.h>

#include <limits>
#include <stdexcept>
#include <string>

DatabaseKdtree::DatabaseKdtree(
    const DatabaseKdtreeParams& params,
    std::shared_ptr<IDescriptorKdtree> plugin_owner)
    : params_(params) {
  if (params_.num_candidates == 0 || params_.distance_threshold < 0.0 ||
      params_.kdtree_rebuild_threshold == 0 ||
      params_.descriptor_vector_dim == 0) {
    throw std::invalid_argument("database KD-tree parameters are out of range");
  }
  tryRebuild();
  if (plugin_owner) plugin_owners_.push_back(std::move(plugin_owner));
}

DatabaseKdtree::DatabaseKdtree(
    const DatabaseKdtreeParams& params,
    std::shared_ptr<const open_lmm::DescriptorEngine> engine)
    : DatabaseKdtree(params) {
  if (!engine) {
    throw std::invalid_argument("descriptor engine is null");
  }
  const auto& metadata = engine->IndexMetadata();
  if (metadata.plugin_id.empty() || metadata.format_id.empty() ||
      metadata.format_version == 0 ||
      metadata.index_dimension != params_.descriptor_vector_dim) {
    throw std::invalid_argument("descriptor engine index metadata mismatch");
  }
  engine_ = std::move(engine);
}

DatabaseKdtree::StorageKind DatabaseKdtree::storageKind() const {
  bool legacy = false;
  bool artifact = false;
  for (const auto& entry : database_) {
    const auto& stored = std::get<2>(entry);
    legacy |= static_cast<bool>(stored.legacy);
    artifact |= stored.artifact.has_value();
  }
  if (legacy && artifact) return StorageKind::kMixed;
  if (legacy) return StorageKind::kLegacy;
  if (artifact) return StorageKind::kArtifact;
  return StorageKind::kEmpty;
}

void DatabaseKdtree::validateArtifactIdentity(
    const open_lmm::DescriptorArtifact& artifact) const {
  if (!engine_) {
    throw std::invalid_argument("descriptor index has no comparison engine");
  }
  const auto& metadata = engine_->IndexMetadata();
  if (artifact.plugin_id() != metadata.plugin_id ||
      artifact.format_id() != metadata.format_id ||
      artifact.format_version() != metadata.format_version ||
      artifact.index_key().size() != metadata.index_dimension ||
      metadata.index_dimension != params_.descriptor_vector_dim) {
    throw std::invalid_argument("descriptor artifact index identity mismatch");
  }
}

/*********************************************************************************************************************/
void DatabaseKdtree::insert(open_lmm::AgentId agent_id, size_t key,
                            const std::shared_ptr<IDescriptorKdtree>& descriptor) {
  if (!descriptor) {
    throw std::invalid_argument("legacy descriptor is null");
  }
  if (engine_ || storageKind() == StorageKind::kArtifact ||
      storageKind() == StorageKind::kMixed) {
    throw std::invalid_argument(
        "legacy descriptor cannot be inserted into an artifact index");
  }
  if (static_cast<std::size_t>(descriptor->getDescriptorKey().size()) !=
      params_.descriptor_vector_dim) {
    throw std::invalid_argument("legacy descriptor index dimension mismatch");
  }
  // Add the new descriptor to the insertion queue
  database_.emplace_back(
      agent_id, key, StoredDescriptor{descriptor, std::nullopt});
  // Rebuild the database if necessary
  tree_descriptor_keys_.data.emplace_back(descriptor->getDescriptorKey());
  tryRebuild();
}

void DatabaseKdtree::insertArtifact(
    open_lmm::AgentId agent_id, size_t key,
    open_lmm::DescriptorArtifact artifact) {
  if (storageKind() == StorageKind::kLegacy ||
      storageKind() == StorageKind::kMixed) {
    throw std::invalid_argument(
        "descriptor artifact cannot be inserted into a legacy index");
  }
  validateArtifactIdentity(artifact);
  Eigen::VectorXd index_key(
      static_cast<Eigen::Index>(artifact.index_key().size()));
  for (std::size_t index = 0; index < artifact.index_key().size(); ++index) {
    index_key(static_cast<Eigen::Index>(index)) = artifact.index_key()[index];
  }
  tree_descriptor_keys_.data.emplace_back(std::move(index_key));
  database_.emplace_back(
      agent_id, key,
      StoredDescriptor{{}, std::optional<open_lmm::DescriptorArtifact>(
                                std::move(artifact))});
  tryRebuild();
}

// TODO(gil) : refactor query/queryK/sc-distance more readable
/*********************************************************************************************************************/
std::optional<std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>
DatabaseKdtree::query(const std::shared_ptr<IDescriptorKdtree>& query) const {
  std::vector<std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>> matches =
      queryK(query, 1);
  if (matches.size()) {
    return std::make_tuple(std::get<0>(matches[0]), std::get<1>(matches[0]),
                           std::get<2>(matches[0]));
  } else {
    return std::nullopt;
  }
}

/*********************************************************************************************************************/
std::vector<std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>> DatabaseKdtree::queryK(
    const std::shared_ptr<IDescriptorKdtree>& query, size_t k) const {
  if (!query) throw std::invalid_argument("legacy query descriptor is null");
  const auto kind = storageKind();
  if (kind == StorageKind::kArtifact || kind == StorageKind::kMixed) {
    throw std::invalid_argument(
        "legacy query cannot read an artifact descriptor index");
  }
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
    const auto& candidate = std::get<2>(database_[key]).legacy;
    if (!candidate) {
      throw std::logic_error("legacy descriptor index contains invalid storage");
    }
    const auto [distance, rel_pose] = query->distance(candidate);
    if (distance < params_.distance_threshold) {
      descriptor_neighbors.push_back(std::make_tuple(key, distance, rel_pose));
    }
  }

  // Sort all the descriptor neighbors according to their distance
  std::sort(descriptor_neighbors.begin(), descriptor_neighbors.end(),
            keyDistPoseTupleComp);

  // Accumulate the final results
  std::vector<std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>> knn;
  for (size_t i = 0; i < std::min(k, descriptor_neighbors.size()); i++) {
    const size_t global_key = std::get<0>(descriptor_neighbors[i]);
    const open_lmm::AgentId agent_id = std::get<0>(database_[global_key]);
    const size_t local_key = std::get<1>(database_[global_key]);
    const Eigen::Isometry3d init_rel_pose =
        std::get<2>(descriptor_neighbors[i]);
    knn.push_back(std::make_tuple(agent_id, local_key, init_rel_pose));
  }
  return knn;
}

open_lmm::Result<std::optional<
    std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>
DatabaseKdtree::queryArtifact(
    const open_lmm::AlgorithmExecutionContext& context,
    const open_lmm::DescriptorArtifact& query) const {
  const auto operation_context = [&] {
    auto enriched = context;
    enriched.operation = "descriptor.index.query";
    if (engine_) enriched.plugin_id = engine_->IndexMetadata().plugin_id;
    return enriched;
  }();
  if (!engine_) {
    return open_lmm::Result<std::optional<
        std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
        open_lmm::WithAlgorithmContext(
            open_lmm::Error::InvalidArgument(
                "descriptor index has no comparison engine"),
            operation_context));
  }
  try {
    const auto kind = storageKind();
    if (kind == StorageKind::kLegacy || kind == StorageKind::kMixed) {
      return open_lmm::Result<std::optional<
          std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
          open_lmm::WithAlgorithmContext(
              open_lmm::Error::InvalidArgument(
                  "artifact query cannot read a legacy descriptor index"),
              operation_context));
    }
    validateArtifactIdentity(query);
    auto active = open_lmm::CheckAlgorithmCancellation(
        operation_context, "before descriptor index query");
    if (!active) {
      return open_lmm::Result<std::optional<
          std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
          active.GetError());
    }
    const size_t candidates = std::max<std::size_t>(1, params_.num_candidates);
    auto key_neighbors =
        findDescriptorKeyNeighborsSafe(query.index_key(), candidates);
    std::optional<std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>> best;
    double best_score = std::numeric_limits<double>::infinity();
    for (const auto& [entry_index, key_distance] : key_neighbors) {
      (void)key_distance;
      const auto& candidate =
          std::get<2>(database_.at(entry_index)).artifact;
      if (!candidate) {
        return open_lmm::Result<std::optional<
            std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
            open_lmm::WithAlgorithmContext(
                open_lmm::Error::InvalidArgument(
                    "descriptor index contains incompatible storage"),
                operation_context));
      }
      auto compared = engine_->Compare(operation_context, query, *candidate);
      if (!compared) {
        return open_lmm::Result<std::optional<
            std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
            compared.GetError());
      }
      if (compared.Value().score < params_.distance_threshold &&
          compared.Value().score < best_score) {
        best_score = compared.Value().score;
        best = std::make_tuple(std::get<0>(database_[entry_index]),
                               std::get<1>(database_[entry_index]),
                               compared.Value().relative_pose);
      }
    }
    active = open_lmm::CheckAlgorithmCancellation(
        operation_context, "after descriptor index query");
    if (!active) {
      return open_lmm::Result<std::optional<
          std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
          active.GetError());
    }
    return open_lmm::Result<std::optional<
        std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Ok(
        std::move(best));
  } catch (const std::exception& error) {
    return open_lmm::Result<std::optional<
        std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
        open_lmm::WithAlgorithmContext(
            open_lmm::Error::InvalidArgument(
                std::string("descriptor index exception: ") + error.what()),
            operation_context));
  } catch (...) {
    return open_lmm::Result<std::optional<
        std::tuple<open_lmm::AgentId, size_t, Eigen::Isometry3d>>>::Failure(
        open_lmm::WithAlgorithmContext(
            open_lmm::Error::InvalidArgument(
                "unknown descriptor index exception"),
            operation_context));
  }
}

/*********************************************************************************************************************/
// TODO(gil) : refactor rebuild function
void DatabaseKdtree::tryRebuild() {
  // Do not rebuild until we have sufficient modifications [early exit]
  if ((kd_tree_ || database_.size() < params_.kdtree_rebuild_threshold) &&
      database_.size() % params_.kdtree_rebuild_threshold != 0) {
    return;
  }
  // Construct the index for the KDTree and organize the descriptorKeys
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
  indexed_size_ = database_.size();
}

/*********************************************************************************************************************/
std::vector<std::pair<size_t, double>>
DatabaseKdtree::findDescriptorKeyNeighborsSafe(const std::shared_ptr<IDescriptorKdtree>& query,
                                               size_t k) const {
  const Eigen::VectorXd& key = query->getDescriptorKey();
  std::vector<double> values(key.data(), key.data() + key.size());
  return findDescriptorKeyNeighborsSafe(values, k);
}

std::vector<std::pair<size_t, double>>
DatabaseKdtree::findDescriptorKeyNeighborsSafe(
    const std::vector<double>& query_key, size_t k) const {
  std::vector<std::pair<size_t, double>> neighbors;
  if (database_.empty() || query_key.size() !=
          params_.descriptor_vector_dim) {
    return neighbors;
  }
  if (kd_tree_ && indexed_size_ > 0) {
    const size_t num_results = std::min(k, indexed_size_);
    std::vector<size_t> knn_indices(num_results);
    std::vector<double> knn_distances_sq(num_results);
    nanoflann::KNNResultSet<double> result_set(num_results);
    result_set.init(knn_indices.data(), knn_distances_sq.data());
    kd_tree_->findNeighbors(result_set, query_key.data());
    for (size_t i = 0; i < result_set.size(); i++) {
      neighbors.emplace_back(knn_indices[i],
                             std::sqrt(knn_distances_sq[i]));
    }
  }

  // Descriptors queued since the last batched rebuild are searched directly.
  // This also covers the initial/single-point and post-clear database safely.
  Eigen::Map<const Eigen::VectorXd> query_vector(
      query_key.data(), static_cast<Eigen::Index>(query_key.size()));
  for (std::size_t index = indexed_size_;
       index < tree_descriptor_keys_.data.size(); ++index) {
    neighbors.emplace_back(
        index, (tree_descriptor_keys_.data[index] - query_vector).norm());
  }
  std::sort(neighbors.begin(), neighbors.end(), keyDistPairComp);
  if (neighbors.size() > k) neighbors.resize(k);

  return neighbors;
}

void DatabaseKdtree::merge(const DescriptorIndex& index) {
  const auto* other = dynamic_cast<const DatabaseKdtree*>(&index);
  if (!other) {
    throw std::invalid_argument("incompatible descriptor index implementation");
  }
  const auto kind = storageKind();
  const auto other_kind = other->storageKind();
  if (kind == StorageKind::kMixed || other_kind == StorageKind::kMixed) {
    throw std::invalid_argument("mixed descriptor index storage is invalid");
  }
  if (params_.descriptor_vector_dim != other->params_.descriptor_vector_dim) {
    throw std::invalid_argument("descriptor index key dimension mismatch");
  }
  const bool both_nonempty = kind != StorageKind::kEmpty &&
                             other_kind != StorageKind::kEmpty;
  if (both_nonempty && kind != other_kind) {
    throw std::invalid_argument("descriptor index storage kind mismatch");
  }
  const bool artifact_merge = kind == StorageKind::kArtifact ||
                              other_kind == StorageKind::kArtifact;
  if (artifact_merge) {
    if (!engine_ || !other->engine_ ||
        engine_->IndexMetadata() != other->engine_->IndexMetadata()) {
      throw std::invalid_argument("descriptor engine identity mismatch");
    }
    for (const auto& entry : other->database_) {
      const auto& artifact = std::get<2>(entry).artifact;
      if (!artifact) {
        throw std::invalid_argument(
            "descriptor index contains incompatible storage");
      }
      validateArtifactIdentity(*artifact);
    }
  } else if ((kind == StorageKind::kLegacy ||
              other_kind == StorageKind::kLegacy) &&
             (engine_ || other->engine_)) {
    throw std::invalid_argument("legacy descriptor index has an engine");
  }

  // Compatibility is fully validated before the first mutation. In
  // particular, a rejected merge preserves the destination database.
  database_.insert(database_.end(), other->database_.begin(),
                   other->database_.end());
  plugin_owners_.insert(plugin_owners_.end(), other->plugin_owners_.begin(),
                        other->plugin_owners_.end());
  tree_descriptor_keys_.data.insert(tree_descriptor_keys_.data.end(),
                                    other->tree_descriptor_keys_.data.begin(),
                                    other->tree_descriptor_keys_.data.end());
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
}

void DatabaseKdtree::clear() {
  database_.clear();
  tree_descriptor_keys_.data.clear();
  kd_tree_ = std::make_shared<KDTree>(params_.descriptor_vector_dim,
                                      tree_descriptor_keys_, KDTreeParams(20));
  indexed_size_ = 0;
}
