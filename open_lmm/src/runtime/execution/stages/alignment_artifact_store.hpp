#pragma once

#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <runtime/state/artifact_repository.hpp>
#include <storage/transactions/output_repository.hpp>
#include <runtime/state/runtime_state.hpp>

namespace open_lmm {

struct AlignmentArtifactIdentityInput {
  std::string data_loader_config;
  std::string loop_detector_config;
  std::string optimizer_config;
  int anchor_agent_index = 0;
  std::vector<AgentId> agents;
  std::vector<std::filesystem::path> data_directories;
  std::string pose_file_name;
  std::string scan_directory_name;
  std::string scan_extension;
  std::filesystem::path cache_root;
};

struct AlignmentArtifactIdentity {
  std::string config_fingerprint;
  std::map<AgentId, std::string> input_fingerprints;
  std::string runtime_fingerprint;
  std::filesystem::path cache_path;
};

// Owns alignment cache identity and loaded approvals. It never commits files;
// all output remains part of the caller's runtime transaction.
class AlignmentArtifactStore {
 public:
  static Result<AlignmentArtifactStore> Open(
      const AlignmentArtifactIdentityInput& input,
      AgentSymbolCatalogHandle catalog);
  // Rehydrates an ephemeral writer from the committed authority. Coordinators
  // must not retain this object as a second mutable runtime/config mirror.
  static Result<AlignmentArtifactStore> FromCommitted(
      const RuntimeState& committed);

  [[nodiscard]] const AlignmentArtifactIdentity& Identity() const;
  [[nodiscard]] const std::map<AgentId, StoredAlignment>& Cached() const;
  void InstallInto(SharedDatabase& database) const;

  Result<void> Prepare(const RuntimeState& state,
                       const std::filesystem::path& output_directory,
                       PendingOutputSet& pending,
                       ArtifactRepository& artifacts) const;

 private:
  AlignmentArtifactIdentity identity_;
  std::map<AgentId, StoredAlignment> cached_;
};

}  // namespace open_lmm
