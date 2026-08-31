#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <open_lmm/utils/config_schema.hpp>

namespace open_lmm {

class BootstrapConfigSnapshotFactory;

// Immutable, fully validated root configuration used to construct one runtime
// runtime. It has no process-global identity and never reloads itself.
class BootstrapConfigSnapshot {
 public:
  [[nodiscard]] const std::filesystem::path& ConfigDirectory() const {
    return config_directory_;
  }
  [[nodiscard]] const ValidatedConfigDocument& Root() const { return root_; }
  [[nodiscard]] const std::filesystem::path& MapServerConfig() const {
    return map_server_config_;
  }
  [[nodiscard]] const std::filesystem::path& DataLoaderConfig() const {
    return data_loader_config_;
  }
  [[nodiscard]] const std::filesystem::path& LoopDetectorConfig() const {
    return loop_detector_config_;
  }
  [[nodiscard]] const std::filesystem::path& BackendOptimizerConfig() const {
    return backend_optimizer_config_;
  }
  [[nodiscard]] const std::filesystem::path& DynamicRemoverConfig() const {
    return dynamic_remover_config_;
  }
  [[nodiscard]] const std::filesystem::path& DataRoot() const {
    return data_root_;
  }
  [[nodiscard]] const std::vector<std::string>& DataSubdirectories() const {
    return data_subdirectories_;
  }
  [[nodiscard]] const std::filesystem::path& OutputRoot() const {
    return output_root_;
  }

 private:
  friend class BootstrapConfigSnapshotFactory;
  friend Result<BootstrapConfigSnapshot> LoadBootstrapConfig(
      const std::filesystem::path& config_directory);
  friend Result<BootstrapConfigSnapshot> LoadBootstrapConfigCandidate(
      const std::filesystem::path& config_directory,
      std::string_view root_document_json);

  BootstrapConfigSnapshot(
      std::filesystem::path config_directory, ValidatedConfigDocument root,
      std::filesystem::path map_server_config,
      std::filesystem::path data_loader_config,
      std::filesystem::path loop_detector_config,
      std::filesystem::path backend_optimizer_config,
      std::filesystem::path dynamic_remover_config,
      std::filesystem::path data_root,
      std::vector<std::string> data_subdirectories,
      std::filesystem::path output_root)
      : config_directory_(std::move(config_directory)),
        root_(std::move(root)),
        map_server_config_(std::move(map_server_config)),
        data_loader_config_(std::move(data_loader_config)),
        loop_detector_config_(std::move(loop_detector_config)),
        backend_optimizer_config_(std::move(backend_optimizer_config)),
        dynamic_remover_config_(std::move(dynamic_remover_config)),
        data_root_(std::move(data_root)),
        data_subdirectories_(std::move(data_subdirectories)),
        output_root_(std::move(output_root)) {}

  std::filesystem::path config_directory_;
  ValidatedConfigDocument root_;
  std::filesystem::path map_server_config_;
  std::filesystem::path data_loader_config_;
  std::filesystem::path loop_detector_config_;
  std::filesystem::path backend_optimizer_config_;
  std::filesystem::path dynamic_remover_config_;
  std::filesystem::path data_root_;
  std::vector<std::string> data_subdirectories_;
  std::filesystem::path output_root_;
};

[[nodiscard]] Result<BootstrapConfigSnapshot> LoadBootstrapConfig(
    const std::filesystem::path& config_directory);

// Builds a runtime bootstrap snapshot from a bounded in-memory root candidate.
// Module documents referenced by the root remain immutable file snapshots
// loaded by RuntimeBootstrapper. This API lets GUI/API callers replace a
// runtime without writing config.json before validation succeeds.
[[nodiscard]] Result<BootstrapConfigSnapshot> LoadBootstrapConfigCandidate(
    const std::filesystem::path& config_directory,
    std::string_view root_document_json);

}  // namespace open_lmm
