#pragma once

#include <open_lmm/common/result.hpp>

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace open_lmm {

struct ConfigEditorValues {
  std::string config_map_server;
  std::string config_data_loader;
  std::string config_loop_detector;
  std::string config_backend_optimizer;
  std::string config_dynamic_remover;
  std::string root_dir_path;
  std::vector<std::string> sub_dir_list;
  std::string root_save_dir;
};

struct AlignmentConfigValues {
  double kiss_voxel_size{2.0};
  bool kiss_use_quatro{false};
  double pose_nn_distance_threshold{10.0};
};

[[nodiscard]] Result<std::vector<std::string>> DiscoverDatasetDirectories(
    const std::filesystem::path& root);
[[nodiscard]] Result<AlignmentConfigValues> LoadAlignmentConfig(
    const std::filesystem::path& path);
[[nodiscard]] Result<std::string> BuildAlignmentConfigCandidate(
    const std::filesystem::path& path,
    const AlignmentConfigValues& values);
[[nodiscard]] Result<std::string> LoadDynamicRemoverConfigCandidate(
    const std::filesystem::path& path);
Result<void> SaveAlignmentConfig(const std::filesystem::path& path,
                                 const AlignmentConfigValues& values);

class ConfigEditorDocument {
 public:
  static Result<ConfigEditorDocument> Load(const std::filesystem::path& path);
  static Result<ConfigEditorDocument> Parse(std::string text,
                                            std::filesystem::path path = {});

  [[nodiscard]] Result<ConfigEditorValues> Values() const;
  Result<void> SetValues(const ConfigEditorValues& values);
  Result<void> Save() const;
  Result<void> SaveAs(const std::filesystem::path& path) const;
  [[nodiscard]] Result<std::string> CanonicalJson() const;
  [[nodiscard]] const std::filesystem::path& Path() const { return path_; }

 private:
  ConfigEditorDocument(nlohmann::json json, std::filesystem::path path)
      : json_(std::move(json)), path_(std::move(path)) {}
  static Result<void> Validate(const nlohmann::json& json);

  nlohmann::json json_;
  std::filesystem::path path_;
};

}  // namespace open_lmm
