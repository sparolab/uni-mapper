#include <adapters/gui/config_editor.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <fstream>
#include <sstream>
#include <algorithm>

namespace open_lmm {

Result<std::vector<std::string>> DiscoverDatasetDirectories(
    const std::filesystem::path& root) {
  std::error_code error;
  if (!std::filesystem::is_directory(root, error)) {
    return Result<std::vector<std::string>>::Failure(
        Error::InvalidArgument("dataset root is not a directory: " +
                               root.string()));
  }
  std::vector<std::string> directories;
  for (std::filesystem::directory_iterator iterator(root, error), end;
       !error && iterator != end; iterator.increment(error)) {
    if (iterator->is_directory(error) && !error)
      directories.push_back(iterator->path().filename().string());
  }
  if (error) {
    return Result<std::vector<std::string>>::Failure(
        Error::IoError(error.message()));
  }
  std::sort(directories.begin(), directories.end());
  return Result<std::vector<std::string>>::Ok(std::move(directories));
}

Result<AlignmentConfigValues> LoadAlignmentConfig(
    const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) return Result<AlignmentConfigValues>::Failure(
      Error::FileNotFound(path.string()));
  std::ostringstream contents;
  contents << input.rdbuf();
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kLoopDetector, contents.str(), path.string());
  if (!validated)
    return Result<AlignmentConfigValues>::Failure(validated.GetError());
  const auto& alignment = validated.Value().Document().at("alignment");
  return Result<AlignmentConfigValues>::Ok(
      {alignment.at("kiss_voxel_size").get<double>(),
       alignment.at("kiss_use_quatro").get<bool>(),
       alignment.at("pose_nn_distance_threshold").get<double>(),
       alignment.at("inter_loop_keyframe_spacing_m").get<double>()});
}

Result<void> SaveAlignmentConfig(const std::filesystem::path& path,
                                 const AlignmentConfigValues& values) {
  auto candidate = BuildAlignmentConfigCandidate(path, values);
  if (!candidate) return Result<void>::Failure(candidate.GetError());
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) return Result<void>::Failure(Error::IoError(temporary));
    output << candidate.Value() << '\n';
    if (!output) return Result<void>::Failure(Error::IoError(temporary));
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    return Result<void>::Failure(Error::IoError(error.message()));
  }
  return Result<void>::Ok();
}

Result<std::string> BuildAlignmentConfigCandidate(
    const std::filesystem::path& path,
    const AlignmentConfigValues& values) {
  std::ifstream input(path);
  if (!input) {
    return Result<std::string>::Failure(Error::FileNotFound(path.string()));
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  auto current = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kLoopDetector, contents.str(), path.string());
  if (!current) return Result<std::string>::Failure(current.GetError());
  auto candidate = current.Value().Document();
  candidate["alignment"]["kiss_voxel_size"] = values.kiss_voxel_size;
  candidate["alignment"]["kiss_use_quatro"] = values.kiss_use_quatro;
  candidate["alignment"]["pose_nn_distance_threshold"] =
      values.pose_nn_distance_threshold;
  candidate["alignment"]["inter_loop_keyframe_spacing_m"] =
      values.inter_loop_keyframe_spacing_m;
  auto validated = BuiltinConfigSchemaRegistry().Validate(
      ConfigDocumentKind::kLoopDetector, candidate, path.string());
  if (!validated) {
    return Result<std::string>::Failure(validated.GetError());
  }
  return Result<std::string>::Ok(validated.Value().CanonicalJson(2));
}

Result<std::string> LoadDynamicRemoverConfigCandidate(
    const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return Result<std::string>::Failure(Error::FileNotFound(path.string()));
  }
  std::ostringstream contents;
  contents << input.rdbuf();
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kDynamicRemover, contents.str(), path.string());
  if (!validated) return Result<std::string>::Failure(validated.GetError());
  return Result<std::string>::Ok(validated.Value().CanonicalJson(2));
}

Result<ConfigEditorDocument> ConfigEditorDocument::Load(
    const std::filesystem::path& path) {
  std::ifstream input(path);
  if (!input) {
    return Result<ConfigEditorDocument>::Failure(Error::FileNotFound(path.string()));
  }
  std::ostringstream text;
  text << input.rdbuf();
  return Parse(text.str(), path);
}

Result<ConfigEditorDocument> ConfigEditorDocument::Parse(
    std::string text, std::filesystem::path path) {
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kRoot, text,
      path.empty() ? "<editor>" : path.string());
  if (!validated)
    return Result<ConfigEditorDocument>::Failure(validated.GetError());
  return Result<ConfigEditorDocument>::Ok(ConfigEditorDocument(
      validated.Value().Document(), std::move(path)));
}

Result<void> ConfigEditorDocument::Validate(const nlohmann::json& json) {
  auto validated = BuiltinConfigSchemaRegistry().Validate(
      ConfigDocumentKind::kRoot, json, "<editor>");
  if (!validated) return Result<void>::Failure(validated.GetError());
  return Result<void>::Ok();
}

Result<ConfigEditorValues> ConfigEditorDocument::Values() const {
  auto valid = Validate(json_);
  if (!valid) return Result<ConfigEditorValues>::Failure(valid.GetError());
  ConfigEditorValues values;
  values.config_map_server = json_["global"]["config_map_server"];
  values.config_data_loader = json_["global"]["config_data_loader"];
  values.config_loop_detector = json_["global"]["config_loop_detector"];
  values.config_backend_optimizer = json_["global"]["config_backend_optimizer"];
  values.config_dynamic_remover = json_["global"]["config_dynamic_remover"];
  values.root_dir_path = json_["directory"]["root_dir_path"];
  values.sub_dir_list =
      json_["directory"]["sub_dir_list"].get<std::vector<std::string>>();
  values.root_save_dir = json_["directory"]["root_save_dir"];
  return Result<ConfigEditorValues>::Ok(std::move(values));
}

Result<void> ConfigEditorDocument::SetValues(const ConfigEditorValues& values) {
  auto candidate = json_;
  candidate["global"]["config_map_server"] = values.config_map_server;
  candidate["global"]["config_data_loader"] = values.config_data_loader;
  candidate["global"]["config_loop_detector"] = values.config_loop_detector;
  candidate["global"]["config_backend_optimizer"] = values.config_backend_optimizer;
  candidate["global"]["config_dynamic_remover"] = values.config_dynamic_remover;
  candidate["directory"]["root_dir_path"] = values.root_dir_path;
  candidate["directory"]["sub_dir_list"] = values.sub_dir_list;
  candidate["directory"]["root_save_dir"] = values.root_save_dir;
  auto valid = Validate(candidate);
  if (!valid) return valid;
  json_ = std::move(candidate);
  return Result<void>::Ok();
}

Result<void> ConfigEditorDocument::Save() const {
  if (path_.empty()) {
    return Result<void>::Failure(Error::InvalidArgument("config path is empty"));
  }
  return SaveAs(path_);
}

Result<void> ConfigEditorDocument::SaveAs(
    const std::filesystem::path& path) const {
  auto validated = BuiltinConfigSchemaRegistry().Validate(
      ConfigDocumentKind::kRoot, json_, path.string());
  if (!validated) return Result<void>::Failure(validated.GetError());
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) return Result<void>::Failure(Error::IoError(temporary));
    output << validated.Value().CanonicalJson(2) << '\n';
    if (!output) return Result<void>::Failure(Error::IoError(temporary));
  }
  std::error_code error;
  std::filesystem::rename(temporary, path, error);
  if (error) {
    std::filesystem::remove(temporary);
    return Result<void>::Failure(Error::IoError(error.message()));
  }
  return Result<void>::Ok();
}

Result<std::string> ConfigEditorDocument::CanonicalJson() const {
  auto validated = BuiltinConfigSchemaRegistry().Validate(
      ConfigDocumentKind::kRoot, json_,
      path_.empty() ? "<editor>" : path_.string());
  if (!validated) return Result<std::string>::Failure(validated.GetError());
  return Result<std::string>::Ok(validated.Value().CanonicalJson(2));
}

}  // namespace open_lmm
