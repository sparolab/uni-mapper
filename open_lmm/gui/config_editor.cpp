#include <open_lmm/gui/config_editor.hpp>

#include <fstream>
#include <sstream>

namespace open_lmm {
namespace {
Result<std::string> RequiredString(const nlohmann::json& json,
                                   const char* section, const char* key) {
  if (!json.contains(section) || !json[section].is_object() ||
      !json[section].contains(key) || !json[section][key].is_string()) {
    return Result<std::string>::Failure(Error::ParseError(
        std::string(section) + "/" + key + " must be a string"));
  }
  auto value = json[section][key].get<std::string>();
  if (value.empty()) {
    return Result<std::string>::Failure(Error::InvalidArgument(
        std::string(section) + "/" + key + " must be non-empty"));
  }
  return Result<std::string>::Ok(std::move(value));
}
}  // namespace

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
  try {
    auto json = nlohmann::json::parse(text, nullptr, true, true);
    auto valid = Validate(json);
    if (!valid) return Result<ConfigEditorDocument>::Failure(valid.GetError());
    return Result<ConfigEditorDocument>::Ok(
        ConfigEditorDocument(std::move(json), std::move(path)));
  } catch (const nlohmann::json::exception& e) {
    return Result<ConfigEditorDocument>::Failure(Error::ParseError(e.what()));
  }
}

Result<void> ConfigEditorDocument::Validate(const nlohmann::json& json) {
  for (const char* key : {"config_map_server", "config_data_loader",
                          "config_loop_detector", "config_backend_optimizer",
                          "config_dynamic_remover"}) {
    auto value = RequiredString(json, "global", key);
    if (!value) return Result<void>::Failure(value.GetError());
  }
  for (const char* key : {"root_dir_path", "root_save_dir"}) {
    auto value = RequiredString(json, "directory", key);
    if (!value) return Result<void>::Failure(value.GetError());
  }
  if (!json["directory"].contains("sub_dir_list") ||
      !json["directory"]["sub_dir_list"].is_array() ||
      json["directory"]["sub_dir_list"].empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "directory/sub_dir_list must be a non-empty string array"));
  }
  for (const auto& item : json["directory"]["sub_dir_list"]) {
    if (!item.is_string() || item.get<std::string>().empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "directory/sub_dir_list must contain non-empty strings"));
    }
  }
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
  auto valid = Validate(json_);
  if (!valid) return valid;
  const auto temporary = path.string() + ".tmp";
  {
    std::ofstream output(temporary, std::ios::trunc);
    if (!output) return Result<void>::Failure(Error::IoError(temporary));
    output << json_.dump(2) << '\n';
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

}  // namespace open_lmm
