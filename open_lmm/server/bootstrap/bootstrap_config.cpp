#include "bootstrap_config.hpp"

#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/config_schema.hpp>

namespace open_lmm {
namespace {

Result<std::filesystem::path> NormalizeConfigDirectory(
    const std::filesystem::path& config_directory) {
  if (config_directory.empty()) {
    return Result<std::filesystem::path>::Failure(
        Error::InvalidArgument("config directory must be non-empty"));
  }
  std::error_code path_error;
  auto absolute_directory = std::filesystem::absolute(config_directory,
                                                        path_error);
  if (path_error) {
    return Result<std::filesystem::path>::Failure(
        Error::IoError("failed to resolve config directory " +
                       config_directory.string() + ": " +
                       path_error.message())
            .WithConfig(config_directory.string()));
  }
  absolute_directory = absolute_directory.lexically_normal();
  auto normalized_directory =
      std::filesystem::weakly_canonical(absolute_directory, path_error);
  if (path_error) {
    return Result<std::filesystem::path>::Failure(
        Error::IoError("failed to canonicalize config directory " +
                       absolute_directory.string() + ": " +
                       path_error.message())
            .WithConfig(absolute_directory.string()));
  }
  if (!std::filesystem::is_directory(normalized_directory, path_error) ||
      path_error) {
    return Result<std::filesystem::path>::Failure(
        Error::InvalidArgument("config directory is not a directory: " +
                               normalized_directory.string())
            .WithConfig(normalized_directory.string()));
  }

  return Result<std::filesystem::path>::Ok(std::move(normalized_directory));
}

}  // namespace

class BootstrapConfigSnapshotFactory {
 public:
  static Result<BootstrapConfigSnapshot> Build(
    const std::filesystem::path& normalized_directory,
    const Config& source, const std::filesystem::path& root_path) {
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kRoot, source.ToJson(), root_path.string());
  if (!validated) {
    return Result<BootstrapConfigSnapshot>::Failure(validated.GetError());
  }
  const auto& document = validated.Value().Document();
  const auto& global = document.at("global");
  const auto& directory = document.at("directory");
  const auto resolve_path = [&](const std::string& value)
      -> Result<std::filesystem::path> {
    std::filesystem::path candidate(value);
    if (candidate.is_relative()) candidate = normalized_directory / candidate;
    candidate = candidate.lexically_normal();
    std::error_code error;
    auto resolved = std::filesystem::weakly_canonical(candidate, error);
    if (error) {
      return Result<std::filesystem::path>::Failure(
          Error::IoError("failed to resolve bootstrap path " +
                         candidate.string() + ": " + error.message())
              .WithConfig(root_path.string()));
    }
    return Result<std::filesystem::path>::Ok(std::move(resolved));
  };
  const auto resolve_global = [&](const char* key) {
    return resolve_path(global.at(key).get<std::string>());
  };
  auto map_server = resolve_global("config_map_server");
  auto data_loader = resolve_global("config_data_loader");
  auto loop_detector = resolve_global("config_loop_detector");
  auto backend_optimizer = resolve_global("config_backend_optimizer");
  auto dynamic_remover = resolve_global("config_dynamic_remover");
  auto data_root =
      resolve_path(directory.at("root_dir_path").get<std::string>());
  auto output_root =
      resolve_path(directory.at("root_save_dir").get<std::string>());
  if (!map_server) {
    return Result<BootstrapConfigSnapshot>::Failure(map_server.GetError());
  }
  if (!data_loader) {
    return Result<BootstrapConfigSnapshot>::Failure(data_loader.GetError());
  }
  if (!loop_detector) {
    return Result<BootstrapConfigSnapshot>::Failure(loop_detector.GetError());
  }
  if (!backend_optimizer) {
    return Result<BootstrapConfigSnapshot>::Failure(
        backend_optimizer.GetError());
  }
  if (!dynamic_remover) {
    return Result<BootstrapConfigSnapshot>::Failure(
        dynamic_remover.GetError());
  }
  if (!data_root) {
    return Result<BootstrapConfigSnapshot>::Failure(data_root.GetError());
  }
  if (!output_root) {
    return Result<BootstrapConfigSnapshot>::Failure(output_root.GetError());
  }
  const auto subdirectories =
      directory.at("sub_dir_list").get<std::vector<std::string>>();
  return Result<BootstrapConfigSnapshot>::Ok(BootstrapConfigSnapshot(
      normalized_directory,
      std::move(validated).Value(),
      std::move(map_server).Value(),
      std::move(data_loader).Value(),
      std::move(loop_detector).Value(),
      std::move(backend_optimizer).Value(),
      std::move(dynamic_remover).Value(),
      std::move(data_root).Value(),
      subdirectories,
      std::move(output_root).Value()));
  }
};

Result<BootstrapConfigSnapshot> LoadBootstrapConfig(
    const std::filesystem::path& config_directory) {
  auto normalized = NormalizeConfigDirectory(config_directory);
  if (!normalized)
    return Result<BootstrapConfigSnapshot>::Failure(normalized.GetError());
  const auto root_path = normalized.Value() / "config.json";
  auto source = LoadConfigFileBounded(
      root_path, SchemaLimits{}.maximum_document_bytes);
  if (!source)
    return Result<BootstrapConfigSnapshot>::Failure(source.GetError());
  return BootstrapConfigSnapshotFactory::Build(
      normalized.Value(), source.Value(), root_path);
}

Result<BootstrapConfigSnapshot> LoadBootstrapConfigCandidate(
    const std::filesystem::path& config_directory,
    std::string_view root_document_json) {
  if (root_document_json.empty() ||
      root_document_json.size() > SchemaLimits{}.maximum_document_bytes) {
    return Result<BootstrapConfigSnapshot>::Failure(Error::InvalidArgument(
        "bootstrap root candidate is empty or exceeds byte limit"));
  }
  auto normalized = NormalizeConfigDirectory(config_directory);
  if (!normalized)
    return Result<BootstrapConfigSnapshot>::Failure(normalized.GetError());
  const auto root_path = normalized.Value() / "config.json";
  Config source = Config::FromJson(std::string(root_document_json),
                                   root_path.string() + ":candidate");
  if (!source.is_valid()) {
    return Result<BootstrapConfigSnapshot>::Failure(
        Error::ParseError(source.error_message()).WithConfig(
            root_path.string() + ":candidate"));
  }
  return BootstrapConfigSnapshotFactory::Build(
      normalized.Value(), source, root_path);
}

}  // namespace open_lmm
