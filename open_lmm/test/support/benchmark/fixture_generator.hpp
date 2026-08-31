#pragma once

#include "fixture_manifest.hpp"

#include <filesystem>
#include <string>

#include <nlohmann/json.hpp>

namespace open_lmm::test::benchmark {

struct GeneratedFixture {
  std::filesystem::path root;
  std::filesystem::path config_directory;
  std::filesystem::path data_directory;
  std::filesystem::path output_directory;
  std::filesystem::path manifest_path;
  nlohmann::json manifest;
};

struct FixtureGenerationOptions {
  bool enable_map_update = false;
  bool parallel_map_update = false;
};

// Creates a complete fixture under a path that must not exist. The generated
// config uses fixture-root-relative data/output paths, so file content and
// hashes do not depend on the caller's temporary directory.
GeneratedFixture GenerateFixture(const std::filesystem::path& new_root,
                                 const std::string& fixture_id,
                                 FixtureGenerationOptions options = {});
ValidationResult VerifyMaterializedFixture(
    const std::filesystem::path& root, const nlohmann::json& manifest);

}  // namespace open_lmm::test::benchmark
