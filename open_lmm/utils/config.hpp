#pragma once

#include <any>
#include <cstddef>
#include <filesystem>
#include <open_lmm/common/result.hpp>
#include <vector>
#include <string>
#include <memory>
#include <optional>
#include <iostream>
#include <string_view>

namespace open_lmm {

/**
 * @brief Configuration loader from JSON files
 */
class Config {
public:
  /**
   * @brief Configuration JSON filename
   */
  Config(const std::string& config_filename);
  virtual ~Config();

  // Build an immutable in-memory snapshot without consulting the filesystem.
  static Config FromJson(std::string_view json_text,
                         std::string source_name = "<memory>");
  [[nodiscard]] std::string ToJson() const;

  [[nodiscard]] bool is_valid() const { return !error_message_.has_value(); }
  [[nodiscard]] const std::string& error_message() const {
    return error_message_.value();
  }

  /**
   * @brief Get the creation date of the configuration file
   * @author added by Gilhwan Kang
   */
  const std::string create_date();

  /**
   * @brief Get a parameter
   * @note  If the parameter is not found, this method aborts the program
   * @param  module_name   Module name
   * @param  param_name    Parameter name
   * @return               Returns the parameter value
   */
  template <typename T>
  T param_cast(const std::string& module_name, const std::string& param_name) const;

  /**
   * @brief Save config parameters as a JSON file
   * @param path  Destination path
   */
  void save(const std::string& path) const;

protected:
  std::any config;
  std::string config_path;
  std::optional<std::string> error_message_;
};

// Reads and parses one regular file through a single descriptor. The byte
// limit is enforced from file metadata before allocating or parsing, and the
// read is rejected if the file changes while the snapshot is being acquired.
[[nodiscard]] Result<Config> LoadConfigFileBounded(
    const std::filesystem::path& path, std::size_t maximum_bytes);


}  // namespace open_lmm
