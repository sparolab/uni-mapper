#pragma once

#include <any>
#include <open_lmm/common/result.hpp>
#include <vector>
#include <string>
#include <memory>
#include <optional>
#include <iostream>
#include <mutex>
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
   * @param  module_name Module name
   * @param  param_name  Parameter name
   * @return             Parameter value if the param is found, otherwise nullopt
   */
  template <typename T>
  std::optional<T> param(const std::string& module_name, const std::string& param_name) const;

  /**
   * @brief Get a parameter with default value
   * @param  module_name   Module name
   * @param  param_name    Parameter name
   * @param  default_value Default value
   * @return               Parameter value if the param is found, otherwise the default value
   */
  template <typename T>
  T param(const std::string& module_name, const std::string& param_name, const T& default_value) const;

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
   * @brief Get a parameter from a nested module
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @return                     Parameter value if the param is found, otherwise nullopt
   */
  template <typename T>
  std::optional<T> param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const;

  /**
   * @brief Get a parameter from a nested module with default value
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @param  default_value       Default value
   * @return                     Parameter value if the param is found, otherwise the default value
   */
  template <typename T>
  T param_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name, const T& default_value) const;

  /**
   * @brief Get a parameter
   * @note  If the parameter is not found, this method aborts the program
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @return                     Returns the parameter value
   */
  template <typename T>
  T param_cast_nested(const std::vector<std::string>& nested_module_names, const std::string& param_name) const;

  /**
   * @brief Override a parameter value
   * @note  This parameter override is volatile and does not make any changes on the JSON file
   * @param module_name  Module name
   * @param param_name   Parameter name
   * @param value        Value to override the parameter
   * @return             True if the parameter exists, otherwise false
   */
  template <typename T>
  bool override_param(const std::string& module_name, const std::string& param_name, const T& value);

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


/**
 * @brief Global configuration class to bootstrap the root path of the configuration files 
 */
class GlobalConfig : public Config {
private:
  GlobalConfig(const std::string& global_config_path) : Config(global_config_path) {}

public:
  ~GlobalConfig() override = default;
  static Result<void> reload(const std::string& config_path);
  static std::string config_directory();
  static GlobalConfig* instance(const std::string& config_path = std::string()) {
    if (inst == nullptr) {
      inst = new GlobalConfig(config_path + "/config.json");
      inst->override_param("global", "config_path", config_path);
      inst->date = inst->create_date();
    }
    return inst;
  }

  static std::string get_global_config_path(const std::string& config_name) {
    auto config = instance();
    const std::string directory = config->param<std::string>("global", "config_path", ".");
    const std::string filename = config->param<std::string>(
        "global", config_name, config_name + ".json");
    return directory + "/" + filename;
  }

  /**
   * @brief added by Gilhwan Kang
   */
  static const std::string get_root_data_dir() {
    auto config = instance();
    const std::string root_data_dir = config->param<std::string>("directory", "root_dir_path", "");
    return root_data_dir;
  }

  static const std::vector<std::string> get_sub_dir_list() {
    auto config = instance();
    const std::vector<std::string> sub_dir_list = config->param<std::vector<std::string>>("directory", "sub_dir_list", {});
    return sub_dir_list;
  }

  static const std::string get_save_dir_path() {
    auto config = instance();
    const std::string root_save_dir = config->param<std::string>("directory", "root_save_dir", "");
    const std::string date = config->date;
    return root_save_dir + "/" + date;
  }

  static GlobalConfig* inst;
  static std::mutex inst_mutex;
  std::string date;
};

}  // namespace open_lmm
