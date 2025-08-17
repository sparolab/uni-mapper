#pragma once

#include <any>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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

  /**
   * @brief Get the creation date of the configuration file
   * @author added by Gilhwan Kang
   */
  const std::string create_date();

  /**
   * @brief Get a parameter
   * @param  module_name Module name
   * @param  param_name  Parameter name
   * @return             Parameter value if the param is found, otherwise
   * nullopt
   */
  template <typename T>
  std::optional<T> param(const std::string& module_name,
                         const std::string& param_name) const;

  /**
   * @brief Get a parameter with default value
   * @param  module_name   Module name
   * @param  param_name    Parameter name
   * @param  default_value Default value
   * @return               Parameter value if the param is found, otherwise the
   * default value
   */
  template <typename T>
  T param(const std::string& module_name, const std::string& param_name,
          const T& default_value) const;

  /**
   * @brief Get a parameter
   * @note  If the parameter is not found, this method aborts the program
   * @param  module_name   Module name
   * @param  param_name    Parameter name
   * @return               Returns the parameter value
   */
  template <typename T>
  T param_cast(const std::string& module_name,
               const std::string& param_name) const;

  /**
   * @brief Get a parameter from a nested module
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @return                     Parameter value if the param is found,
   * otherwise nullopt
   */
  template <typename T>
  std::optional<T> param_nested(
      const std::vector<std::string>& nested_module_names,
      const std::string& param_name) const;

  /**
   * @brief Get a parameter from a nested module with default value
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @param  default_value       Default value
   * @return                     Parameter value if the param is found,
   * otherwise the default value
   */
  template <typename T>
  T param_nested(const std::vector<std::string>& nested_module_names,
                 const std::string& param_name, const T& default_value) const;

  /**
   * @brief Get a parameter
   * @note  If the parameter is not found, this method aborts the program
   * @param  nested_module_names Nested module names
   * @param  param_name          Parameter name
   * @return                     Returns the parameter value
   */
  template <typename T>
  T param_cast_nested(const std::vector<std::string>& nested_module_names,
                      const std::string& param_name) const;

  /**
   * @brief Override a parameter value
   * @note  This parameter override is volatile and does not make any changes on
   * the JSON file
   * @param module_name  Module name
   * @param param_name   Parameter name
   * @param value        Value to override the parameter
   * @return             True if the parameter exists, otherwise false
   */
  template <typename T>
  bool override_param(const std::string& module_name,
                      const std::string& param_name, const T& value);

  /**
   * @brief Save config parameters as a JSON file
   * @param path  Destination path
   */
  void save(const std::string& path) const;

 protected:
  std::any config;
  std::string config_path;
};

/**
 * @brief Global configuration class to bootstrap the root path of the
 * configuration files
 */
class GlobalConfig : public Config {
 private:
  GlobalConfig(const std::string& global_config_path)
      : Config(global_config_path) {}
  virtual ~GlobalConfig() override {}

 public:
  static GlobalConfig* instance(
      const std::string& config_path = std::string()) {
    if (inst == nullptr) {
      inst = new GlobalConfig(config_path + "/config.json");
      inst->override_param("global", "config_path", config_path);
      inst->date = inst->create_date();
    }
    return inst;
  }

  static std::string get_global_config_path(const std::string& config_name) {
    auto config = instance();
    const std::string directory =
        config->param<std::string>("global", "config_path", ".");
    const std::string filename = config->param<std::string>(
        "global", config_name, config_name + ".json");
    return directory + "/" + filename;
  }

  /**
   * @brief added by Gilhwan Kang
   */
  static const std::string get_root_data_dir() {
    auto config = instance();
    const std::string root_data_dir =
        config->param<std::string>("directory", "root_dir_path", "");
    return root_data_dir;
  }

  static const std::vector<std::string> get_sub_dir_list() {
    auto config = instance();
    const std::vector<std::string> sub_dir_list =
        config->param<std::vector<std::string>>("directory", "sub_dir_list",
                                                {});
    return sub_dir_list;
  }

  static const std::string get_save_dir_path() {
    auto config = instance();
    const std::string root_save_dir =
        config->param<std::string>("directory", "root_save_dir", "");
    const std::string date = config->date;
    return root_save_dir + "/" + date;
  }

  static GlobalConfig* inst;
  std::string date;
};

}  // namespace open_lmm
