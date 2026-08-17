#include "config.hpp"
#include "config_impl.hpp"

namespace open_lmm {

GlobalConfig* GlobalConfig::inst = nullptr;

Config::Config(const std::string& config_filename)
: config_path(config_filename)
{
  nlohmann::json json;
  if (config_filename.empty()) {
    config = json;
    return;
  }
  std::ifstream ifs(config_filename);
  if (!ifs) {
    error_message_ = "failed to open config file: " + config_filename;
    spdlog::error("{}", *error_message_);
  } else {
    try {
      json = nlohmann::json::parse(ifs, nullptr, true, true);
    } catch (const nlohmann::json::exception& e) {
      error_message_ = "failed to parse config file " + config_filename +
                       ": " + e.what();
      spdlog::error("{}", *error_message_);
    }
  }

  config = json;
}

const std::string Config::create_date() {
  time_t t = time(nullptr);
  const tm* local_time = localtime(&t);
  std::stringstream s;
  s << "20" << local_time->tm_year - 100 << "_";
  s << local_time->tm_mon + 1 << "_";
  s << local_time->tm_mday << "_";
  s << local_time->tm_hour << "_";
  s << local_time->tm_min << "_";
  s << local_time->tm_sec;
  return (s.str());
}

Config::~Config() {}

void Config::save(const std::string& path) const {
  const auto& json = std::any_cast<const nlohmann::json&>(config);

  std::ofstream ofs(path);
  ofs << std::setw(2) << json << std::endl;
}

DEFINE_CONFIG_IO_SPECIALIZATION(bool)
DEFINE_CONFIG_IO_SPECIALIZATION(int)
DEFINE_CONFIG_IO_SPECIALIZATION(float)
DEFINE_CONFIG_IO_SPECIALIZATION(double)
DEFINE_CONFIG_IO_SPECIALIZATION(std::string)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<bool>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<int>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<double>)
DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<std::string>)

DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector2d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector3d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Vector4d)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Quaterniond)
DEFINE_CONFIG_IO_SPECIALIZATION(Eigen::Isometry3d)

DEFINE_CONFIG_IO_SPECIALIZATION(std::vector<Eigen::Isometry3d>)

}  // namespace open_lmm
