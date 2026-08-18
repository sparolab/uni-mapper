#include "config.hpp"
#include "config_impl.hpp"
#include "config_schema.hpp"
#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

GlobalConfig* GlobalConfig::inst = nullptr;
std::mutex GlobalConfig::inst_mutex;

GlobalConfig::GlobalConfig(const std::string& global_config_path)
    : Config(global_config_path) {
  if (!is_valid()) return;
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kRoot, ToJson(), global_config_path);
  if (!validated) {
    error_message_ = validated.GetError().Message();
    return;
  }
  config = validated.Value().Document();
}

Result<void> GlobalConfig::reload(const std::string& config_path) {
  auto candidate = std::unique_ptr<GlobalConfig>(
      new GlobalConfig(config_path + "/config.json"));
  if (!candidate->is_valid()) {
    return Result<void>::Failure(Error::ParseError(candidate->error_message()));
  }
  candidate->override_param("global", "config_path", config_path);
  candidate->date = candidate->create_date();
  std::lock_guard lock(inst_mutex);
  delete inst;
  inst = candidate.release();
  return Result<void>::Ok();
}

std::string GlobalConfig::config_directory() {
  std::lock_guard lock(inst_mutex);
  if (!inst) return {};
  return inst->param<std::string>("global", "config_path", "");
}

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
    LogError(*error_message_);
  } else {
    try {
      json = nlohmann::json::parse(ifs, nullptr, true, true);
    } catch (const nlohmann::json::exception& e) {
      error_message_ = "failed to parse config file " + config_filename +
                       ": " + e.what();
      LogError(*error_message_);
    }
  }

  config = json;
}

Config Config::FromJson(std::string_view json_text, std::string source_name) {
  Config result("");
  result.config_path = std::move(source_name);
  try {
    result.config = nlohmann::json::parse(
        json_text.begin(), json_text.end(), nullptr, true, true);
  } catch (const nlohmann::json::exception& e) {
    result.error_message_ = "failed to parse config snapshot " +
                            result.config_path + ": " + e.what();
    LogError(*result.error_message_);
  }
  return result;
}

std::string Config::ToJson() const {
  return std::any_cast<const nlohmann::json&>(config).dump();
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
  if (!ofs) throw std::runtime_error("failed to open config output: " + path);
  ofs << std::setw(2) << json << std::endl;
  if (!ofs) throw std::runtime_error("failed to write config output: " + path);
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
