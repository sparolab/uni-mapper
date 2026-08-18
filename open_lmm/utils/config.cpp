#include "config.hpp"
#include "config_impl.hpp"
#include "config_schema.hpp"
#include <open_lmm/utils/logging.hpp>

#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <sys/stat.h>
#include <unistd.h>

namespace open_lmm {

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

Result<Config> LoadConfigFileBounded(const std::filesystem::path& path,
                                     std::size_t maximum_bytes) {
  const std::string source = path.string();
  if (path.empty()) {
    return Result<Config>::Failure(
        Error::InvalidArgument("config file path must be non-empty"));
  }

  const int raw_fd = ::open(path.c_str(), O_RDONLY | O_CLOEXEC);
  if (raw_fd < 0) {
    const int error = errno;
    Error result = error == ENOENT
                       ? Error::FileNotFound(source)
                       : Error::IoError("failed to open config file " + source +
                                        ": " + std::strerror(error));
    return Result<Config>::Failure(std::move(result).WithConfig(source));
  }
  struct Descriptor {
    int value;
    ~Descriptor() { ::close(value); }
  } descriptor{raw_fd};

  struct stat before {};
  if (::fstat(descriptor.value, &before) != 0) {
    const int error = errno;
    return Result<Config>::Failure(
        Error::IoError("failed to inspect config file " + source + ": " +
                       std::strerror(error))
            .WithConfig(source));
  }
  if (!S_ISREG(before.st_mode)) {
    return Result<Config>::Failure(
        Error::InvalidArgument("config file must be a regular file: " + source)
            .WithConfig(source));
  }
  if (before.st_size < 0 ||
      static_cast<uintmax_t>(before.st_size) >
          static_cast<uintmax_t>(maximum_bytes) ||
      static_cast<uintmax_t>(before.st_size) >
          static_cast<uintmax_t>(std::numeric_limits<std::size_t>::max())) {
    return Result<Config>::Failure(
        Error::InvalidArgument("config file exceeds byte limit " +
                               std::to_string(maximum_bytes) + ": " + source)
            .WithConfig(source));
  }

  const auto expected = static_cast<std::size_t>(before.st_size);
  std::string contents(expected, '\0');
  std::size_t offset = 0;
  while (offset < expected) {
    const ssize_t count =
        ::read(descriptor.value, contents.data() + offset, expected - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count < 0) {
      const int error = errno;
      return Result<Config>::Failure(
          Error::IoError("failed to read config file " + source + ": " +
                         std::strerror(error))
              .WithConfig(source));
    }
    if (count == 0) {
      return Result<Config>::Failure(
          Error::IoError("config file became shorter while reading: " + source)
              .WithConfig(source));
    }
    offset += static_cast<std::size_t>(count);
  }

  char extra = 0;
  ssize_t extra_count;
  do {
    extra_count = ::read(descriptor.value, &extra, 1);
  } while (extra_count < 0 && errno == EINTR);
  if (extra_count < 0) {
    const int error = errno;
    return Result<Config>::Failure(
        Error::IoError("failed to finish reading config file " + source +
                       ": " + std::strerror(error))
            .WithConfig(source));
  }

  struct stat after {};
  if (::fstat(descriptor.value, &after) != 0) {
    const int error = errno;
    return Result<Config>::Failure(
        Error::IoError("failed to re-inspect config file " + source + ": " +
                       std::strerror(error))
            .WithConfig(source));
  }
  const bool metadata_changed =
      before.st_dev != after.st_dev || before.st_ino != after.st_ino ||
      before.st_size != after.st_size ||
      before.st_mtim.tv_sec != after.st_mtim.tv_sec ||
      before.st_mtim.tv_nsec != after.st_mtim.tv_nsec ||
      before.st_ctim.tv_sec != after.st_ctim.tv_sec ||
      before.st_ctim.tv_nsec != after.st_ctim.tv_nsec;
  if (extra_count != 0 || metadata_changed) {
    return Result<Config>::Failure(
        Error::IoError("config file changed while reading: " + source)
            .WithConfig(source));
  }

  Config parsed = Config::FromJson(contents, source);
  if (!parsed.is_valid()) {
    return Result<Config>::Failure(
        Error::ParseError(parsed.error_message()).WithConfig(source));
  }
  return Result<Config>::Ok(std::move(parsed));
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
