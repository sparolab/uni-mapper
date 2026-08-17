// #include "data_loader_base.hpp"
#include "data_loader_file.hpp"
#include <spdlog/spdlog.h>

namespace open_lmm {

// TODO(gil) : add DataLoaderRosbag
std::unique_ptr<DataLoaderBase> DataLoaderBase::createInstance(Config config) {
  std::string data_loader_type =
      config.param<std::string>("data_loader", "data_loader_type", "");
  if (data_loader_type == "file_based") {
    return std::make_unique<DataLoaderFile>(config);
  }
  spdlog::error("[DataLoaderBase] Unknown data_loader_type: '{}'. "
                "Supported: file_based", data_loader_type);
  std::exit(1);
};
}  // namespace open_lmm