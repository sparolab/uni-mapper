// #include "data_loader_base.hpp"
#include "data_loader_file.hpp"

namespace open_lmm {

// TODO(gil) : add DataLoaderRosbag
std::unique_ptr<DataLoaderBase> DataLoaderBase::createInstance(Config config) {
  std::string data_loader_type =
      config.param<std::string>("data_loader", "data_loader_type", "");
  if (data_loader_type == "file_based") {
    return std::make_unique<DataLoaderFile>(config);
  } else {
    throw std::invalid_argument(
        "[data_loader_base.cpp] Invalid data loader type: " + data_loader_type);
  }
};
}  // namespace open_lmm