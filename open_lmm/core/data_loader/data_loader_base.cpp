// #include "data_loader_base.hpp"
#include "data_loader_file.hpp"

namespace open_lmm {

// TODO(gil) : add DataLoaderRosbag
Result<std::unique_ptr<DataLoaderBase>> DataLoaderBase::createInstance(Config config) {
  std::string data_loader_type =
      config.param<std::string>("data_loader", "data_loader_type", "");
  if (data_loader_type == "file_based") {
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(
        std::make_unique<DataLoaderFile>(config));
  }
  return Result<std::unique_ptr<DataLoaderBase>>::Failure(
      Error::InvalidArgument("Unknown data_loader_type: '" + data_loader_type +
                             "'. Supported: file_based"));
};
}  // namespace open_lmm
