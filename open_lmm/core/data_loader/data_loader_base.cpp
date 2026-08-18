// #include "data_loader_base.hpp"
#include "data_loader_file.hpp"

namespace open_lmm {

// TODO(gil) : add DataLoaderRosbag
Result<std::unique_ptr<DataLoaderBase>> DataLoaderBase::createInstance(
    const DataLoaderConfig& config) {
  if (config.type == "file_based") {
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(
        std::make_unique<DataLoaderFile>(config));
  }
  return Result<std::unique_ptr<DataLoaderBase>>::Failure(
      Error::InvalidArgument("Unknown data_loader_type: '" + config.type +
                             "'. Supported: file_based"));
};
}  // namespace open_lmm
