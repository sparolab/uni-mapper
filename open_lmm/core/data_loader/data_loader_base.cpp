// #include "data_loader_base.hpp"
#include "data_loader_file.hpp"

namespace open_lmm {

Result<std::size_t> DataLoaderBase::VisitRawScanData(
    const fs::path& data_dir_path, const RawScanVisitor& visitor) {
  auto loaded = loadRawScanData(data_dir_path);
  if (!loaded) return Result<std::size_t>::Failure(loaded.GetError());
  auto scans = std::move(loaded).Value();
  for (std::size_t index = 0; index < scans.size(); ++index) {
    auto visited = visitor(index, scans[index]);
    if (!visited) return Result<std::size_t>::Failure(visited.GetError());
  }
  return Result<std::size_t>::Ok(scans.size());
}

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
