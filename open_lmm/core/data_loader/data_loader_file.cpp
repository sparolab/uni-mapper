#include "data_loader_file.hpp"

#include <open_lmm/common/validation.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/core/algorithm_invariants.hpp>

namespace fs = std::filesystem;
namespace open_lmm {

DataLoaderFile::DataLoaderFile(DataLoaderConfig config)
    : param_(std::move(config)) {

  if (param_.pose_file_name.empty() || param_.scan_dir_name.empty()) {
    initialization_error_ = Error::InvalidArgument(
        "data_loader pose_file_name and scan_dir_name must be non-empty");
  } else if (param_.voxel_size <= 0.0f) {
    initialization_error_ = Error::InvalidArgument(
        "data_loader voxel_size must be greater than zero");
  } else if (param_.min_range < 0.0f || param_.max_range <= param_.min_range) {
    initialization_error_ = Error::InvalidArgument(
        "data_loader range must satisfy 0 <= min_range < max_range");
  } else if (param_.delimiter.size() != 1) {
    initialization_error_ = Error::InvalidArgument(
        "data_loader delimiter must be exactly one character");
  }

  if (param_.pose_format == "kitti") {
    transformFunctor = kittiPoseToIsometry3d;
  } else if (param_.pose_format == "tum") {
    transformFunctor = tumPoseToIsometry3d;
  } else if (param_.pose_format == "custom") {
    initialization_error_ = Error::InvalidArgument(
        "pose format 'custom' is not implemented");
  } else {
    initialization_error_ = Error::InvalidArgument(
        "Unsupported pose format: '" + param_.pose_format +
        "'. Supported: kitti, tum");
  }

  if (param_.scan_type == "pcd") {
    convertScanFunctor = readPointsFromPCD;
  } else if (param_.scan_type == "bin") {
    convertScanFunctor = readPointsFromBin;
  } else {
    initialization_error_ = Error::InvalidArgument(
        "Unsupported scan type: '" + param_.scan_type +
        "'. Supported: pcd, bin");
  }
}

Result<AgentRawData> DataLoaderFile::Process(
    const AlgorithmExecutionContext& context, const DataLoaderInput& input) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation = CheckAlgorithmCancellation(context, "before data load");
  if (!cancellation) {
    return Result<AgentRawData>::Failure(cancellation.GetError());
  }
  try {
    auto result = Load(context, input.data_directory);
    if (!result) {
      return Result<AgentRawData>::Failure(
          WithAlgorithmContext(result.GetError(), context));
    }
    cancellation = CheckAlgorithmCancellation(context, "after data load");
    if (!cancellation) {
      return Result<AgentRawData>::Failure(cancellation.GetError());
    }
    return result;
  } catch (const std::exception& error) {
    return Result<AgentRawData>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(std::string("data loader exception: ") +
                               error.what()),
        context));
  } catch (...) {
    return Result<AgentRawData>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("unknown data loader exception"), context));
  }
}

Result<AgentRawData> DataLoaderFile::Load(
    const AlgorithmExecutionContext& context, const fs::path& data_dir) {
  OPEN_LMM_ZONE_N("DataLoader.Process");
  if (initialization_error_) {
    return Result<AgentRawData>::Failure(*initialization_error_);
  }
  auto poses_result = loadPoseData(context, data_dir);
  if (!poses_result) return Result<AgentRawData>::Failure(poses_result.GetError());
  auto poses = std::move(poses_result).Value();
  ScanVec filtered_scans;
  filtered_scans.reserve(poses.size());
  auto scans_result = ForEachScanFile(
      context, data_dir, poses.size(), AlgorithmProgressPhase::kReadAndFilter,
      true,
      [&](std::size_t, const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan) {
        filtered_scans.push_back(scan);
        return Result<void>::Ok();
      });
  if (!scans_result) {
    return Result<AgentRawData>::Failure(scans_result.GetError());
  }
  OPEN_LMM_PLOT("data_loader.pose_count", poses.size());
  OPEN_LMM_PLOT("data_loader.scan_count", filtered_scans.size());
  auto count_result = ValidateScanPoseCount(filtered_scans.size(), poses.size(),
                                             data_dir.string());
  if (!count_result) return Result<AgentRawData>::Failure(count_result.GetError());

  AgentRawData output{
      .agent_id       = context.agent.id,
      .odom_poses     = std::move(poses),
      .filtered_scans = std::move(filtered_scans),
  };
  auto valid_output = ValidateAgentRawData(
      output, "DataLoader agent '" + context.agent.id.Value() + "'");
  if (!valid_output) {
    return Result<AgentRawData>::Failure(valid_output.GetError());
  }
  return Result<AgentRawData>::Ok(std::move(output));
}

Result<PoseVec> DataLoaderFile::loadPoseData(
    const AlgorithmExecutionContext& context, fs::path data_dir_path) {
  OPEN_LMM_ZONE_N("DataLoader.LoadPoses");
  const fs::path pose_file_path = data_dir_path / param_.pose_file_name;
  if (!fs::exists(pose_file_path)) {
    return Result<PoseVec>::Failure(Error::FileNotFound(pose_file_path.string()));
  }

  std::ifstream file(pose_file_path);
  char delimiter;
  if (param_.delimiter.size() != 1) {
    return Result<PoseVec>::Failure(Error::InvalidArgument(
        "Delimiter must be exactly one character, got: '" + param_.delimiter + "'"));
  } else {
    delimiter = param_.delimiter[0];
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    lines.push_back(line);
  }

  PoseVec poses;

  std::size_t line_number = 0;
  for (auto line : lines) {
    auto cancellation =
        CheckAlgorithmCancellation(context, "while loading poses");
    if (!cancellation) {
      return Result<PoseVec>::Failure(cancellation.GetError());
    }
    ++line_number;
    std::istringstream iss(line);
    std::vector<double> values;
    std::string value;
    try {
      while (std::getline(iss, value, delimiter)) {
        if (!value.empty()) values.push_back(std::stod(value));
      }
      const std::size_t expected_values =
          param_.pose_format == "kitti" ? 12U :
          param_.pose_format == "tum" ? 8U : values.size();
      if (values.size() != expected_values) {
        return Result<PoseVec>::Failure(Error::ParseError(
            pose_file_path.string() + ":" + std::to_string(line_number) +
            ": expected " + std::to_string(expected_values) +
            " values, got " + std::to_string(values.size())));
      }
      poses.push_back(param_.extrinsic * transformFunctor(values));
    } catch (const std::exception& e) {
      return Result<PoseVec>::Failure(Error::ParseError(
          pose_file_path.string() + ":" + std::to_string(line_number) +
          ": " + e.what()));
    }
  }

  return Result<PoseVec>::Ok(std::move(poses));
}

// const fs::path data_dir_name = data_dir_path.filename();
// const fs::path scan_save_dir =
//     configured_save_dir / data_dir_name / param_.scan_dir_name;
// fs::create_directories(scan_save_dir);
// const fs::path file_name = scan_file.filename();
// const fs::path save_file_path = scan_save_dir / file_name;
// filtered_scans.push_back(p_scan_preprocessed);
// pcl::io::savePCDFileBinaryCompressed(save_file_path, *p_scan_preprocessed);

Result<std::size_t> DataLoaderFile::ForEachScanFile(
    const AlgorithmExecutionContext& context,
    const fs::path& data_dir_path,
    std::optional<std::size_t> expected_count,
    AlgorithmProgressPhase phase, bool filter,
    const ScanConsumer& consumer) {
  OPEN_LMM_ZONE_N("DataLoader.ForEachScanFile");
  const fs::path scan_dir_path = data_dir_path / param_.scan_dir_name;
  if (!fs::exists(scan_dir_path) || !fs::is_directory(scan_dir_path)) {
    return Result<std::size_t>::Failure(
        Error::FileNotFound(scan_dir_path.string()));
  }

  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kEnumerate, 0);
  std::vector<fs::path> scan_files;
  for (const auto& scan_file : fs::directory_iterator(scan_dir_path)) {
    const auto extension = scan_file.path().extension().string();
    if (scan_file.is_regular_file() && extension.size() > 1 &&
        extension.substr(1) == param_.scan_type) {
      scan_files.push_back(scan_file.path());
    }
  }

  std::sort(scan_files.begin(), scan_files.end());
  OPEN_LMM_PLOT("data_loader.enumerated_scans", scan_files.size());
  if (scan_files.empty()) {
    return Result<std::size_t>::Failure(Error::InvalidArgument(
        "No ." + param_.scan_type + " scan files found in " +
        scan_dir_path.string()));
  }
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kEnumerate,
                          scan_files.size(), scan_files.size());
  if (expected_count && *expected_count != scan_files.size()) {
    auto count = ValidateScanPoseCount(scan_files.size(), *expected_count,
                                       data_dir_path.string());
    return Result<std::size_t>::Failure(count.GetError());
  }
  ReportAlgorithmProgress(context, phase, 0, scan_files.size());
  for (std::size_t index = 0; index < scan_files.size(); ++index) {
    auto cancellation =
        CheckAlgorithmCancellation(context, "while reading scan files");
    if (!cancellation) {
      return Result<std::size_t>::Failure(cancellation.GetError());
    }
    const auto& scan_file = scan_files[index];
    try {
      auto scan = convertScanFunctor(scan_file.string());
      auto valid = ValidatePointCloud(
          scan, "agent " + context.agent.id.Value() + " frame " +
                    std::to_string(index) + " path " + scan_file.string());
      if (!valid) return Result<std::size_t>::Failure(valid.GetError());
      if (filter) {
        scan = downsampleWithRangeFilter(
            scan, param_.voxel_size, param_.min_range, param_.max_range);
        valid = ValidatePointCloud(
            scan, "filtered agent " + context.agent.id.Value() + " frame " +
                      std::to_string(index) + " path " + scan_file.string());
        if (!valid) return Result<std::size_t>::Failure(valid.GetError());
      }
      auto consumed = consumer(index, scan);
      if (!consumed) {
        return Result<std::size_t>::Failure(WithAlgorithmContext(
            consumed.GetError(), context));
      }
    } catch (const std::exception& e) {
      return Result<std::size_t>::Failure(Error::IoError(
          "agent " + context.agent.id.Value() + ", frame " +
          std::to_string(index) + ", scan " + scan_file.string() +
          ": " + e.what()));
    }
    ReportAlgorithmProgress(context, phase, index + 1, scan_files.size());
  }
  return Result<std::size_t>::Ok(scan_files.size());
}

Result<std::size_t> DataLoaderFile::VisitRawScanData(
    const AlgorithmExecutionContext& context,
    const fs::path& data_dir_path, const RawScanVisitor& visitor,
    AlgorithmProgressPhase phase) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before raw scan streaming");
  if (!cancellation) {
    return Result<std::size_t>::Failure(cancellation.GetError());
  }
  try {
    auto streamed = ForEachScanFile(context, data_dir_path, std::nullopt,
                                    phase, false, visitor);
    if (!streamed) {
      return Result<std::size_t>::Failure(
          WithAlgorithmContext(streamed.GetError(), context));
    }
    cancellation =
        CheckAlgorithmCancellation(context, "after raw scan streaming");
    if (!cancellation) {
      return Result<std::size_t>::Failure(cancellation.GetError());
    }
    return streamed;
  } catch (const std::exception& error) {
    return Result<std::size_t>::Failure(WithAlgorithmContext(
        Error::IoError("agent data " + data_dir_path.string() +
                       ": raw scan streaming exception: " + error.what()),
        context));
  } catch (...) {
    return Result<std::size_t>::Failure(WithAlgorithmContext(
        Error::IoError("agent data " + data_dir_path.string() +
                       ": unknown raw scan streaming exception"),
        context));
  }
}

}  // namespace open_lmm
