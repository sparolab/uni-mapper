#include "data_loader_file.hpp"

#include <pcl/common/transforms.h>
#include <tqdmcpp/tqdmcpp.hpp>

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

Result<AgentRawData> DataLoaderFile::Process(const AgentContext& ctx,
                                             const fs::path& data_dir) {
  OPEN_LMM_ZONE_N("DataLoader.Process");
  if (initialization_error_) {
    return Result<AgentRawData>::Failure(*initialization_error_);
  }
  auto poses_result = loadPoseData(data_dir);
  if (!poses_result) return Result<AgentRawData>::Failure(poses_result.GetError());
  auto scans_result = loadFilteredScanData(data_dir);
  if (!scans_result) return Result<AgentRawData>::Failure(scans_result.GetError());
  auto poses = std::move(poses_result).Value();
  auto filtered_scans = std::move(scans_result).Value();
  OPEN_LMM_PLOT("data_loader.pose_count", poses.size());
  OPEN_LMM_PLOT("data_loader.scan_count", filtered_scans.size());
  auto count_result = ValidateScanPoseCount(filtered_scans.size(), poses.size(),
                                             data_dir.string());
  if (!count_result) return Result<AgentRawData>::Failure(count_result.GetError());

  std::vector<Eigen::Vector3f> map_points;
  {
    OPEN_LMM_ZONE_N("DataLoader.BuildMapPoints");
  // KISSMatcher용 2m voxel 다운샘플 맵 포인트 생성
  pcl::PointCloud<pcl::PointXYZI>::Ptr map(new pcl::PointCloud<pcl::PointXYZI>);
  for (size_t i = 0; i < filtered_scans.size(); ++i) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(
        new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*filtered_scans[i], *transformed_scan,
                             poses[i].matrix());
    *map += *transformed_scan;
  }
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*map, *map, indices);
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ds =
      downsampleWithRangeFilter(map, 2.0, 0, 0, false);
  std::vector<int> tmp_indices;
  pcl::removeNaNFromPointCloud(*map_ds, *map_ds, tmp_indices);
  pclToEigen(*map_ds, map_points);
  }

  AgentRawData output{
      .agent_id       = ctx.id,
      .odom_poses     = std::move(poses),
      .filtered_scans = std::move(filtered_scans),
      .map_points     = std::move(map_points),
  };
  auto valid_output = ValidateAgentRawData(
      output, "DataLoader agent '" + ctx.id.Value() + "'");
  if (!valid_output) {
    return Result<AgentRawData>::Failure(valid_output.GetError());
  }
  return Result<AgentRawData>::Ok(std::move(output));
}

Result<PoseVec> DataLoaderFile::loadPoseData(
    fs::path data_dir_path) {
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

Result<ScanVec> DataLoaderFile::loadFilteredScanData(fs::path data_dir_path) {
  OPEN_LMM_ZONE_N("DataLoader.DownsampleScans");
  auto raw_result = loadRawScanData(data_dir_path);
  if (!raw_result) return Result<ScanVec>::Failure(raw_result.GetError());
  auto raw_scans = std::move(raw_result).Value();
  ScanVec p_filtered_scans;

  if (param_.show_progress) {
    auto progress = tq::tqdm(raw_scans);
    progress.set_prefix("Data Loader");
    for (auto scan : progress) {
      p_filtered_scans.push_back(downsampleWithRangeFilter(
          scan, param_.voxel_size, param_.min_range, param_.max_range));
    }
    progress.finish();
  } else {
    for (const auto& scan : raw_scans) {
      p_filtered_scans.push_back(downsampleWithRangeFilter(
          scan, param_.voxel_size, param_.min_range, param_.max_range));
    }
  }
  return Result<ScanVec>::Ok(std::move(p_filtered_scans));
}

Result<ScanVec> DataLoaderFile::loadRawScanData(fs::path data_dir_path) {
  OPEN_LMM_ZONE_N("DataLoader.ReloadRawScans");
  const fs::path scan_dir_path = data_dir_path / param_.scan_dir_name;
  if (!fs::exists(scan_dir_path) || !fs::is_directory(scan_dir_path)) {
    return Result<ScanVec>::Failure(Error::FileNotFound(scan_dir_path.string()));
  }

  std::vector<fs::path> scan_files;
  for (const auto& scan_file : fs::directory_iterator(scan_dir_path)) {
    if (scan_file.is_regular_file() &&
        scan_file.path().extension().string().substr(1) == param_.scan_type) {
      scan_files.push_back(scan_file.path());
    }
  }

  std::sort(scan_files.begin(), scan_files.end());
  OPEN_LMM_PLOT("data_loader.enumerated_scans", scan_files.size());

  ScanVec raw_scans;

  if (scan_files.empty()) {
    return Result<ScanVec>::Failure(Error::InvalidArgument(
        "No ." + param_.scan_type + " scan files found in " +
        scan_dir_path.string()));
  }

  for (const auto& scan_file : scan_files) {
    try {
      raw_scans.push_back(convertScanFunctor(scan_file.string()));
    } catch (const std::exception& e) {
      return Result<ScanVec>::Failure(Error::IoError(
          "agent data " + data_dir_path.string() + ", scan " +
          scan_file.string() + ": " + e.what()));
    }
  }

  return Result<ScanVec>::Ok(std::move(raw_scans));
}

Result<std::size_t> DataLoaderFile::VisitRawScanData(
    const fs::path& data_dir_path, const RawScanVisitor& visitor) {
  OPEN_LMM_ZONE_N("DataLoader.StreamRawScans");
  const fs::path scan_dir_path = data_dir_path / param_.scan_dir_name;
  if (!fs::exists(scan_dir_path) || !fs::is_directory(scan_dir_path)) {
    return Result<std::size_t>::Failure(
        Error::FileNotFound(scan_dir_path.string()));
  }
  std::vector<fs::path> scan_files;
  for (const auto& scan_file : fs::directory_iterator(scan_dir_path)) {
    if (scan_file.is_regular_file() &&
        scan_file.path().extension().string().substr(1) == param_.scan_type) {
      scan_files.push_back(scan_file.path());
    }
  }
  std::sort(scan_files.begin(), scan_files.end());
  if (scan_files.empty()) {
    return Result<std::size_t>::Failure(Error::InvalidArgument(
        "No ." + param_.scan_type + " scan files found in " +
        scan_dir_path.string()));
  }
  for (std::size_t index = 0; index < scan_files.size(); ++index) {
    try {
      auto scan = convertScanFunctor(scan_files[index].string());
      auto visited = visitor(index, scan);
      if (!visited) {
        return Result<std::size_t>::Failure(visited.GetError());
      }
    } catch (const std::exception& error) {
      return Result<std::size_t>::Failure(Error::IoError(
          "agent data " + data_dir_path.string() + ", scan " +
          scan_files[index].string() + ": " + error.what()));
    }
  }
  return Result<std::size_t>::Ok(scan_files.size());
}

}  // namespace open_lmm
