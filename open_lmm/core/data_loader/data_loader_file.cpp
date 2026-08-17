#include "data_loader_file.hpp"

#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>
#include <open_lmm/common/validation.hpp>

namespace fs = std::filesystem;
namespace open_lmm {

DataLoaderFile::DataLoaderFile(Config config) {
  parseConfig(config);

  if (param_.pose_format == "kitti") {
    transformFunctor = kittiPoseToIsometry3d;
  } else if (param_.pose_format == "tum") {
    transformFunctor = tumPoseToIsometry3d;
  } else if (param_.pose_format == "custom") {
    transformFunctor = customPoseToIsometry3d;
  } else {
    initialization_error_ = Error::InvalidArgument(
        "Unsupported pose format: '" + param_.pose_format +
        "'. Supported: kitti, tum, custom");
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

void DataLoaderFile::parseConfig(Config config) {
  param_.pose_format =
      config.param<std::string>("data_loader", "pose_format", "");
  param_.scan_type = config.param<std::string>("data_loader", "scan_type", "");
  param_.scan_dir_name =
      config.param<std::string>("data_loader", "scan_dir_name", "");
  param_.pose_file_name =
      config.param<std::string>("data_loader", "pose_file_name", "");
  param_.extrinsic = config.param<Eigen::Isometry3d>(
      "data_loader", "extrinsic", Eigen::Isometry3d::Identity());
  param_.voxel_size = config.param<float>("data_loader", "voxel_size", 0.1f);
  param_.min_range = config.param<float>("data_loader", "min_range", 0.0f);
  param_.max_range = config.param<float>("data_loader", "max_range", 100.0f);
  param_.delimiter = config.param<std::string>("data_loader", "delimiter", " ");
}

Result<AgentRawData> DataLoaderFile::Process(const AgentContext& ctx,
                                             const fs::path& data_dir) {
  if (initialization_error_) {
    return Result<AgentRawData>::Failure(*initialization_error_);
  }
  auto poses_result = loadPoseData(data_dir);
  if (!poses_result) return Result<AgentRawData>::Failure(poses_result.GetError());
  auto scans_result = loadFilteredScanData(data_dir);
  if (!scans_result) return Result<AgentRawData>::Failure(scans_result.GetError());
  auto poses = std::move(poses_result).Value();
  auto filtered_scans = std::move(scans_result).Value();
  auto count_result = ValidateScanPoseCount(filtered_scans.size(), poses.size(),
                                             data_dir.string());
  if (!count_result) return Result<AgentRawData>::Failure(count_result.GetError());

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
  std::vector<Eigen::Vector3f> map_points;
  pclToEigen(*map_ds, map_points);

  return Result<AgentRawData>::Ok(AgentRawData{
      .agent_id       = ctx.id,
      .odom_poses     = std::move(poses),
      .filtered_scans = std::move(filtered_scans),
      .map_points     = std::move(map_points),
  });
}

Result<PoseVec> DataLoaderFile::loadPoseData(
    fs::path data_dir_path) {
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
//     GlobalConfig::get_save_dir_path() / data_dir_name / param_.scan_dir_name;
// fs::create_directories(scan_save_dir);
// const fs::path file_name = scan_file.filename();
// const fs::path save_file_path = scan_save_dir / file_name;
// filtered_scans.push_back(p_scan_preprocessed);
// pcl::io::savePCDFileBinaryCompressed(save_file_path, *p_scan_preprocessed);

Result<ScanVec> DataLoaderFile::loadFilteredScanData(fs::path data_dir_path) {
  auto raw_result = loadRawScanData(data_dir_path);
  if (!raw_result) return Result<ScanVec>::Failure(raw_result.GetError());
  auto raw_scans = std::move(raw_result).Value();
  ScanVec p_filtered_scans;

  auto T = tq::tqdm(raw_scans);
  T.set_prefix("Data Loader");
  for (auto scan : T) {
    p_filtered_scans.push_back(downsampleWithRangeFilter(
        scan, param_.voxel_size, param_.min_range, param_.max_range));
  }
  T.finish();
  return Result<ScanVec>::Ok(std::move(p_filtered_scans));
}

Result<ScanVec> DataLoaderFile::loadRawScanData(fs::path data_dir_path) {
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

  ScanVec raw_scans;
  ScanVec filtered_scans;

  for (auto scan_file : scan_files) {
    const pcl::PointCloud<pcl::PointXYZI>::Ptr p_scan =
        convertScanFunctor(scan_file.string());
    raw_scans.push_back(p_scan);
  }

  return Result<ScanVec>::Ok(std::move(raw_scans));
}

}  // namespace open_lmm
