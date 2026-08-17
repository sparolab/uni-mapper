#include "data_loader_file.hpp"

#include <pcl/common/transforms.h>
#include <spdlog/spdlog.h>

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
    spdlog::error("[DataLoaderFile] Unsupported pose format: {}. "
                  "Supported: kitti, tum, custom", param_.pose_format);
    std::exit(1);
  }

  if (param_.scan_type == "pcd") {
    convertScanFunctor = readPointsFromPCD;
  } else if (param_.scan_type == "bin") {
    convertScanFunctor = readPointsFromBin;
  } else {
    spdlog::error("[DataLoaderFile] Unsupported scan type: {}. "
                  "Supported: pcd, bin", param_.scan_type);
    std::exit(1);
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

AgentRawData DataLoaderFile::Process(const AgentContext& ctx,
                                     const fs::path& data_dir) {
  auto poses          = loadPoseData(data_dir);
  auto raw_scans      = loadRawScanData(data_dir);
  auto filtered_scans = loadFilteredScanData(data_dir);

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

  return AgentRawData{
      .agent_id       = ctx.id,
      .odom_poses     = std::move(poses),
      .raw_scans      = std::move(raw_scans),
      .filtered_scans = std::move(filtered_scans),
      .map_points     = std::move(map_points),
  };
}

std::vector<Eigen::Isometry3d> DataLoaderFile::loadPoseData(
    fs::path data_dir_path) {
  const fs::path pose_file_path = data_dir_path / param_.pose_file_name;
  if (!fs::exists(pose_file_path)) {
    spdlog::error("[DataLoaderFile] Pose file not found: {}", pose_file_path.string());
    std::exit(1);
  }

  std::ifstream file(pose_file_path);
  char delimiter;
  if (param_.delimiter.size() > 1) {
    spdlog::error("[DataLoaderFile] Delimiter must be a single character, got: {}",
                  param_.delimiter);
    std::exit(1);
  } else {
    delimiter = param_.delimiter[0];
  }

  std::vector<std::string> lines;
  std::string line;
  while (std::getline(file, line)) {
    lines.push_back(line);
  }

  PoseVec poses;

  for (auto line : lines) {
    std::istringstream iss(line);
    std::vector<double> values;
    std::string value;
    while (std::getline(iss, value, delimiter)) {
      values.push_back(std::stod(value));
    }
    poses.push_back(param_.extrinsic * transformFunctor(values));
  }

  return poses;
}

// const fs::path data_dir_name = data_dir_path.filename();
// const fs::path scan_save_dir =
//     GlobalConfig::get_save_dir_path() / data_dir_name / param_.scan_dir_name;
// fs::create_directories(scan_save_dir);
// const fs::path file_name = scan_file.filename();
// const fs::path save_file_path = scan_save_dir / file_name;
// filtered_scans.push_back(p_scan_preprocessed);
// pcl::io::savePCDFileBinaryCompressed(save_file_path, *p_scan_preprocessed);

ScanVec DataLoaderFile::loadFilteredScanData(fs::path data_dir_path) {
  auto raw_scans = loadRawScanData(data_dir_path);
  ScanVec p_filtered_scans;

  auto T = tq::tqdm(raw_scans);
  T.set_prefix("Data Loader");
  for (auto scan : T) {
    p_filtered_scans.push_back(downsampleWithRangeFilter(
        scan, param_.voxel_size, param_.min_range, param_.max_range));
  }
  T.finish();
  return p_filtered_scans;
}

ScanVec DataLoaderFile::loadRawScanData(fs::path data_dir_path) {
  const fs::path scan_dir_path = data_dir_path / param_.scan_dir_name;
  if (!fs::exists(scan_dir_path) || !fs::is_directory(scan_dir_path)) {
    spdlog::error("[DataLoaderFile] Scan directory not found: {}", scan_dir_path.string());
    std::exit(1);
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

  return raw_scans;
}

}  // namespace open_lmm