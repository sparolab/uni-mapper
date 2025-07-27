#include "data_loader_file.hpp"

#include <pcl/common/transforms.h>

namespace fs = std::filesystem;
namespace open_lmm {

DataLoaderFile::DataLoaderFile(Config config) {
  parseConfig(config);

  // TODO(gil) : switch-case
  if (param_.pose_format == "kitti") {
    transformFunctor = kittiPoseToIsometry3d;
  } else if (param_.pose_format == "tum") {
    transformFunctor = tumPoseToIsometry3d;
  } else if (param_.pose_format == "custom") {
    transformFunctor = customPoseToIsometry3d;
  } else {
    throw std::invalid_argument(
        "[data_loader_file.hpp] Unsupported data pose format: " +
        param_.pose_format);
  }

  if (param_.scan_type == "pcd") {
    convertScanFunctor = readPointsFromPCD;
  } else if (param_.scan_type == "bin") {
    convertScanFunctor = readPointsFromBin;
  } else if (param_.pose_format == "custom") {
    throw std::invalid_argument(
        "[data_loader_file.hpp] Not implemented scan type: " +
        param_.scan_type);
  } else {
    throw std::invalid_argument(
        "[data_loader_file.hpp] Unsupported data scan type: " +
        param_.scan_type);
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

std::tuple<PoseVec, ScanVec, ScanVec> DataLoaderFile::process(
    std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
    fs::path data_dir_path) {
  auto poses = loadPoseData(data_dir_path);
  auto raw_scans = loadRawScanData(data_dir_path);
  auto filtered_scans = loadFilteredScanData(data_dir_path);
  shared_data->db_odom_poses[agent_id] = poses;
  shared_data->db_scans[agent_id] = filtered_scans;

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
  std::vector<Eigen::Vector3f> points;
  pclToEigen(*map, points);

  // TODO(gil) : hardcoded voxel leaf size
  pcl::PointCloud<pcl::PointXYZI>::Ptr map_ds =
      downsampleWithRangeFilter(map, 2.0, 0, 0, false);
  std::vector<int> tmp_indices;
  pcl::removeNaNFromPointCloud(*map_ds, *map_ds, tmp_indices);
  std::vector<Eigen::Vector3f> tmp_points;
  pclToEigen(*map_ds, tmp_points);

  shared_data->db_original_maps[agent_id] = tmp_points;
  return {poses, raw_scans, filtered_scans};
}

std::vector<Eigen::Isometry3d> DataLoaderFile::loadPoseData(
    fs::path data_dir_path) {
  const fs::path pose_file_path = data_dir_path / param_.pose_file_name;
  if (!fs::exists(pose_file_path)) {
    throw std::invalid_argument(
        "[data_loader_file.hpp] Invalid pose file path: " +
        pose_file_path.string());
  }

  std::ifstream file(pose_file_path);
  char delimiter;
  if (param_.delimiter.size() > 1) {
    throw std::invalid_argument("[data_loader_file.hpp] Invalid delimiter: " +
                                param_.delimiter);
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
    throw std::invalid_argument(
        "[data_loader_file.hpp] Invalid scan dir path: " +
        scan_dir_path.string());
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