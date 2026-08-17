#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {
namespace fs = std::filesystem;

struct Thresholds {
  double pose_translation_m = 0.001;
  double pose_rotation_rad = 0.001;
  double pcd_rms_m = 0.02;
  double pcd_max_m = 0.20;
  double point_count_ratio = 0.001;
};
struct Pose { Eigen::Vector3d t; Eigen::Quaterniond q; };
struct DistanceStats { std::size_t samples{}; double mean{}, rms{}, max{}; };

std::vector<std::string> Split(const std::string& line) {
  std::vector<std::string> fields;
  std::istringstream input(line);
  std::string field;
  while (std::getline(input, field, ',')) fields.push_back(field);
  return fields;
}

bool LoadPoses(const fs::path& path, std::map<int, Pose>& poses) {
  std::ifstream input(path);
  if (!input) { std::cerr << "failed to open pose file: " << path << '\n'; return false; }
  std::string line;
  std::size_t line_number = 0;
  try {
    while (std::getline(input, line)) {
      ++line_number;
      if (line.empty()) continue;
      const auto f = Split(line);
      if (f.size() != 8) {
        std::cerr << path << ':' << line_number << ": expected 8 fields\n";
        return false;
      }
      const int index = std::stoi(f[0]);
      Pose pose{{std::stod(f[1]), std::stod(f[2]), std::stod(f[3])},
                Eigen::Quaterniond(std::stod(f[7]), std::stod(f[4]),
                                   std::stod(f[5]), std::stod(f[6])).normalized()};
      if (!poses.emplace(index, std::move(pose)).second) {
        std::cerr << path << ':' << line_number << ": duplicate index\n";
        return false;
      }
    }
  } catch (const std::exception& e) {
    std::cerr << path << ':' << line_number << ": " << e.what() << '\n';
    return false;
  }
  return !poses.empty();
}

bool ComparePoses(const fs::path& baseline, const fs::path& candidate,
                  const Thresholds& limits) {
  std::map<int, Pose> before, after;
  if (!LoadPoses(baseline, before) || !LoadPoses(candidate, after)) return false;
  if (before.size() != after.size()) {
    std::cerr << "pose count mismatch: " << before.size() << " vs " << after.size() << '\n';
    return false;
  }
  double t_sq = 0.0, r_sq = 0.0, t_max = 0.0, r_max = 0.0;
  for (const auto& [index, lhs] : before) {
    const auto it = after.find(index);
    if (it == after.end()) { std::cerr << "missing pose index " << index << '\n'; return false; }
    const double t = (lhs.t - it->second.t).norm();
    const double dot = std::clamp(std::abs(lhs.q.dot(it->second.q)), 0.0, 1.0);
    const double r = 2.0 * std::acos(dot);
    t_sq += t * t; r_sq += r * r;
    t_max = std::max(t_max, t); r_max = std::max(r_max, r);
  }
  std::cout << std::setprecision(9) << "  poses count=" << before.size()
            << " translation_rms_m=" << std::sqrt(t_sq / before.size())
            << " translation_max_m=" << t_max
            << " rotation_rms_rad=" << std::sqrt(r_sq / before.size())
            << " rotation_max_rad=" << r_max << '\n';
  return t_max <= limits.pose_translation_m && r_max <= limits.pose_rotation_rad;
}

DistanceStats CompareDirection(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& target) {
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  constexpr std::size_t kMaxSamples = 100000;
  const std::size_t stride = std::max<std::size_t>(1, source->size() / kMaxSamples);
  double sum = 0.0, squared_sum = 0.0, maximum = 0.0;
  std::size_t samples = 0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (std::size_t i = 0; i < source->size(); i += stride) {
    if (tree.nearestKSearch((*source)[i], 1, indices, squared_distances) != 1) continue;
    const double distance = std::sqrt(squared_distances.front());
    sum += distance; squared_sum += squared_distances.front();
    maximum = std::max(maximum, distance); ++samples;
  }
  return {samples, samples ? sum / samples : 0.0,
          samples ? std::sqrt(squared_sum / samples) : 0.0, maximum};
}

void PrintDistance(const char* label, const DistanceStats& s) {
  std::cout << "  " << label << " samples=" << s.samples << " mean_m=" << s.mean
            << " rms_m=" << s.rms << " max_m=" << s.max << '\n';
}

bool CompareClouds(const fs::path& baseline, const fs::path& candidate,
                   const Thresholds& limits) {
  auto before = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto after = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (pcl::io::loadPCDFile(baseline.string(), *before) != 0 ||
      pcl::io::loadPCDFile(candidate.string(), *after) != 0 ||
      before->empty() || after->empty()) {
    std::cerr << "failed to load non-empty PCD inputs\n"; return false;
  }
  const double count_ratio = std::abs(static_cast<double>(before->size()) -
      static_cast<double>(after->size())) / static_cast<double>(before->size());
  const auto forward = CompareDirection(before, after);
  const auto reverse = CompareDirection(after, before);
  std::cout << "  points baseline=" << before->size() << " candidate=" << after->size()
            << " count_ratio=" << count_ratio << '\n';
  PrintDistance("baseline_to_candidate", forward);
  PrintDistance("candidate_to_baseline", reverse);
  return count_ratio <= limits.point_count_ratio && forward.samples && reverse.samples &&
         forward.rms <= limits.pcd_rms_m && reverse.rms <= limits.pcd_rms_m &&
         forward.max <= limits.pcd_max_m && reverse.max <= limits.pcd_max_m;
}

std::vector<std::string> FindAgents(const fs::path& directory) {
  std::vector<std::string> agents;
  const std::string prefix = "optimized_poses_", suffix = ".txt";
  for (const auto& entry : fs::directory_iterator(directory)) {
    const std::string name = entry.path().filename().string();
    if (!entry.is_regular_file() || !name.starts_with(prefix) || !name.ends_with(suffix)) continue;
    agents.push_back(name.substr(prefix.size(), name.size() - prefix.size() - suffix.size()));
  }
  std::sort(agents.begin(), agents.end());
  return agents;
}

double Parse(char* text, const char* name) {
  const double value = std::stod(text);
  if (value < 0.0) throw std::invalid_argument(std::string(name) + " must be nonnegative");
  return value;
}
}  // namespace

int main(int argc, char** argv) {
  const bool compare_pcd = argc > 3 && std::string(argv[3]) == "--pcd";
  const int first_threshold = compare_pcd ? 4 : 3;
  const int threshold_count = argc - first_threshold;
  if (argc < 3 || threshold_count < 0 || threshold_count > (compare_pcd ? 5 : 2)) {
    std::cerr << "usage: open_lmm_artifact_compare BASELINE_DIR CANDIDATE_DIR "
                 "[--pcd] [pose_translation_m] [pose_rotation_rad] "
                 "[pcd_rms_m] [pcd_max_m] [point_count_ratio]\n";
    return 2;
  }
  try {
    const fs::path baseline = argv[1], candidate = argv[2];
    if (!fs::is_directory(baseline) || !fs::is_directory(candidate)) {
      std::cerr << "both arguments must be result directories\n"; return 2;
    }
    Thresholds limits;
    if (threshold_count > 0)
      limits.pose_translation_m = Parse(argv[first_threshold], "pose_translation_m");
    if (threshold_count > 1)
      limits.pose_rotation_rad = Parse(argv[first_threshold + 1], "pose_rotation_rad");
    if (threshold_count > 2) limits.pcd_rms_m = Parse(argv[first_threshold + 2], "pcd_rms_m");
    if (threshold_count > 3) limits.pcd_max_m = Parse(argv[first_threshold + 3], "pcd_max_m");
    if (threshold_count > 4)
      limits.point_count_ratio = Parse(argv[first_threshold + 4], "point_count_ratio");
    const auto agents = FindAgents(baseline);
    if (agents.empty()) { std::cerr << "baseline has no pose files\n"; return 2; }
    bool passed = true;
    for (const auto& agent : agents) {
      std::cout << "agent=" << agent << '\n';
      const bool pose_ok = ComparePoses(
          baseline / ("optimized_poses_" + agent + ".txt"),
          candidate / ("optimized_poses_" + agent + ".txt"), limits);
      bool cloud_ok = true;
      if (compare_pcd) {
        cloud_ok = CompareClouds(baseline / ("global_map_" + agent + ".pcd"),
                                 candidate / ("global_map_" + agent + ".pcd"), limits);
      } else {
        std::cout << "  pcd=SKIP (use --pcd to enable)\n";
      }
      std::cout << "  result=" << (pose_ok && cloud_ok ? "PASS" : "FAIL") << '\n';
      passed = passed && pose_ok && cloud_ok;
    }
    std::cout << "overall=" << (passed ? "PASS" : "FAIL") << '\n';
    return passed ? 0 : 1;
  } catch (const std::exception& e) {
    std::cerr << "comparison failed: " << e.what() << '\n'; return 2;
  }
}
