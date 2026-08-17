#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace {

struct DistanceStats {
  std::size_t samples = 0;
  double mean = 0.0;
  double rms = 0.0;
  double max = 0.0;
};

DistanceStats CompareDirection(
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& source,
    const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& target,
    std::size_t max_samples) {
  pcl::KdTreeFLANN<pcl::PointXYZI> tree;
  tree.setInputCloud(target);
  const std::size_t stride =
      std::max<std::size_t>(1, source->size() / max_samples);
  double sum = 0.0;
  double squared_sum = 0.0;
  double maximum = 0.0;
  std::size_t samples = 0;
  std::vector<int> indices(1);
  std::vector<float> squared_distances(1);
  for (std::size_t i = 0; i < source->size(); i += stride) {
    if (tree.nearestKSearch((*source)[i], 1, indices, squared_distances) != 1) {
      continue;
    }
    const double distance = std::sqrt(squared_distances.front());
    sum += distance;
    squared_sum += squared_distances.front();
    maximum = std::max(maximum, distance);
    ++samples;
  }
  return {
      .samples = samples,
      .mean = samples == 0 ? 0.0 : sum / samples,
      .rms = samples == 0 ? 0.0 : std::sqrt(squared_sum / samples),
      .max = maximum,
  };
}

void Print(const char* label, const DistanceStats& stats) {
  std::cout << label << " samples=" << stats.samples
            << " mean_m=" << stats.mean << " rms_m=" << stats.rms
            << " max_m=" << stats.max << '\n';
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: open_lmm_artifact_compare before.pcd after.pcd\n";
    return 2;
  }
  auto before = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  auto after = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  if (pcl::io::loadPCDFile(argv[1], *before) != 0 ||
      pcl::io::loadPCDFile(argv[2], *after) != 0) {
    std::cerr << "failed to load PCD input\n";
    return 1;
  }
  std::cout << "before_points=" << before->size()
            << " after_points=" << after->size() << '\n';
  constexpr std::size_t kMaxSamples = 100000;
  Print("before_to_after", CompareDirection(before, after, kMaxSamples));
  Print("after_to_before", CompareDirection(after, before, kMaxSamples));
  return 0;
}
