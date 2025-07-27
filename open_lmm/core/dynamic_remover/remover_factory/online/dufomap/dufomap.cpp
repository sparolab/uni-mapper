#include "dufomap.hpp"

#include <pcl/common/transforms.h>

DUFOMap::DUFOMap(const DUFOMapParams& params) : params_(params) {
  initialize(params_);
}

void DUFOMap::initialize(const DUFOMapParams& params) {
  ufo_map_ = new ufo::Map < ufo::MapType::SEEN_FREE | ufo::MapType::REFLECTION |
             ufo::MapType::LABEL > (params.resolution, params.levels);
  ufo_map_->reserve(100'000'000);
  static_map_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(
      new pcl::PointCloud<pcl::PointXYZI>());
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DUFOMap::run(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    Eigen::Isometry3d& optimized_pose) {
  ufo::Pose6f viewpoint;
  viewpoint.x() = optimized_pose.translation()[0];
  viewpoint.y() = optimized_pose.translation()[1];
  viewpoint.z() = optimized_pose.translation()[2];
  Eigen::Quaterniond q(optimized_pose.rotation());
  viewpoint.qw() = q.w();
  viewpoint.qx() = q.x();
  viewpoint.qy() = q.y();
  viewpoint.qz() = q.z();
  ufo::PointCloudColor cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::transformPointCloud(*scan, *transformed_scan, optimized_pose.matrix());
  pclToUfo(transformed_scan, cloud);
  ufo::insertPointCloud(*ufo_map_, cloud, viewpoint.translation,
                        params_.integration);

  ufo::PointCloudColor cloud_static;
  ufo::PointCloudColor cloud_dynamic;
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_static =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud_dynamic =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  for (auto& p : cloud) {
    if (!ufo_map_->seenFree(p)) {
      cloud_static.push_back(p);
      // ufoToPcl(cloud_static, pcl_cloud_static, false);

    } else {
      cloud_dynamic.push_back(p);
      // ufoToPcl(cloud_dynamic, pcl_cloud_dynamic, true);
    }
  }
  ufoToPcl(cloud_static, pcl_cloud_static, false);
  ufoToPcl(cloud_dynamic, pcl_cloud_dynamic, true);
  *static_map_ += *pcl_cloud_static;
  if (params_.replace_intensity) {
    *static_map_ += *pcl_cloud_dynamic;
  }
  return pcl_cloud_static;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DUFOMap::getStaticMap() {
  return static_map_;
}

void DUFOMap::pclToUfo(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud,
                       ufo::PointCloudColor& ufo_cloud) {
  ufo_cloud.clear();
  ufo_cloud.reserve(pcl_cloud->size());

  for (const auto& pcl_point : *pcl_cloud) {
    ufo::Point point(pcl_point.x, pcl_point.y, pcl_point.z);
    // intensity를 색상으로 변환 (회색조)
    uint8_t intensity = static_cast<uint8_t>(pcl_point.intensity * 255);
    ufo::Color color(intensity, intensity, intensity);

    ufo_cloud.emplace_back(point, color);
  }
}

void DUFOMap::ufoToPcl(const ufo::PointCloudColor& ufo_cloud,
                       pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud,
                       bool is_dynamic) {
  pcl_cloud->clear();
  pcl_cloud->reserve(ufo_cloud.size());

  for (const auto& ufo_point : ufo_cloud) {
    pcl::PointXYZI pcl_point;
    pcl_point.x = ufo_point.x;
    pcl_point.y = ufo_point.y;
    pcl_point.z = ufo_point.z;
    if (is_dynamic) {
      pcl_point.intensity = 1;
    } else {
      pcl_point.intensity = 0;
    }

    pcl_cloud->push_back(pcl_point);
  }
}