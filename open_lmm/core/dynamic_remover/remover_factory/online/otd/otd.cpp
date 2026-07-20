#include "otd.hpp"

OTD::OTD(const OTDParams& params) : params_(params) { initialize(); }

OTD::~OTD() {}

void OTD::initialize() {
  scan_num_ = 0;
  p_grdseg_ = std::make_shared<otd::Grd_Seg<PointType>>(
      params_.sensor_height, params_.tau_seeds, params_.tau_dis, false);
  p_otd_ = std::make_shared<otd::Otd3D<PointType>>(params_.tau_ratio,
                                                   params_.voxel_size);

  static_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
  dynamic_map_ =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OTD::run(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    Eigen::Isometry3d& optimized_pose) {
  // PointCloudType::Ptr scan(new PointCloudType);
  PointCloudType::Ptr ground_ptr(new PointCloudType);
  PointCloudType::Ptr nonground_ptr(new PointCloudType);

  p_grdseg_->Run(scan, ground_ptr, nonground_ptr, optimized_pose.matrix());
  p_otd_->Run(ground_ptr, nonground_ptr, scan_num_);

  // scan_.writeLabel("/home/gil/labels/", scan_num_);

  *static_map_ += *ground_ptr;
  if (params_.replace_intensity) {
    *static_map_ += *nonground_ptr;
  }

  // std::cout << "raw size : " << scan->size() << std::endl;
  // std::cout << "grund size : " << ground_ptr->size() << std::endl;
  // std::cout << "ngd size : " << nonground_ptr->size() << std::endl;
  // std::cout << "----" << std::endl;

  // TODO(gil) : save label

  scan_num_++;
  // TODO 임시
  return scan;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OTD::getStaticMap() {
  // std::cout << "size : " << p_otd_->GetMap(params_.replace_intensity)->size()
  //           << std::endl;
  return p_otd_->GetMap(params_.replace_intensity);
}