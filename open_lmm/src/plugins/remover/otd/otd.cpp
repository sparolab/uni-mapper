#include "otd.hpp"

OTD::OTD(const OTDParams& params) : params_(params) { initialize(); }

OTD::~OTD() {}

void OTD::initialize() {
  scan_num_ = 0;
  p_grdseg_ = std::make_shared<otd::Grd_Seg<PointType>>(
      params_.sensor_height, params_.tau_seeds, params_.tau_dis, false);
  p_otd_ = std::make_shared<otd::Otd3D<PointType>>(params_.tau_ratio,
                                                   params_.voxel_size);

  ground_scan_.reset(new PointCloudType);
  nonground_scan_.reset(new PointCloudType);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr OTD::run(
    pcl::PointCloud<pcl::PointXYZI>::Ptr& scan,
    Eigen::Isometry3d& optimized_pose) {
  // PointCloudType::Ptr scan(new PointCloudType);
  ground_scan_->clear();
  nonground_scan_->clear();

  p_grdseg_->Run(scan, ground_scan_, nonground_scan_, optimized_pose.matrix());
  p_otd_->Run(ground_scan_, nonground_scan_, scan_num_);

  // scan_.writeLabel("/home/gil/labels/", scan_num_);

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
