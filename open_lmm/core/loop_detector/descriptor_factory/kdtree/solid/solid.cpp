#include "solid.h"

#include <algorithm>
#include <cmath>
#include <limits>

SolidParams::SolidParams() {
  open_lmm::Config config = open_lmm::Config(
      open_lmm::GlobalConfig::get_global_config_path("config_loop_detector"));
  fov_u = config.param<double>("loop_detector", "fov_u", 2.0);
  fov_d = config.param<double>("loop_detector", "fov_d", -24.8);
  num_angle = config.param<int>("loop_detector", "num_angle", 60);
  num_range = config.param<int>("loop_detector", "num_range", 40);
  num_height = config.param<int>("loop_detector", "num_height", 32);
  min_distance = config.param<int>("loop_detector", "min_distance", 3);
  max_distance = config.param<int>("loop_detector", "max_distance", 80);
  voxel_size = config.param<double>("loop_detector", "voxel_size", 0.4);
}

SOLiD::SOLiD(const SolidParams& params) : params_(params) {
  descriptor_ = Eigen::MatrixXd::Zero(params_.num_range + params_.num_angle, 1);
  r_solid_key_ = Eigen::VectorXd::Zero(params_.num_range);
}

bool SolidParams::equals(const SolidParams& other) const {
  return fov_u == other.fov_u && fov_d == other.fov_d &&
         num_angle == other.num_angle && num_range == other.num_range &&
         num_height == other.num_height && min_distance == other.min_distance &&
         max_distance == other.max_distance && voxel_size == other.voxel_size;
}

const Eigen::MatrixXd& SOLiD::getDescriptor() const { return descriptor_; }

const Eigen::VectorXd& SOLiD::getDescriptorKey() const { return r_solid_key_; }

const Eigen::VectorXd& SOLiD::getASolidKey() const { return a_solid_key_; }

const SolidParams& SOLiD::getParams() const { return params_; }

std::pair<double, Eigen::Isometry3d> SOLiD::distance(
    const std::shared_ptr<IDescriptorKdtree>& other) const {
  // Guard Code: Two ScanContexts are only comparable if they are constructed
  // with identical parameters
  // if (!params_.equals(other.params_)) {
  //   throw std::runtime_error(
  //       "SOLiD::distance The two SOLiDs are not comparable due to different "
  //       "parameters.");
  // }

  double r_solid_distance = loopDetection(other);
  Eigen::Isometry3d min_rel_pose = poseEstimation(other);

  return {r_solid_distance, min_rel_pose};
}

double SOLiD::loopDetection(
    const std::shared_ptr<IDescriptorKdtree>& other) const {
  const Eigen::VectorXd r_query = getDescriptorKey();
  const Eigen::VectorXd r_candidate = other->getDescriptorKey();
  double cosine_similarity =
      (r_query.dot(r_candidate)) / (r_query.norm() * r_candidate.norm());
  return (1 - cosine_similarity);
}

Eigen::Isometry3d SOLiD::poseEstimation(
    const std::shared_ptr<IDescriptorKdtree>& other) const {
  const Eigen::VectorXd a_query = getASolidKey();
  const Eigen::MatrixXd descriptor_candidate = other->getDescriptor();
  const Eigen::VectorXd a_candidate =
      getASolidKeyFromDescriptor(descriptor_candidate);

  double min_distance = std::numeric_limits<double>::max();
  int min_offset = 0;
  const int a_solid_size = a_query.size();

  for (int offset = 0; offset < a_solid_size; ++offset) {
    double a_solid_distance = shiftedDistance(offset, a_query, a_candidate);

    if (a_solid_distance < min_distance) {
      min_distance = a_solid_distance;
      min_offset = offset;
    }
  }
  const double angle_resolution = (2 * M_PI) / a_solid_size;  // radians
  Eigen::Isometry3d min_rel_pose = Eigen::Isometry3d::Identity();
  min_rel_pose.rotate(Eigen::AngleAxisd(min_offset * angle_resolution,
                                        Eigen::Vector3d::UnitZ()));
  return min_rel_pose;
}

const Eigen::VectorXd SOLiD::getASolidKeyFromDescriptor(
    const Eigen::MatrixXd& descriptor) const {
  return descriptor.col(0).tail(params_.num_angle);
}

double SOLiD::shiftedDistance(const size_t offset, const Eigen::VectorXd& query,
                              const Eigen::VectorXd& candidate) const {
  const size_t key_size = query.size();
  // Eigen::VectorXd rotated_key(key_size);
  // rotated_key << query.segment(key_size - offset, key_size),
  // query.head(key_size - offset);
  Eigen::VectorXd rotated_a_solid = Eigen::VectorXd::Zero(key_size);
  for (size_t idx = 0; idx < key_size; idx++) {
    size_t offset_idx = (idx + offset) % key_size;
    rotated_a_solid[idx] = query[offset_idx];
  }
  return (candidate - rotated_a_solid).cwiseAbs().sum();
}

std::shared_ptr<IDescriptorKdtree> SOLiD::makeDescriptor(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& lidar_scan) {
  std::shared_ptr<SOLiD> solid = std::make_shared<SOLiD>(params_);

  Eigen::MatrixXd range_matrix =
      Eigen::MatrixXd::Zero(params_.num_range, params_.num_height);
  Eigen::MatrixXd angle_matrix =
      Eigen::MatrixXd::Zero(params_.num_angle, params_.num_height);

  double gap_angle = 360.0 / params_.num_angle;
  double gap_range =
      static_cast<double>(params_.max_distance) / params_.num_range;
  double gap_height = (params_.fov_u - params_.fov_d) / params_.num_height;

  for (const pcl::PointXYZI& point : lidar_scan->points) {
    double x = point.x;
    double y = point.y;
    double z = point.z;
    if (x == 0.0f) x = 0.001f;
    if (y == 0.0f) y = 0.001f;
    double theta = std::atan2(y, x) * 180.0 / M_PI;
    if (theta < 0) theta += 360.0;
    double dist_xy = std::sqrt(x * x + y * y);
    float phi = std::atan2(z, dist_xy) * 180.0f / static_cast<float>(M_PI);
    size_t idx_range = std::min(static_cast<size_t>(dist_xy / gap_range),
                                params_.num_range - 1);
    size_t idx_angle =
        std::min(static_cast<size_t>(theta / gap_angle), params_.num_angle - 1);
    size_t idx_height =
        std::min(static_cast<size_t>((phi - params_.fov_d) / gap_height),
                 params_.num_height - 1);
    if (idx_range >= 0 && idx_range < params_.num_range && idx_angle >= 0 &&
        idx_angle < params_.num_angle && idx_height >= 0 &&
        idx_height < params_.num_height) {
      range_matrix(idx_range, idx_height) += 1.0;
      angle_matrix(idx_angle, idx_height) += 1.0;
    }
  }

  Eigen::VectorXd number_vector = Eigen::VectorXd::Zero(params_.num_height);
  for (int col_idx = 0; col_idx < range_matrix.cols(); ++col_idx) {
    number_vector(col_idx) = range_matrix.col(col_idx).sum();
  }
  double min_val = number_vector.minCoeff();
  double max_val = number_vector.maxCoeff();
  if (max_val > min_val) {
    number_vector = (number_vector.array() - min_val) / (max_val - min_val);
  }

  Eigen::VectorXd range_solid = range_matrix * number_vector;
  Eigen::VectorXd angle_solid = angle_matrix * number_vector;

  solid->descriptor_.col(0).head(params_.num_range) = range_solid;
  solid->descriptor_.col(0).tail(params_.num_angle) = angle_solid;

  solid->r_solid_key_ = range_solid;
  solid->a_solid_key_ = angle_solid;

  return solid;
}
