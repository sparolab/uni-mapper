#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include "loop_detector_kdtree.hpp"

namespace open_lmm {

std::unique_ptr<LoopDetectorBase> LoopDetectorBase::createInstance(
    Config config) {
  std::string loop_detector_type =
      config.param<std::string>("loop_detector", "loop_detector_type", "");
  if (loop_detector_type == "kdtree") {
    return std::make_unique<LoopDetectorKdtree>(KdtreeParams());
  } else if (loop_detector_type == "hashmap") {
    throw std::runtime_error(
        "[loop_detector_base.cpp] Hashmap loop detector is not implemented ");
  } else {
    throw std::invalid_argument(
        "[loop_detector_base.cpp] Invalid loop detector type: " +
        loop_detector_type);
  }
};

bool LoopDetectorBase::TryKissMatcher(
    const std::vector<Eigen::Vector3f> tgt_map_vec,
    const std::vector<Eigen::Vector3f> src_map_vec, const float leaf_size,
    const bool use_quatro, Eigen::Matrix4f& output) {
  kiss_matcher::KISSMatcherConfig config =
      kiss_matcher::KISSMatcherConfig(leaf_size);
  config.use_quatro_ = use_quatro;
  kiss_matcher::KISSMatcher matcher(config);

  const auto solution = matcher.estimate(src_map_vec, tgt_map_vec);

  Eigen::Matrix4f solution_eigen = Eigen::Matrix4f::Identity();
  solution_eigen.block<3, 3>(0, 0) = solution.rotation.cast<float>();
  solution_eigen.topRightCorner(3, 1) = solution.translation.cast<float>();

  size_t num_rot_inliers = matcher.getNumRotationInliers();
  size_t num_final_inliers = matcher.getNumFinalInliers();

  size_t thres_num_inliers = 5;
  if (num_final_inliers < thres_num_inliers) {
    std::cout << "\033[1;33m=> KISS-MATCHER might have failed :(\033[0m\n";
    output = Eigen::Matrix4f::Identity();
    return false;
  } else {
    std::cout << "\033[1;32m=> KISS-MATCHER likely succeeded XD\033[0m\n";
    output = solution_eigen;
    return true;
  }
}

}  // namespace open_lmm