//
// Created by gil on 23. 11. 16.
//

#include "SingleMapDB.h"

namespace fs = boost::filesystem;


SingleMapDB::SingleMapDB() = default;

SingleMapDB::~SingleMapDB() = default;

SingleMapDB::SingleMapDB(std::string _data_path, int _map_idx): map_idx_(_map_idx) {
    std::cout << "Data index" << _map_idx << ": " << _data_path << std::endl;
//    std::string data_log = "Data" + std::to_string(_map_idx) + ": " + _data_path;
//    ROS_INFO("\033[1;34m  Uni-Mapper Started.\033[0m");

    pose_path_ = _data_path + "optimized_poses.txt";
    pcd_dir_path_ = _data_path + "Scans";
//    pcd_dir_path_ = _data_path + "StaticScans";
    graph_start_idx_ = map_idx_ * GRAPH_IDX_SCALE;

    anchor_node = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0),
                               gtsam::Point3(0.0, 0.0, 0.0));

    std_manager_ = new STDescManager(std_config_);

    getPoses();
    getPcdPath();
    sizeCheck();
    initNoise();
    initAnchorNode();
}

void SingleMapDB::initAnchorNode() {
    std::cout << graph_start_idx_ << std::endl;
    if (graph_start_idx_ == 0) {
        pose_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(graph_start_idx_, anchor_node, prior_noise_));
    } else {
        pose_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(graph_start_idx_, anchor_node, large_noise_));
    }
    values_.insert(graph_start_idx_, anchor_node);
}

void SingleMapDB::initNoise() {
    prior_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12).finished());

    odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

    gtsam::Vector robust_noise_vec(6);
    robust_noise_vec << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
    robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1),
            gtsam::noiseModel::Diagonal::Variances(robust_noise_vec));

    large_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << M_PI*M_PI, M_PI*M_PI, M_PI*M_PI, 1e8, 1e8, 1e8).finished());
}

void SingleMapDB::getPoses() {
    std::ifstream file(pose_path_);
    if (!file.is_open()){
        std::cerr << "Error: could not open file " << pose_path_ << std::endl;
        return;
    }

    Eigen::Matrix4d T_wl = Eigen::Matrix4d::Identity();

    Eigen::Vector3d prev_pose3 = Eigen::Vector3d::Zero();

    int idx = 0;
    std::string line;
    // TODO : curr = only kitti. Add other types of trajectory file format
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::vector<double> values;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 4; j++) {
                double val;
                iss >> val;
                T_wl(i, j) = val;
            }
        }

        Eigen::Vector3d curr_pose3 = T_wl.topRightCorner<3,1>().head<3>();
        if((prev_pose3 - curr_pose3).norm() > kf_dist_ || idx == 0){
            prev_pose3 = curr_pose3;
            pose_vec_.push_back(T_wl);
            kf_indexs_.push_back(idx);
        }
        idx++;
    }
}

void SingleMapDB::getPcdPath() {
    std::vector<std::string> pcd_paths_tmp;

    for (const auto& entry : fs::directory_iterator(pcd_dir_path_)) {
        if (entry.path().extension() == ".pcd") {
            pcd_paths_tmp.push_back(entry.path().string());
        }
    }
    std::sort(pcd_paths_tmp.begin(), pcd_paths_tmp.end());

    for(int idx : kf_indexs_) {
        if(idx < pcd_paths_tmp.size()){
            pcd_paths_vec_.push_back(pcd_paths_tmp[idx]);
        }
    }
}

void SingleMapDB::sizeCheck(){
    if (pose_vec_.size() != pcd_paths_vec_.size()){
        std::cout << "total frame number of poses and pcd files are not matched!!!" << "\n" <<
                  "pose num : " << pose_vec_.size() << "\n" <<
                  "pcd num : " << pcd_paths_vec_.size() << "\n";
        exit(-1);
    }else{
        std::cout << "total frame number of poses and pcd files are matched!!!" << "\n" <<
        "total frame num : " << pose_vec_.size() << "\n";
        total_frame_num_ = pose_vec_.size();
    }
}

