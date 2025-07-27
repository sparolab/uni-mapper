//
// Created by gil on 23. 11. 16.
//

#ifndef UNI_MAPPER_SINGLEMAPDB_H
#define UNI_MAPPER_SINGLEMAPDB_H

#include "utility.h"
#include "RosParamServer.h"
#include "STDesc.h"

class SingleMapDB : public RosParamServer{
public:
    int map_idx_;
    int graph_start_idx_;
    int total_frame_num_;

    std::string pose_path_;
    std::string pcd_dir_path_;
    std::vector<Eigen::Matrix4d> pose_vec_;
    std::vector<std::string> pcd_paths_vec_;

    gtsam::Values values_;
    gtsam::NonlinearFactorGraph pose_graph_;
    gtsam::Pose3 anchor_node;

    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
    gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr large_noise_;

    STDescManager* std_manager_;

    nav_msgs::Path original_path_;
    nav_msgs::Path updated_path_;
    std::vector<std::pair<int, int>> single_loop_pair_vec_;
    std::vector<std::pair<int, int>> multi_loop_pair_vec_;

    std::vector<int> kf_indexs_;

public:
    SingleMapDB();
    ~SingleMapDB();
    SingleMapDB(std::string, int);

    void getPoses();
    void getPcdPath();
    void sizeCheck();
    void initNoise();
    void initAnchorNode();
};


#endif //UNI_MAPPER_SINGLEMAPDB_H
