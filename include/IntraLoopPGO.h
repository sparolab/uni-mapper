//
// Created by gil on 23. 11. 13.
//

#ifndef UNI_MAPPER_INTRALOOPPGO_H
#define UNI_MAPPER_INTRALOOPPGO_H

#include "utility.h"
#include "RosParamServer.h"
#include "SingleMapDB.h"
#include <Eigen/Dense>


class IntraLoopPGO : public RosParamServer{
public:
    int total_frame_num_;
    int graph_start_idx_;
    std::vector<Eigen::Matrix4d> pose_vec_;
    std::vector<std::string> pcd_file_vec_;

    STDescManager* std_manager_{};
    STDescManager* std_manager_db_;

    gtsam::ISAM2 *isam_{};
    gtsam::Values& init_values_;
    gtsam::NonlinearFactorGraph& pose_graph_;
    gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
    gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;

    nav_msgs::Path& original_path_;
    nav_msgs::Path& updated_path_;
    std::vector<std::pair<int, int>>& loop_pair_vec_;

//    std::mutex mtx_;

public:
    IntraLoopPGO();
    IntraLoopPGO(SingleMapDB*);
    ~IntraLoopPGO();

    void run();

    void initGTSAM();
    void increOptGraph(int);
    void addOdomFactor(int, Eigen::Matrix4d&);
    void STDLoopClosing(int);
    void optimizeGraph();
    void updatePath();
};


#endif //UNI_MAPPER_INTRALOOPPGO_H
