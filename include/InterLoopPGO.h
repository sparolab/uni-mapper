//
// Created by gil on 23. 11. 15.
//

#ifndef UNI_MAPPER_INTERLOOPPGO_H
#define UNI_MAPPER_INTERLOOPPGO_H

#include "utility.h"
#include "SingleMapDB.h"
#include "BetweenFactorWithAnchoring.h"
#include "gtsam/nonlinear/GncOptimizer.h"
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>


class InterLoopPGO : public RosParamServer{
public:
    STDescManager* central_std_manager_;
    STDescManager* query_std_manager_;

    gtsam::Values& central_value_;
    gtsam::NonlinearFactorGraph& central_pose_graph_;
    gtsam::Values& query_value_;
    gtsam::NonlinearFactorGraph& query_pose_graph_;

    nav_msgs::Path& central_path_;
    nav_msgs::Path& query_path_;

    int central_frame_num_;
    int query_frame_num_;
    int central_graph_start_idx_;
    int query_graph_start_idx_;

    gtsam::ISAM2 *isam_{};
    gtsam::noiseModel::Base::shared_ptr robust_loop_noise_;
    gtsam::noiseModel::Diagonal::shared_ptr icp_noise_;
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    gtsam::GncOptimizer<gtsam::LevenbergMarquardtParams> *gncOptimizer{};

    std::vector<std::pair<int, int>>& multi_loop_pair_vec_;

    std::mutex mtx_;

    std::vector<std::string> central_pcd_paths_vec_;
    std::vector<std::string> query_pcd_paths_vec_;

public:
    InterLoopPGO();
    ~InterLoopPGO();
    InterLoopPGO(SingleMapDB*, SingleMapDB*);

    void run();

    void initGTSAM();
    void combineToCentralGraph();
    void interSTDLoopClosing();
    void optimizeInterGraph();
    void updateMultiPath();
    void interRSLoopClosing();
};


#endif //UNI_MAPPER_INTERLOOPPGO_H
