//
// Created by gil on 23. 11. 17.
//

#include "RosParamServer.h"

RosParamServer::RosParamServer() {
    nh_.param<std::string>("data/root_path", data_root_path_, "/home/gil/");
//    nh_.param<std::vector<std::string>>("data/map", data_dir_path_vec_, "/home/gil/");
    nh_.getParam("data/map", data_dir_path_vec_);
    nh_.param<std::string>("data/save_dir", save_dir_path_, "unimapper/"); // it means /.../home/LTslam/
    nh_.param<int>("data/kf_dist", kf_dist_, 0);
    nh_.param<int>("data/rs_dist", rs_dist_, 10);
    nh_.param<int>("data/rs_dist_th", rs_dist_th_, 10);
    read_parameters();
}

void RosParamServer::read_parameters() {
    // pre-preocess
    nh_.param<double>("STD/ds_size", std_config_.ds_size_, 0.5);
    nh_.param<int>("STD/maximum_corner_num", std_config_.maximum_corner_num_, 100);
    // key points
    nh_.param<double>("STD/plane_merge_normal_thre",std_config_.plane_merge_normal_thre_, 0.1);
    nh_.param<double>("STD/plane_detection_thre", std_config_.plane_detection_thre_,0.01);
    nh_.param<double>("STD/voxel_size", std_config_.voxel_size_, 2.0);
    nh_.param<int>("STD/voxel_init_num", std_config_.voxel_init_num_, 10);
    nh_.param<double>("STD/proj_image_resolution",std_config_.proj_image_resolution_, 0.5);
    nh_.param<double>("STD/proj_dis_min", std_config_.proj_dis_min_, 0);
    nh_.param<double>("STD/proj_dis_max", std_config_.proj_dis_max_, 2);
    nh_.param<double>("STD/corner_thre", std_config_.corner_thre_, 10);
    // std descriptor
    nh_.param<int>("STD/descriptor_near_num", std_config_.descriptor_near_num_, 10);
    nh_.param<double>("STD/descriptor_min_len", std_config_.descriptor_min_len_, 2);
    nh_.param<double>("STD/descriptor_max_len", std_config_.descriptor_max_len_,50);
    nh_.param<double>("STD/non_max_suppression_radius",std_config_.non_max_suppression_radius_, 2.0);
    nh_.param<double>("STD/std_side_resolution", std_config_.std_side_resolution_,0.2);
    // candidate search
    nh_.param<int>("STD/skip_near_num", std_config_.skip_near_num_, 50);
    nh_.param<int>("STD/candidate_num", std_config_.candidate_num_, 50);
    nh_.param<int>("STD/sub_frame_num", std_config_.sub_frame_num_, 10);
    nh_.param<double>("STD/rough_dis_threshold", std_config_.rough_dis_threshold_,0.01);
    nh_.param<double>("STD/vertex_diff_threshold",std_config_.vertex_diff_threshold_, 0.5);
    nh_.param<double>("STD/icp_threshold", std_config_.icp_threshold_, 0.5);
    nh_.param<double>("STD/normal_threshold", std_config_.normal_threshold_, 0.2);
    nh_.param<double>("STD/dis_threshold", std_config_.dis_threshold_, 0.5);

    std::cout << "Sucessfully load parameters:" << std::endl;
    std::cout << "----------------Main Parameters-------------------"
              << std::endl;
    std::cout << "voxel size:" << std_config_.voxel_size_ << std::endl;
    std::cout << "loop detection threshold: " << std_config_.icp_threshold_
              << std::endl;
    std::cout << "sub-frame number: " << std_config_.sub_frame_num_
              << std::endl;
    std::cout << "candidate number: " << std_config_.candidate_num_
              << std::endl;
    std::cout << "maximum corners size: " << std_config_.maximum_corner_num_
              << std::endl;
}