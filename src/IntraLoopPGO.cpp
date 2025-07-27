//
// Created by gil on 23. 11. 13.
//

#include "IntraLoopPGO.h"

//IntraLoopPGO::IntraLoopPGO() = default;

IntraLoopPGO::~IntraLoopPGO() {
    delete std_manager_;
    delete isam_;
}

IntraLoopPGO::IntraLoopPGO(SingleMapDB* _DB): original_path_(_DB->original_path_),
                                              updated_path_(_DB->updated_path_),
                                              init_values_(_DB->values_),
                                              pose_graph_(_DB->pose_graph_),
                                              loop_pair_vec_(_DB->single_loop_pair_vec_) {
    total_frame_num_ = _DB->total_frame_num_;
    graph_start_idx_ = _DB->graph_start_idx_;  // remain 0 for anchor node;
    pose_vec_ = _DB->pose_vec_;
    pcd_file_vec_ = _DB->pcd_paths_vec_;

    prior_noise_ = _DB->prior_noise_;
    odometry_noise_ = _DB->odometry_noise_;
    robust_loop_noise_ = _DB->robust_loop_noise_;

    std_manager_ = new STDescManager(std_config_);
    std_manager_db_ = _DB->std_manager_;
}

void IntraLoopPGO::run() {
    initGTSAM();
    increOptGraph(total_frame_num_);
    optimizeGraph();
    updatePath();
}

void IntraLoopPGO::initGTSAM() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
}

void IntraLoopPGO::increOptGraph(int total_frame_num) {
    int frame_idx = 0;
    Eigen::Matrix4d prev_eigen_pose = Eigen::Matrix4d::Identity();
//    ros::Rate loop(50);

    while(frame_idx < total_frame_num) {
//        ros::spinOnce();
        addOdomFactor(frame_idx, prev_eigen_pose);
        STDLoopClosing(frame_idx);
//        loop.sleep();
        frame_idx++;
    }
    ROS_INFO("\033[1;32m----> All intra loop pairs Detected.\033[0m");
}

void IntraLoopPGO::addOdomFactor(int _idx, Eigen::Matrix4d& _prev_eigen_pose) {

    Eigen::Matrix4d curr_eigen_pose = pose_vec_[_idx];
    gtsam::Pose3 curr_gtsam_pose = gtsam::Pose3(gtsam::Rot3(curr_eigen_pose.block<3,3>(0, 0)),
                                                gtsam::Point3(curr_eigen_pose.block<3,1>(0, 3)));

    int global_idx = genGraphIdx(graph_start_idx_, _idx);

    if(_idx == 0) {
        init_values_.insert(global_idx, curr_gtsam_pose);
        pose_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(global_idx, curr_gtsam_pose, prior_noise_));
        _prev_eigen_pose = curr_eigen_pose;
    } else{
        Eigen::Matrix4d rel_odom = _prev_eigen_pose.inverse() * curr_eigen_pose;
        gtsam::Pose3 rel_gtsam_odom = gtsam::Pose3(gtsam::Rot3(rel_odom.block<3,3>(0, 0)),
                                                   gtsam::Point3(rel_odom.block<3,1>(0, 3)));
        init_values_.insert(global_idx, curr_gtsam_pose);
        pose_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(global_idx - 1, global_idx, rel_gtsam_odom, odometry_noise_));
        _prev_eigen_pose = curr_eigen_pose;
    }

    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped = gtsamPose3ToPoseStamped(curr_gtsam_pose);
    updated_path_.poses.push_back(pose_stamped);
//    original_path_.poses.push_back(pose_stamped);
}

void IntraLoopPGO::STDLoopClosing(int _frame_idx) {
    Eigen::Matrix4d curr_pose = pose_vec_[_frame_idx];
    std::string pcd_file_path = pcd_file_vec_[_frame_idx];
    pcl::PointCloud<PointType>::Ptr curr_pcd_local(new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr curr_pcd_global(new pcl::PointCloud<PointType>);
    pcl::io::loadPCDFile<PointType>(pcd_file_path, *curr_pcd_local);
    pcl::transformPointCloud(*curr_pcd_local, *curr_pcd_global, curr_pose);

    std::vector<STDesc> stds_vec;
    std_manager_->GenerateSTDescs(curr_pcd_global, stds_vec);

    std::pair<int, double> search_result(-1, 0);
    std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
    loop_transform.first << 0, 0, 0;
    loop_transform.second = Eigen::Matrix3d::Identity();
    std::vector<std::pair<STDesc, STDesc>> loop_std_pair;
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr plane_cloud = std_manager_->plane_cloud_vec_.back();
    std_manager_->SearchLoop(stds_vec, search_result, loop_transform, loop_std_pair, plane_cloud);

    if(search_result.first > 0){
        int matched_frame = search_result.first;
        std::cout << "STD loop found! : " << _frame_idx << "------->" << matched_frame << std::endl;

        std_manager_->PlaneGeomrtricIcp(std_manager_->plane_cloud_vec_.back(),
                                        std_manager_->plane_cloud_vec_[matched_frame],
                                        loop_transform);

        auto delta_T = Eigen::Affine3d::Identity();
        delta_T.translate(loop_transform.first);
        delta_T.rotate(loop_transform.second);

        Eigen::Matrix4d origin_pose_tgt = pose_vec_[matched_frame];
        Eigen::Matrix4d origin_pose_src = pose_vec_[_frame_idx];
        Eigen::Matrix4d refined_pose_src = delta_T * origin_pose_src.matrix();
        gtsam::Pose3 loop_rel_pose = gtsam::Pose3(refined_pose_src).between(gtsam::Pose3(origin_pose_tgt));

        int map_src_idx = genGraphIdx(graph_start_idx_, _frame_idx);
        int map_tgt_idx = genGraphIdx(graph_start_idx_, matched_frame);

        pose_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(map_src_idx, map_tgt_idx, loop_rel_pose, robust_loop_noise_));
        loop_pair_vec_.emplace_back(_frame_idx, search_result.first);
    }

    std_manager_->AddSTDescs(stds_vec);
//        std_manager_->key_cloud_vec_.push_back(curr_pcd_global->makeShared());

    // Local coordinate of STD : For multi session
    std::vector<STDesc> stds_vec_local;
    std_manager_db_->GenerateSTDescs(curr_pcd_local, stds_vec_local);
    std_manager_db_->AddSTDescs(stds_vec_local);
}

void IntraLoopPGO::optimizeGraph() {
    isam_->update(pose_graph_, init_values_);
    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();

//    auto t_update_begin = std::chrono::high_resolution_clock::now();
    init_values_ = isam_->calculateBestEstimate();
//    auto t_update_end = std::chrono::high_resolution_clock::now();
//    std::cout << "[Time] : " << (t_update_end - t_update_begin).count() * 1000 << std::endl;
}


void IntraLoopPGO::updatePath(){
    updated_path_.poses.clear();

    for (const gtsam::Key idx : init_values_.keys()) {
        auto gtsam_pose = init_values_.at<gtsam::Pose3>(idx);
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped = gtsamPose3ToPoseStamped(gtsam_pose);
        updated_path_.poses.push_back(pose_stamped);
    }
    original_path_.poses.clear();
}


