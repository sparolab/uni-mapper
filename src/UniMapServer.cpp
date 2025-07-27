//
// Created by gil on 23. 11. 13.
//

#include "UniMapServer.h"

UniMapServer::UniMapServer() {
    for(auto dir_name : data_dir_path_vec_ ) {
        std::string full_path = data_root_path_ + dir_name + "/";
        full_data_path_vec_.emplace_back(full_path);
    }
    for(int i = 0; i < full_data_path_vec_.size(); i++) {
        SingleMapDB* test = new SingleMapDB(full_data_path_vec_[i], i);
        multi_map_db_.emplace_back(test);
    }

    pub_origin_path_ = nh_.advertise<nav_msgs::Path>("origin_path", 5);
    pub_updated_path_ = nh_.advertise<nav_msgs::Path>("updated_path", 5);
    pub_loop_edge_ = nh_.advertise<visualization_msgs::MarkerArray>("loop_edge", 5);

    pub_updated_path_vec_.resize(full_data_path_vec_.size());
    pub_loop_marker_vec_.resize(full_data_path_vec_.size());
    pub_pcd_vec_.resize(full_data_path_vec_.size());

    for(int i = 0; i < full_data_path_vec_.size(); i++) {
        std::string path_msg_name = "updated_path_" + std::to_string(i);
        pub_updated_path_vec_[i] = nh_.advertise<nav_msgs::Path>(path_msg_name, 5);
    }
    for(int i = 0; i < full_data_path_vec_.size(); i++) {
        std::string path_msg_name = "loop_marker_" + std::to_string(i);
        pub_loop_marker_vec_[i] = nh_.advertise<visualization_msgs::MarkerArray>(path_msg_name, 5);
    }
    for(int i = 0; i < full_data_path_vec_.size(); i++) {
        std::string pcd_msg_name = "pointcloud_" + std::to_string(i);
        pub_pcd_vec_[i] = nh_.advertise<sensor_msgs::PointCloud2>(pcd_msg_name, 5);
    }
}

UniMapServer::~UniMapServer() = default;

void UniMapServer::run(){
    singleMapOpt();
    multiMapMerging();
    pubPCL();
    saveUpdatedPoses();
    ros::shutdown();
}

void UniMapServer::singleMapOpt(){
//#pragma omp parallel for
    for(const auto& iter : multi_map_db_) {
        curr_single_map_idx_ = iter->map_idx_;
        IntraLoopPGO single_pgo(iter);
        single_pgo.run();
    }
}

void UniMapServer::multiMapMerging() {
    SingleMapDB* central_map = multi_map_db_[0];

    for (int i = 1; i < multi_map_db_.size(); i++) {
        InterLoopPGO multi_pgo(central_map, multi_map_db_[i]);
        multi_pgo.run();
    }
}

void UniMapServer::saveUpdatedPoses() {
    for (int i = 0; i < multi_map_db_.size(); i++) {

        std::fstream updated_pose = std::fstream(save_dir_path_ + "updated_" + data_dir_path_vec_[i] + ".txt", std::fstream::out);
        updated_pose << std::fixed << std::setprecision(9);

        nav_msgs::Path path = multi_map_db_[i]->updated_path_;

        for(int j = 0; j < multi_map_db_[i]->total_frame_num_; j++) {
            geometry_msgs::PoseStamped pose = path.poses[j];
            Eigen::Affine3d T_wc = Eigen::Affine3d::Identity();
            Eigen::Vector3d t_wc(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Quaterniond q_wc(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
            Eigen::Matrix3d r_wc = q_wc.normalized().toRotationMatrix();
            T_wc.translate(t_wc);
            T_wc.rotate(r_wc);

            for(int row = 0; row < 3; row++){
                for(int col=0; col < 4; col++){
                    updated_pose << T_wc(row,col);

                    if(row==2 && col==3) { updated_pose << std::endl;}
                    else{updated_pose << " ";}
                }
            }

        }
    }
}

void UniMapServer::pubPCL() {
    pcl::VoxelGrid<PointType> ds_filter;
    // leaf size config
    ds_filter.setLeafSize(0.5, 0.5, 0.5);

//    #pragma omp parallel for num_threads(multi_map_db_.size())
    for (int i = 0; i < multi_map_db_.size(); i++) {
        nav_msgs::Path path = multi_map_db_[i]->updated_path_;

        for(int j = 0; j < multi_map_db_[i]->total_frame_num_; j++) {
            geometry_msgs::PoseStamped pose = path.poses[j];
            Eigen::Affine3d T_wc = Eigen::Affine3d::Identity();
            Eigen::Vector3d t_wc(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
            Eigen::Quaterniond q_wc(pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
            Eigen::Matrix3d r_wc = q_wc.normalized().toRotationMatrix();
            T_wc.translate(t_wc);
            T_wc.rotate(r_wc);

            std::string pcd_file_path = multi_map_db_[i]->pcd_paths_vec_[j];
            pcl::PointCloud<PointType>::Ptr curr_pcd(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile<PointType>(pcd_file_path, *curr_pcd);
            ds_filter.setInputCloud(curr_pcd);
            ds_filter.filter(*curr_pcd);

            pcl::transformPointCloud(*curr_pcd, *curr_pcd, T_wc);
            publishCloud(&pub_pcd_vec_[i], curr_pcd, ros::Time().now(), "camera_init");

            // ros::Rate loop(100);
            // loop.sleep();
        }
    }
}

void UniMapServer::vizTrajectory() {
    while (ros::ok()) {
        if (pub_origin_path_.getNumSubscribers() != 0) {
            multi_map_db_[curr_single_map_idx_]->original_path_.header.stamp = ros::Time().now();
            multi_map_db_[curr_single_map_idx_]->original_path_.header.frame_id = "camera_init";
            pub_origin_path_.publish(multi_map_db_[curr_single_map_idx_]->original_path_);
        }

        if (pub_loop_edge_.getNumSubscribers() != 0) {
            vizTmpLoopClosure(&pub_loop_edge_, multi_map_db_[curr_single_map_idx_]->updated_path_ ,
                           multi_map_db_[curr_single_map_idx_]->single_loop_pair_vec_);
        }

        for (int i = 0; i <= curr_single_map_idx_; i++) {
            multi_map_db_[i]->updated_path_.header.stamp = ros::Time().now();
            multi_map_db_[i]->updated_path_.header.frame_id = "camera_init";
            pub_updated_path_vec_[i].publish(multi_map_db_[i]->updated_path_);

            vizMultiLoopClosure(&pub_loop_marker_vec_[i], multi_map_db_[0]->updated_path_, multi_map_db_[i]->updated_path_ ,
                           multi_map_db_[i]->multi_loop_pair_vec_, i);
        }

        ros::Rate loop(3);
        loop.sleep();
    }
}

