//
// Created by gil on 23. 11. 15.
//

#include "InterLoopPGO.h"

//InterLoopPGO::InterLoopPGO() {}
InterLoopPGO::~InterLoopPGO() = default;

InterLoopPGO::InterLoopPGO(SingleMapDB* _central_db, SingleMapDB* _query_db)
:central_value_(_central_db->values_),
central_pose_graph_(_central_db->pose_graph_),
query_value_(_query_db->values_),
query_pose_graph_(_query_db->pose_graph_),
central_path_(_central_db->updated_path_),
query_path_(_query_db->updated_path_),
multi_loop_pair_vec_(_query_db->multi_loop_pair_vec_)
{
    central_std_manager_ = _central_db->std_manager_;
    query_std_manager_ = _query_db->std_manager_;

    central_frame_num_ = _central_db->total_frame_num_;
    query_frame_num_ = _query_db->total_frame_num_;

    central_graph_start_idx_ = _central_db->graph_start_idx_;
    query_graph_start_idx_ = _query_db->graph_start_idx_;
    robust_loop_noise_ = _central_db->robust_loop_noise_;
    icp_noise_ = gtsam::noiseModel::Diagonal::Variances(
            (gtsam::Vector(6) << 1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3).finished());

    central_pcd_paths_vec_ = _central_db->pcd_paths_vec_;
    query_pcd_paths_vec_ = _query_db->pcd_paths_vec_;
}

void InterLoopPGO::initGTSAM() {
    gtsam::ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.relinearizeSkip = 1;
    isam_ = new gtsam::ISAM2(parameters);
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    gtsam::GncParams<gtsam::LevenbergMarquardtParams> gncParams;
    gncParams.maxIterations = 100;
    gncParams.muStep = 1.4;
}

void InterLoopPGO::run() {
    initGTSAM();
    combineToCentralGraph();
    interSTDLoopClosing();
    optimizeInterGraph();
    updateMultiPath();

    interRSLoopClosing();
    optimizeInterGraph();
    updateMultiPath();
    // std::cout << "anchor_symbol: " << std::endl << shared_data->values.at<gtsam::Pose3>(anchor_symbol).matrix() << std::endl;
    for(const auto& key : central_value_.keys()) {
        int graph_idx = key;
        if(graph_idx % GRAPH_IDX_SCALE == 0){
            std::cout << "anchor_symbol: " << std::endl << central_value_.at<gtsam::Pose3>(graph_idx).matrix() << std::endl;
        }
    }



}

void InterLoopPGO::combineToCentralGraph() {
    for (const auto& factor : query_pose_graph_) {
        central_pose_graph_.add(factor);
    }
    for (const auto& value : query_value_) {
        central_value_.insert(value.key, value.value);
    }
}

void InterLoopPGO::interSTDLoopClosing() {
    // TODO : hard coded num_thread to yaml
    #pragma omp parallel for num_threads(16)
    for (int query_frame_idx = 0; query_frame_idx < query_frame_num_; query_frame_idx++) {
        std::pair<int, double> search_result(-1, 0);
        std::pair<Eigen::Vector3d, Eigen::Matrix3d> loop_transform;
        loop_transform.first << 0, 0, 0;
        loop_transform.second = Eigen::Matrix3d::Identity();
        std::vector<std::pair<STDesc, STDesc>> loop_std_pair;

        query_std_manager_->config_setting_.skip_near_num_ = 0;
        central_std_manager_->config_setting_.skip_near_num_ = 0;


        std::vector<STDesc> query_std = query_std_manager_->std_vec_db_[query_frame_idx];
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr query_plane_vec = query_std_manager_->plane_cloud_vec_[query_frame_idx];

        central_std_manager_->SearchLoop(query_std, search_result, loop_transform, loop_std_pair, query_plane_vec);
        int matched_frame = search_result.first;

        auto delta_T = Eigen::Affine3d::Identity();

        if (matched_frame > 0) {
            // obtain optimal transform
            query_std_manager_->PlaneGeomrtricIcp(query_std_manager_->plane_cloud_vec_[query_frame_idx],
                                                 central_std_manager_->plane_cloud_vec_[matched_frame],
                                                 loop_transform);
            delta_T.translate(loop_transform.first);
            delta_T.rotate(loop_transform.second);

            mtx_.lock();
            gtsam::Pose3 relative_pose = gtsam::Pose3(delta_T.matrix());
            central_pose_graph_.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
                    genGraphIdx(central_graph_start_idx_, matched_frame), genGraphIdx(query_graph_start_idx_, query_frame_idx),
                    central_graph_start_idx_, query_graph_start_idx_,
                    relative_pose, robust_loop_noise_));

            std::cout << "[STD ICP loop found] : " << "query :" << query_frame_idx << ", center :" << matched_frame << "(Score : " << search_result.second << ")" <<std::endl;

            multi_loop_pair_vec_.emplace_back(query_frame_idx, matched_frame);
            mtx_.unlock();
        }
    }
}

void InterLoopPGO::interRSLoopClosing() {
    gtsam::Pose3 prev_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

    std::vector<std::pair<int,int>> rs_loop_graph_idx;

    for (int query_frame_idx = 0; query_frame_idx < query_frame_num_; query_frame_idx++) {
        int query_graph_idx = genGraphIdx(query_graph_start_idx_, query_frame_idx);
        gtsam::Pose3 query_sampled_pose = central_value_.at<gtsam::Pose3>(query_graph_idx);
        // TODO : pose distance : x,y 거리임
        if (poseDistance(prev_pose, query_sampled_pose) < rs_dist_)
            continue;

        prev_pose = query_sampled_pose;
        // central map 기준으로 coordinate 변환한 qeury pose
        gtsam::Pose3 query_transformed_pose =
                central_value_.at<gtsam::Pose3>(query_graph_start_idx_) * query_sampled_pose;

        float tmp_dist = 10000;
        int tmp_idx;
        // query sampled pose와 가장 가까운 target idx 구하기 (by for문)
        for (int tgt_frame_idx = 0; tgt_frame_idx < central_frame_num_; tgt_frame_idx++) {
            gtsam::Pose3 tgt_pose = central_value_.at<gtsam::Pose3>(
                    genGraphIdx(central_graph_start_idx_, tgt_frame_idx));
            auto rel_dist = poseDistance(tgt_pose, query_transformed_pose);
            if (rel_dist < tmp_dist) {
                tmp_dist = rel_dist;
                tmp_idx = tgt_frame_idx;
            }
        }

        if (tmp_dist > rs_dist_th_)
            continue;

        int nearest_tgt_frmae = tmp_idx;
        rs_loop_graph_idx.emplace_back(nearest_tgt_frmae, query_frame_idx);
    }

    #pragma omp parallel for num_threads(16)
    for(auto pairs : rs_loop_graph_idx) {

        int nearest_tgt_frmae = pairs.first;
        int nearest_tgt_graph_idx = genGraphIdx(central_graph_start_idx_, nearest_tgt_frmae);
        auto nearest_tgt_pose = central_value_.at<gtsam::Pose3>(nearest_tgt_graph_idx);

        int query_frame_idx = pairs.second;
        int query_graph_idx = genGraphIdx(query_graph_start_idx_, query_frame_idx);
        auto query_sampled_pose = central_value_.at<gtsam::Pose3>(query_graph_idx);
        auto query_transformed_pose = central_value_.at<gtsam::Pose3>(query_graph_start_idx_) * query_sampled_pose;

        // target idx 주변 N개 pointcloud 쌓기
        pcl::PointCloud<PointType>::Ptr tgt_acc_cloud(new pcl::PointCloud<PointType>());
        int search_num = 5;
        for (int i = -search_num; i < search_num; ++i) {
            int key_near = nearest_tgt_frmae + i;
            if (key_near < 0 || key_near >= central_frame_num_ )
                continue;

            pcl::PointCloud<PointType>::Ptr key_near_pcd(new pcl::PointCloud<PointType>);
            pcl::io::loadPCDFile<PointType>(central_pcd_paths_vec_[key_near], *key_near_pcd);
            auto key_near_pose = central_value_.at<gtsam::Pose3>(genGraphIdx(central_graph_start_idx_, key_near));
            pcl::transformPointCloud(*key_near_pcd, *key_near_pcd, key_near_pose.matrix());
            *tgt_acc_cloud += *key_near_pcd;
        }

        down_sampling_voxel(*tgt_acc_cloud, 0.3);
        Eigen::Matrix4d inv = nearest_tgt_pose.matrix().inverse();
        pcl::transformPointCloud(*tgt_acc_cloud, *tgt_acc_cloud, inv);


        pcl::PointCloud<PointType>::Ptr src_cloud(new pcl::PointCloud<PointType>());
        pcl::io::loadPCDFile<PointType>(query_pcd_paths_vec_[query_frame_idx], *src_cloud);
        auto T_tgt_src = nearest_tgt_pose.between(query_transformed_pose);
        auto T_src_tgt = T_tgt_src.inverse();
        pcl::transformPointCloud(*src_cloud, *src_cloud, T_tgt_src.matrix());       //! 왜 tgt_src가 맞음??
        down_sampling_voxel(*src_cloud, 0.3);

        // gicp
        pcl::GeneralizedIterativeClosestPoint<PointType, PointType> gicp;
        gicp.setMaxCorrespondenceDistance(150); // giseop , use a value can cover 2*historyKeyframeSearchNum range in meter
        gicp.setMaximumIterations(100);
        gicp.setTransformationEpsilon(1e-6);
        gicp.setEuclideanFitnessEpsilon(1e-6);
        gicp.setRANSACIterations(0);

        // Align pointclouds
        gicp.setInputSource(src_cloud);
        gicp.setInputTarget(tgt_acc_cloud);
        pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
        gicp.align(*unused_result);

        if(gicp.getFitnessScore() > 1.0){
            continue;
        }

        multi_loop_pair_vec_.emplace_back(query_frame_idx, nearest_tgt_frmae);

        Eigen::Matrix4d T_icp = gicp.getFinalTransformation().cast<double>();
        Eigen::Matrix4d T_final = T_icp * T_tgt_src.matrix();

        pcl::PointCloud<PointType>::Ptr icp_cloud(new pcl::PointCloud<PointType>());
        pcl::io::loadPCDFile<PointType>(query_pcd_paths_vec_[query_frame_idx], *icp_cloud);
        pcl::transformPointCloud(*icp_cloud, *icp_cloud, T_final);


        // query idx의 pointcloud의 initial relative pose 주기
        mtx_.lock();
        gtsam::Pose3 relative_icp_pose = gtsam::Pose3(T_final);
        gtsam::Pose3 test_pose(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));


//        central_pose_graph_.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
//                nearest_tgt_graph_idx, query_graph_idx,
//                central_graph_start_idx_, query_graph_start_idx_,
//                relative_icp_pose, robust_loop_noise_));

        central_pose_graph_.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
                nearest_tgt_graph_idx, query_graph_idx,
                central_graph_start_idx_, query_graph_start_idx_,
                relative_icp_pose, icp_noise_));
//        central_pose_graph_.add( gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
//                nearest_tgt_graph_idx, query_graph_idx,
//                central_graph_start_idx_, query_graph_start_idx_,
//                relative_icp_pose, robust_loop_noise_));




        std::cout << "[RS ICP loop found] : " << "center :" << nearest_tgt_frmae << ", center :" << query_frame_idx << "(Score : " << gicp.getFitnessScore() << ")" <<std::endl;
//        std::cout << "[RS ICP loop found] : " << gicp.getFitnessScore() << std::endl;
        mtx_.unlock();


//        pcl::visualization::PCLVisualizer viewer("ICP result");
//        viewer.setBackgroundColor(0, 0, 0);
//        viewer.addCoordinateSystem(1.0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> red_handler(src_cloud, 255, 0, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> blue_handler(tgt_acc_cloud_ds, 0, 0, 255);
////        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> green_handler(unused_result, 0, 255, 0);
//        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> green_handler(icp_cloud, 0, 255, 0);
////        viewer.addPointCloud<pcl::PointXYZI>(src_cloud, red_handler, "src_cloud");
//        viewer.addPointCloud<pcl::PointXYZI>(tgt_acc_cloud_ds, blue_handler,"target_cloud");
////        viewer.addPointCloud<pcl::PointXYZI>(unused_result, green_handler,"aligned_cloud");
//        viewer.addPointCloud<pcl::PointXYZI>(icp_cloud, green_handler,"aligned_cloud");
//        viewer.spin();
//        viewer.close();
    }

}


void InterLoopPGO::optimizeInterGraph() {
    if(isam_->size() == 0) {
        isam_->update(central_pose_graph_, central_value_);
    } else{
        isam_->update(central_pose_graph_, gtsam::Values());
    }

    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();
    isam_->update();

//    central_pose_graph_.resize(0);

    auto t_update_begin = std::chrono::high_resolution_clock::now();
    central_value_ = isam_->calculateBestEstimate();
    auto t_update_end = std::chrono::high_resolution_clock::now();
    std::cout << "[Time] : " << (t_update_end - t_update_begin).count() * 1000 << std::endl;

//    central_pose_graph_.resize(0);
//    central_value_.clear();
}

void InterLoopPGO::updateMultiPath() {
    nav_msgs::Path updated_central_path;
    nav_msgs::Path updated_query_path;

    for(const auto& key : central_value_.keys()) {
        int graph_idx = key;
        int frame_idx = graph_idx % GRAPH_IDX_SCALE;
        if(graph_idx / GRAPH_IDX_SCALE == central_graph_start_idx_ / GRAPH_IDX_SCALE){
            if(frame_idx > 0) {
                updated_central_path.poses.push_back(gtsamPose3ToPoseStamped(central_value_.at<gtsam::Pose3>(graph_idx)));
            }
        } else if (graph_idx / GRAPH_IDX_SCALE == query_graph_start_idx_ / GRAPH_IDX_SCALE) {
            if(frame_idx > 0) {
                updated_query_path.poses.push_back(gtsamPose3ToPoseStamped(central_value_.at<gtsam::Pose3>(query_graph_start_idx_) * central_value_.at<gtsam::Pose3>(graph_idx)));
            }
        }
    }
    std::cout << "path update!!!!!!!!!!!!!!" << std::endl;
//    central_path_.poses.clear();
//    query_path_.poses.clear();
    central_path_ = updated_central_path;
    query_path_ = updated_query_path;
}






