//
// Created by gil on 23. 11. 14.
//

#ifndef UNI_MAPPER_UTILITY_H
#define UNI_MAPPER_UTILITY_H

// CPP basic
#include <iostream>
#include <thread>
#include <mutex>

// ROS
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <pcl/visualization/pcl_visualizer.h>

// gtSAM
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>



#define GRAPH_IDX_SCALE 100000

namespace fs = boost::filesystem;
using PointType = pcl::PointXYZI;

int genGraphIdx (int _map_idx, int _frmae_idx);

void update_values(gtsam::Values, nav_msgs::Path);

geometry_msgs::PoseStamped gtsamPose3ToPoseStamped(gtsam::Pose3);

void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

void vizTmpLoopClosure(ros::Publisher *thisPub, nav_msgs::Path, std::vector<std::pair<int, int>>);
void vizMultiLoopClosure(ros::Publisher *thisPub, nav_msgs::Path, nav_msgs::Path, std::vector<std::pair<int, int>>, int);

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2);

typedef struct STDConfig {
    /* for point cloud pre-preocess*/
    int stop_skip_enable_ = 0;
    double ds_size_ = 0.5;
    int maximum_corner_num_ = 30;

    /* for key points*/
    double plane_merge_normal_thre_;
    double plane_merge_dis_thre_;
    double plane_detection_thre_ = 0.01;
    double voxel_size_ = 1.0;
    int voxel_init_num_ = 10;
    double proj_image_resolution_ = 0.5;
    double proj_dis_min_ = 0.2;
    double proj_dis_max_ = 5;
    double corner_thre_ = 10;

    /* for STD */
    int descriptor_near_num_ = 10;
    double descriptor_min_len_ = 1;
    double descriptor_max_len_ = 10;
    double non_max_suppression_radius_ = 3.0;
    double std_side_resolution_ = 0.2;

    /* for place recognition*/
    int skip_near_num_ = 50;
    int candidate_num_ = 50;
    int sub_frame_num_ = 10;
    double rough_dis_threshold_ = 0.03;
    double vertex_diff_threshold_ = 0.7;
    double icp_threshold_ = 0.5;
    double normal_threshold_ = 0.1;
    double dis_threshold_ = 0.3;
} STDConfig;

#endif //UNI_MAPPER_UTILITY_H
