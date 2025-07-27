//
// Created by gil on 23. 11. 16.
//

#include "utility.h"



int genGraphIdx (int _map_idx, int _node_offset) {
    return _map_idx + _node_offset + 1;
}

void update_values(gtsam::Values _values, nav_msgs::Path _updated_path){
    for (const gtsam::Key idx : _values.keys()) {
        auto pose = _values.at<gtsam::Pose3>(idx);

        _updated_path.poses.push_back(gtsamPose3ToPoseStamped(pose));

//        _updated_path.poses.push_back(pose_stamped);
    }
}

geometry_msgs::PoseStamped gtsamPose3ToPoseStamped(gtsam::Pose3 _pose) {
    geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.stamp = ros::Time().now();
    pose_stamped.header.frame_id = "camera_init";
    pose_stamped.pose.position.x = _pose.translation().x();
    pose_stamped.pose.position.y = _pose.translation().y();
    pose_stamped.pose.position.z = _pose.translation().z();
    pose_stamped.pose.orientation.x = _pose.rotation().quaternion().x();
    pose_stamped.pose.orientation.y = _pose.rotation().quaternion().y();
    pose_stamped.pose.orientation.z = _pose.rotation().quaternion().z();
    pose_stamped.pose.orientation.w = _pose.rotation().quaternion().w();

    return pose_stamped;
}



void publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame) {
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
}


void vizMultiLoopClosure(ros::Publisher *thisPub, nav_msgs::Path _central_path, nav_msgs::Path _query_path, std::vector<std::pair<int, int>> _loop_pair_vec, int _map_idx)
{
    if (_loop_pair_vec.size() == 0 )
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "camera_init";
    markerNode.header.stamp = ros::Time().now();
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes_" + std::to_string(_map_idx);
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    markerEdge.header.stamp = ros::Time().now();
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges_" + std::to_string(_map_idx);
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1; markerEdge.scale.y = 0.1; markerEdge.scale.z = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto loop_pair : _loop_pair_vec){
        int key_query = loop_pair.first;
        int key_central = loop_pair.second;
        geometry_msgs::Point p;
        p.x = _query_path.poses[key_query].pose.position.x;
        p.y = _query_path.poses[key_query].pose.position.y;
        p.z = _query_path.poses[key_query].pose.position.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = _central_path.poses[key_central].pose.position.x;
        p.y = _central_path.poses[key_central].pose.position.y;
        p.z = _central_path.poses[key_central].pose.position.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    thisPub->publish(markerArray);
}


void vizTmpLoopClosure(ros::Publisher *thisPub, nav_msgs::Path _path, std::vector<std::pair<int, int>> _loop_pair_vec)
{
    if (_path.poses.size() == 0 )
        return;

    visualization_msgs::MarkerArray markerArray;
    // loop nodes
    visualization_msgs::Marker markerNode;
    markerNode.header.frame_id = "camera_init";
    markerNode.header.stamp = ros::Time().now();
    markerNode.action = visualization_msgs::Marker::ADD;
    markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
    markerNode.ns = "loop_nodes";
    markerNode.id = 0;
    markerNode.pose.orientation.w = 1;
    markerNode.scale.x = 0.3; markerNode.scale.y = 0.3; markerNode.scale.z = 0.3;
    markerNode.color.r = 0; markerNode.color.g = 0.8; markerNode.color.b = 1;
    markerNode.color.a = 1;
    // loop edges
    visualization_msgs::Marker markerEdge;
    markerEdge.header.frame_id = "camera_init";
    markerEdge.header.stamp = ros::Time().now();
    markerEdge.action = visualization_msgs::Marker::ADD;
    markerEdge.type = visualization_msgs::Marker::LINE_LIST;
    markerEdge.ns = "loop_edges";
    markerEdge.id = 1;
    markerEdge.pose.orientation.w = 1;
    markerEdge.scale.x = 0.1; markerEdge.scale.y = 0.1; markerEdge.scale.z = 0.1;
    markerEdge.color.r = 0.9; markerEdge.color.g = 0.9; markerEdge.color.b = 0;
    markerEdge.color.a = 1;

    for (auto loop_pair : _loop_pair_vec){
        int key_cur = loop_pair.first;
        int key_pre = loop_pair.second;
        geometry_msgs::Point p;
        p.x = _path.poses[key_cur].pose.position.x;
        p.y = _path.poses[key_cur].pose.position.y;
        p.z = _path.poses[key_cur].pose.position.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
        p.x = _path.poses[key_pre].pose.position.x;
        p.y = _path.poses[key_pre].pose.position.y;
        p.z = _path.poses[key_pre].pose.position.z;
        markerNode.points.push_back(p);
        markerEdge.points.push_back(p);
    }

    markerArray.markers.push_back(markerNode);
    markerArray.markers.push_back(markerEdge);
    thisPub->publish(markerArray);
}

float poseDistance(const gtsam::Pose3& p1, const gtsam::Pose3& p2)
{
    auto p1x = p1.translation().x();
    auto p1y = p1.translation().y();
    auto p1z = p1.translation().z();
    auto p2x = p2.translation().x();
    auto p2y = p2.translation().y();
    auto p2z = p2.translation().z();

//    return sqrt((p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y) + (p1z-p2z)*(p1z-p2z));
    return sqrt((p1x-p2x)*(p1x-p2x) + (p1y-p2y)*(p1y-p2y));
}

