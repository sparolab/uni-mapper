#pragma once
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <vector>

Eigen::Isometry3d kittiPoseToIsometry3d(std::vector<double>& values);
Eigen::Isometry3d tumPoseToIsometry3d(std::vector<double>& values);
Eigen::Isometry3d customPoseToIsometry3d(std::vector<double>& values);

std::vector<double> isometry3dToKittiPose(const Eigen::Isometry3d& pose);
std::vector<double> isometry3dToTumPose(const Eigen::Isometry3d& pose);
std::vector<double> isometry3dToCustomPose(const Eigen::Isometry3d& pose);
