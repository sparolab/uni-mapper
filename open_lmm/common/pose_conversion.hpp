#pragma once

#include <Eigen/Geometry>
#include <vector>

Eigen::Isometry3d kittiPoseToIsometry3d(std::vector<double>& values);
Eigen::Isometry3d tumPoseToIsometry3d(std::vector<double>& values);
