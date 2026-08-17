#pragma once

#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace open_lmm {

inline std::string format_config_number(double value) {
  std::ostringstream stream;
  stream << std::fixed << std::setprecision(6) << value;
  return stream.str();
}

// Convertion to string

template <typename T>
std::string convert_to_string(const T& value) {
  std::ostringstream stream;
  stream << std::boolalpha << value;
  return stream.str();
}

template <typename T2>
std::string convert_to_string(const std::vector<T2>& values) {
  std::stringstream sst;
  sst << "[";
  for (unsigned int i = 0; i < values.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << convert_to_string(values[i]);
  }
  sst << "]";
  return sst.str();
}

template <int D>
std::string convert_to_string(const Eigen::Matrix<double, D, 1>& value) {
  std::stringstream sst;
  sst << "vec(";
  for (unsigned int i = 0; i < value.size(); i++) {
    if (i) {
      sst << ",";
    }
    sst << format_config_number(value[i]);
  }
  sst << ")";
  return sst.str();
}

template <>
inline std::string convert_to_string(const Eigen::Quaterniond& quat) {
  return "quat(" + format_config_number(quat.x()) + "," +
         format_config_number(quat.y()) + "," +
         format_config_number(quat.z()) + "," +
         format_config_number(quat.w()) + ")";
}

template <>
inline std::string convert_to_string(const Eigen::Isometry3d& pose) {
  const Eigen::Vector3d trans(pose.translation());
  const Eigen::Quaterniond quat(pose.linear());
  return "se3(" + format_config_number(trans.x()) + "," +
         format_config_number(trans.y()) + "," +
         format_config_number(trans.z()) + "," +
         format_config_number(quat.x()) + "," +
         format_config_number(quat.y()) + "," +
         format_config_number(quat.z()) + "," +
         format_config_number(quat.w()) + ")";
}
}  // namespace open_lmm
