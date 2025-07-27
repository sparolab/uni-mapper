// STL
#include <iostream>

// ROS2
#include <ament_index_cpp/get_package_prefix.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

// open_lmm
#include <open_lmm/server/map_server.hpp>
#include <open_lmm/utils/config.hpp>
// open_lmm_ros
#include "open_lmm_ros.hpp"

namespace open_lmm {

OpenLMMROS::OpenLMMROS(const rclcpp::NodeOptions &options)
    : Node("open_lmm_ros", options) {
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path[0] != '/') {
    config_path = ament_index_cpp::get_package_share_directory("open_lmm") +
                  "/" + config_path;
  }

  open_lmm::GlobalConfig::instance(config_path);

  open_lmm::MapServer map_server;
  map_server.process();

  // TODO(gil) : rviz visualization
}

} // namespace open_lmm
RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS);
