// STL
#include <iostream>
#include <stdexcept>

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

  auto* global_config = open_lmm::GlobalConfig::instance(config_path);
  if (!global_config->is_valid()) {
    throw std::runtime_error("Global config failed: " +
                             global_config->error_message());
  }

  open_lmm::MapServer map_server;
  auto result = map_server.process();
  if (!result) {
    throw std::runtime_error("MapServer failed: " + result.GetError().Message());
  }

  // TODO(gil) : rviz visualization
}

} // namespace open_lmm
RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS);
