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
#include <open_lmm/gui/gui_controller_bridge.hpp>
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

  this->declare_parameter<std::string>("gui_plugin_path", "");
  this->declare_parameter<bool>("gui_auto_run", false);
  const auto gui_plugin_path =
      this->get_parameter("gui_plugin_path").as_string();
  const bool gui_auto_run = this->get_parameter("gui_auto_run").as_bool();

  auto map_server = std::make_shared<open_lmm::MapServer>();
  if (gui_plugin_path.empty()) {
    auto result = map_server->process();
    if (!result) {
      throw std::runtime_error("MapServer failed: " +
                               result.GetError().Message());
    }
    return;
  }

  controller_ = std::make_shared<open_lmm::PipelineController>(map_server);
  auto loaded = open_lmm::GuiPluginHost::Load(gui_plugin_path);
  if (!loaded) {
    throw std::runtime_error("GUI load failed: " +
                             loaded.GetError().Message());
  }
  gui_host_ = std::move(loaded).Value();
  auto started = gui_host_->Start(open_lmm::MakeGuiServices(controller_));
  if (!started) {
    throw std::runtime_error("GUI start failed: " +
                             started.GetError().Message());
  }
  if (gui_auto_run) {
    auto submitted = controller_->SubmitRunAll();
    if (!submitted) {
      throw std::runtime_error("GUI auto-run failed: " +
                               submitted.GetError().Message());
    }
  }
}

} // namespace open_lmm
RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS);
