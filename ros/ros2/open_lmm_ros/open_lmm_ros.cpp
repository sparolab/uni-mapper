// STL
#include <filesystem>
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
#include <open_lmm/gui/config_editor.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/logging.hpp>
// open_lmm_ros
#include "open_lmm_ros.hpp"

namespace open_lmm {

OpenLMMROS::OpenLMMROS(const rclcpp::NodeOptions &options)
    : Node("open_lmm_ros", options) {
  InitializeLogging();
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
  this->declare_parameter<bool>("gui_enabled", false);
  this->declare_parameter<bool>("gui_auto_run", false);
  auto gui_plugin_path =
      this->get_parameter("gui_plugin_path").as_string();
  const bool gui_enabled = this->get_parameter("gui_enabled").as_bool() ||
                           !gui_plugin_path.empty();
  const bool gui_auto_run = this->get_parameter("gui_auto_run").as_bool();

  auto map_server = std::make_shared<open_lmm::MapServer>();
  if (!gui_enabled) {
    auto result = map_server->process();
    if (!result) {
      throw std::runtime_error("MapServer failed: " +
                               result.GetError().Message());
    }
    return;
  }

  if (gui_plugin_path.empty()) {
    gui_plugin_path =
        (std::filesystem::path(
             ament_index_cpp::get_package_prefix("open_lmm")) /
         "lib" / "libopen_lmm_iridescence_gui.so")
            .string();
  }
  if (!std::filesystem::is_regular_file(gui_plugin_path)) {
    throw std::runtime_error(
        "GUI plugin was not found at " + gui_plugin_path +
        ". Rebuild with OPEN_LMM_BUILD_IRIDESCENCE_GUI=ON or set "
        "gui_plugin_path to a custom plugin.");
  }

  controller_ = std::make_shared<open_lmm::PipelineController>(map_server);
  auto loaded = open_lmm::GuiPluginHost::Load(gui_plugin_path);
  if (!loaded) {
    throw std::runtime_error("GUI load failed: " +
                             loaded.GetError().Message());
  }
  gui_host_ = std::move(loaded).Value();
  auto services = open_lmm::MakeGuiServices(
      controller_, (std::filesystem::path(config_path) / "config.json").string());
  services.create_session = [controller = controller_](const std::string& file) {
    const std::filesystem::path config_file(file);
    if (config_file.filename() != "config.json" ||
        !std::filesystem::is_regular_file(config_file)) {
      return open_lmm::Result<void>::Failure(open_lmm::Error::InvalidArgument(
          "select an existing config.json file"));
    }
    auto document = open_lmm::ConfigEditorDocument::Load(config_file);
    if (!document) return open_lmm::Result<void>::Failure(document.GetError());
    const auto previous_directory = open_lmm::GlobalConfig::config_directory();
    auto reloaded = open_lmm::GlobalConfig::reload(config_file.parent_path().string());
    if (!reloaded) return reloaded;
    auto runner = std::make_shared<open_lmm::MapServer>();
    auto ready = runner->ValidateReady();
    if (!ready) {
      if (!previous_directory.empty())
        open_lmm::GlobalConfig::reload(previous_directory);
      return ready;
    }
    auto replaced = controller->ReplaceRunner(runner);
    if (!replaced && !previous_directory.empty())
      open_lmm::GlobalConfig::reload(previous_directory);
    return replaced;
  };
  auto started = gui_host_->Start(std::move(services));
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
