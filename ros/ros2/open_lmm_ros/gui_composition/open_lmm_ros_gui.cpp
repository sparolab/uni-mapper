#include "open_lmm_ros_gui.hpp"

#include <ament_index_cpp/get_package_prefix.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <filesystem>
#include <stdexcept>
#include <utility>

#include <open_lmm/gui/gui_runtime_host.hpp>

namespace open_lmm {

OpenLMMROSGui::OpenLMMROSGui(const rclcpp::NodeOptions& options)
    : OpenLMMROS(options) {
  this->declare_parameter<std::string>("gui_plugin_path", "");
  this->declare_parameter<bool>("gui_enabled", false);
  auto gui_plugin_path = this->get_parameter("gui_plugin_path").as_string();
  const bool gui_enabled = this->get_parameter("gui_enabled").as_bool() ||
                           !gui_plugin_path.empty();
  if (!gui_enabled) return;

  if (gui_plugin_path.empty()) {
    gui_plugin_path =
        (std::filesystem::path(
             ament_index_cpp::get_package_prefix("open_lmm")) /
         "lib" / "libopen_lmm_iridescence_gui.so")
            .string();
  }
  if (!std::filesystem::is_regular_file(gui_plugin_path)) {
    throw std::runtime_error("GUI plugin was not found at " + gui_plugin_path);
  }
  auto loaded = GuiRuntimeHost::LoadAndStart(
      gui_plugin_path, Runtime(), (ConfigPath() / "config.json").string());
  if (!loaded) {
    throw std::runtime_error("GUI load failed: " +
                             loaded.GetError().Message());
  }
  gui_host_ = std::move(loaded).Value();
}

OpenLMMROSGui::~OpenLMMROSGui() = default;

}  // namespace open_lmm

RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROSGui)
