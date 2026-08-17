#pragma once

#include <rclcpp/rclcpp.hpp>
#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/server/pipeline_controller.hpp>

namespace open_lmm {

class OpenLMMROS : public rclcpp::Node {
public:
  OpenLMMROS(const rclcpp::NodeOptions &options);
  ~OpenLMMROS() = default;
  [[nodiscard]] bool GuiEnabled() const { return gui_host_ != nullptr; }

private:
  std::shared_ptr<open_lmm::PipelineController> controller_;
  std::unique_ptr<open_lmm::GuiPluginHost> gui_host_;
};

} // namespace open_lmm
