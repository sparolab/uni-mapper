#pragma once

#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/server/pipeline_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace open_lmm {

class OpenLMMROS : public rclcpp::Node {
public:
  OpenLMMROS(const rclcpp::NodeOptions &options);
  ~OpenLMMROS() override;
  [[nodiscard]] bool GuiEnabled() const { return gui_host_ != nullptr; }
  [[nodiscard]] PipelineSnapshot Snapshot() const;

private:
  using Trigger = std_srvs::srv::Trigger;

  void HandleStart(const std::shared_ptr<Trigger::Request>& request,
                   std::shared_ptr<Trigger::Response> response);
  void HandleCancel(const std::shared_ptr<Trigger::Request>& request,
                    std::shared_ptr<Trigger::Response> response);
  void HandleStatus(const std::shared_ptr<Trigger::Request>& request,
                    std::shared_ptr<Trigger::Response> response) const;
  void PublishEvent(const ExecutionEvent& event);

  std::shared_ptr<open_lmm::PipelineController> controller_;
  std::unique_ptr<open_lmm::GuiPluginHost> gui_host_;
  ExecutionEventSubscription event_subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr progress_publisher_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr result_publisher_;
  rclcpp::Service<Trigger>::SharedPtr start_service_;
  rclcpp::Service<Trigger>::SharedPtr cancel_service_;
  rclcpp::Service<Trigger>::SharedPtr status_service_;
};

} // namespace open_lmm
