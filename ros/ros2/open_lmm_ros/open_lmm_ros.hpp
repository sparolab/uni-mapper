#pragma once

#include "goal_admission.hpp"
#include <open_lmm/gui/gui_runtime_host.hpp>
#include <open_lmm/server/runtime_client.hpp>
#include <open_lmm_ros/action/execute_pipeline.hpp>
#include <open_lmm_ros/msg/execution_event.hpp>
#include <open_lmm_ros/srv/get_runtime_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace open_lmm {

class OpenLMMROS : public rclcpp::Node {
 public:
  using ExecutePipeline = open_lmm_ros::action::ExecutePipeline;
  using GoalHandleExecutePipeline =
      rclcpp_action::ServerGoalHandle<ExecutePipeline>;
  using GetRuntimeStatus = open_lmm_ros::srv::GetRuntimeStatus;

  explicit OpenLMMROS(const rclcpp::NodeOptions& options);
  ~OpenLMMROS() override;
  [[nodiscard]] bool GuiEnabled() const { return gui_host_ != nullptr; }
  [[nodiscard]] PipelineSnapshot Snapshot() const;

 private:
  rclcpp_action::GoalResponse HandleGoal(
      const rclcpp_action::GoalUUID& uuid,
      std::shared_ptr<const ExecutePipeline::Goal> goal);
  rclcpp_action::CancelResponse HandleCancel(
      const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle);
  void HandleAccepted(
      const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle);
  void ExecuteGoal(
      const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle);
  void HandleStatus(const std::shared_ptr<GetRuntimeStatus::Request>& request,
                    std::shared_ptr<GetRuntimeStatus::Response> response) const;
  void PublishEvent(const ExecutionEvent& event);

  std::shared_ptr<RuntimeClient> runtime_;
  std::unique_ptr<GuiRuntimeHost> gui_host_;
  ExecutionEventSubscription event_subscription_;
  rclcpp::Publisher<open_lmm_ros::msg::ExecutionEvent>::SharedPtr
      event_publisher_;
  rclcpp_action::Server<ExecutePipeline>::SharedPtr execute_action_;
  rclcpp::Service<GetRuntimeStatus>::SharedPtr status_service_;
  mutable std::mutex action_mutex_;
  std::weak_ptr<GoalHandleExecutePipeline> active_goal_;
  GoalAdmissionGate goal_admission_;
  std::optional<JobHandle> active_job_;
  std::jthread action_worker_;
};

}  // namespace open_lmm
