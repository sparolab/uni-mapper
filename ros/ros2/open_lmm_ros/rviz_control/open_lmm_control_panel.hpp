#pragma once

#include "control_panel_model.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rviz_common/panel.hpp>

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>

class QComboBox;
class QLabel;
class QLineEdit;
class QPushButton;

namespace open_lmm_ros::control {

class OpenLmmControlPanel final : public rviz_common::Panel {
 public:
  explicit OpenLmmControlPanel(QWidget* parent = nullptr);
  ~OpenLmmControlPanel() override;

  void onInitialize() override;
  void load(const rviz_common::Config& config) override;
  void save(rviz_common::Config config) const override;

 private:
  struct CallbackGate {
    std::mutex mutex;
    OpenLmmControlPanel* owner = nullptr;
  };

  using ExecutePipeline = action::ExecutePipeline;
  using GoalHandle = rclcpp_action::ClientGoalHandle<ExecutePipeline>;
  using GetRuntimeStatus = srv::GetRuntimeStatus;

  void Reconnect();
  void RequestStatus();
  void ApplyStatus(const GetRuntimeStatus::Response& response);
  void ApplyEvent(const msg::ExecutionEvent& event);
  void SendGoal(const ExecutePipeline::Goal& goal);
  void CancelActiveGoal();
  void RefreshWidgets();
  void SetTransientMessage(const QString& message);
  static void QueueToPanel(
      const std::shared_ptr<CallbackGate>& gate,
      std::function<void(OpenLmmControlPanel&)> callback);
  [[nodiscard]] std::string Endpoint(const char* suffix) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp_action::Client<ExecutePipeline>::SharedPtr action_client_;
  rclcpp::Client<GetRuntimeStatus>::SharedPtr status_client_;
  rclcpp::Subscription<msg::ExecutionEvent>::SharedPtr event_subscription_;
  rclcpp::TimerBase::SharedPtr status_timer_;
  std::atomic_bool status_request_in_flight_{false};
  std::atomic_uint64_t connection_generation_{0};
  std::shared_ptr<CallbackGate> callback_gate_;
  ControlPanelModel model_;

  QLineEdit* namespace_edit_ = nullptr;
  QLabel* connection_label_ = nullptr;
  QLabel* runtime_label_ = nullptr;
  QLabel* job_label_ = nullptr;
  QLabel* revision_label_ = nullptr;
  QLabel* progress_label_ = nullptr;
  QLabel* message_label_ = nullptr;
  QComboBox* stage_combo_ = nullptr;
  QComboBox* node_combo_ = nullptr;
  QComboBox* agent_combo_ = nullptr;
  QPushButton* run_all_button_ = nullptr;
  QPushButton* run_stage_button_ = nullptr;
  QPushButton* run_node_button_ = nullptr;
  QPushButton* optimize_button_ = nullptr;
  QPushButton* cancel_button_ = nullptr;
};

}  // namespace open_lmm_ros::control
