#include "open_lmm_control_panel.hpp"

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/display_context.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction_iface.hpp>

#include <QComboBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMetaObject>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>

#include <algorithm>
#include <chrono>
#include <utility>

namespace open_lmm_ros::control {
namespace {

QString RuntimeStateName(uint8_t value) {
  static constexpr const char* kNames[] = {
      "Creating", "Ready", "Running", "Cancelling", "Recoverable failure",
      "Fatal failure", "Closing", "Closed"};
  return value < std::size(kNames) ? kNames[value] : "Unknown";
}

QString JobStateName(uint8_t value) {
  static constexpr const char* kNames[] = {
      "Queued", "Waiting for dependency", "Running", "Cancelling",
      "Waiting for alignment", "Succeeded", "Failed", "Cancelled"};
  return value < std::size(kNames) ? kNames[value] : "Unknown";
}

QString StageName(uint8_t value) {
  static constexpr const char* kNames[] = {
      "DataLoad", "Alignment", "MapUpdate", "Save"};
  return value < std::size(kNames) ? kNames[value] : "Unknown";
}

}  // namespace

OpenLmmControlPanel::OpenLmmControlPanel(QWidget* parent)
    : rviz_common::Panel(parent), callback_gate_(std::make_shared<CallbackGate>()) {
  callback_gate_->owner = this;
  namespace_edit_ = new QLineEdit("/open_lmm_ros", this);
  connection_label_ = new QLabel("Disconnected", this);
  runtime_label_ = new QLabel("Runtime: unavailable", this);
  job_label_ = new QLabel("Job: none", this);
  revision_label_ = new QLabel("Revisions: unavailable", this);
  progress_label_ = new QLabel("Progress: -", this);
  message_label_ = new QLabel(this);
  message_label_->setWordWrap(true);

  stage_combo_ = new QComboBox(this);
  for (const char* name : {"DataLoad", "Alignment", "MapUpdate", "Save"})
    stage_combo_->addItem(name);
  node_combo_ = new QComboBox(this);
  for (const char* name : {"DataLoad", "LoopDetect", "Optimize", "MapUpdate",
                           "PoseSave", "FallbackMapSave"})
    node_combo_->addItem(name);
  agent_combo_ = new QComboBox(this);

  run_all_button_ = new QPushButton("Run All", this);
  run_stage_button_ = new QPushButton("Run Stage", this);
  run_node_button_ = new QPushButton("Run Node", this);
  optimize_button_ = new QPushButton("Optimize Through", this);
  cancel_button_ = new QPushButton("Cancel Active Goal", this);

  auto* form = new QFormLayout;
  form->addRow("Target namespace", namespace_edit_);
  form->addRow("Stage", stage_combo_);
  form->addRow("Node", node_combo_);
  form->addRow("Agent", agent_combo_);
  auto* commands = new QGridLayout;
  commands->addWidget(run_all_button_, 0, 0);
  commands->addWidget(run_stage_button_, 0, 1);
  commands->addWidget(run_node_button_, 1, 0);
  commands->addWidget(optimize_button_, 1, 1);
  commands->addWidget(cancel_button_, 2, 0, 1, 2);
  auto* layout = new QVBoxLayout(this);
  layout->addLayout(form);
  layout->addLayout(commands);
  layout->addWidget(connection_label_);
  layout->addWidget(runtime_label_);
  layout->addWidget(job_label_);
  layout->addWidget(revision_label_);
  layout->addWidget(progress_label_);
  layout->addWidget(message_label_);
  layout->addStretch();

  connect(namespace_edit_, &QLineEdit::editingFinished, this, [this] {
    Reconnect();
    Q_EMIT configChanged();
  });
  connect(node_combo_, qOverload<int>(&QComboBox::currentIndexChanged), this,
          [this] { RefreshWidgets(); });
  connect(run_all_button_, &QPushButton::clicked, this, [this] {
    ExecutePipeline::Goal goal;
    goal.kind = ExecutePipeline::Goal::RUN_ALL;
    SendGoal(goal);
  });
  connect(run_stage_button_, &QPushButton::clicked, this, [this] {
    auto goal = StageGoal(static_cast<Stage>(stage_combo_->currentIndex()));
    if (goal) SendGoal(*goal);
  });
  connect(run_node_button_, &QPushButton::clicked, this, [this] {
    auto goal = NodeGoal(static_cast<Node>(node_combo_->currentIndex()),
                         agent_combo_->currentText().toStdString());
    if (goal) SendGoal(*goal);
  });
  connect(optimize_button_, &QPushButton::clicked, this, [this] {
    auto goal = OptimizeThroughGoal(agent_combo_->currentText().toStdString());
    if (goal) SendGoal(*goal);
  });
  connect(cancel_button_, &QPushButton::clicked, this,
          [this] { CancelActiveGoal(); });
  RefreshWidgets();
}

OpenLmmControlPanel::~OpenLmmControlPanel() {
  {
    std::lock_guard lock(callback_gate_->mutex);
    callback_gate_->owner = nullptr;
  }
  status_timer_.reset();
  event_subscription_.reset();
  status_client_.reset();
  action_client_.reset();
}

void OpenLmmControlPanel::onInitialize() {
  auto abstraction = getDisplayContext()->getRosNodeAbstraction().lock();
  if (!abstraction) {
    SetTransientMessage("RViz ROS node is unavailable");
    return;
  }
  node_ = abstraction->get_raw_node();
  Reconnect();
}

void OpenLmmControlPanel::load(const rviz_common::Config& config) {
  rviz_common::Panel::load(config);
  QString target;
  if (config.mapGetString("Target Namespace", &target) && !target.isEmpty())
    namespace_edit_->setText(target);
}

void OpenLmmControlPanel::save(rviz_common::Config config) const {
  rviz_common::Panel::save(config);
  config.mapSetValue("Target Namespace", namespace_edit_->text());
}

std::string OpenLmmControlPanel::Endpoint(const char* suffix) const {
  QString base = namespace_edit_->text().trimmed();
  if (base.isEmpty()) base = "/open_lmm_ros";
  while (base.size() > 1 && base.endsWith('/')) base.chop(1);
  if (!base.startsWith('/')) base.prepend('/');
  return (base + '/' + suffix).toStdString();
}

void OpenLmmControlPanel::Reconnect() {
  if (!node_) return;
  const uint64_t generation = ++connection_generation_;
  status_request_in_flight_ = false;
  status_timer_.reset();
  event_subscription_.reset();
  status_client_ = node_->create_client<GetRuntimeStatus>(Endpoint("status"));
  action_client_ = rclcpp_action::create_client<ExecutePipeline>(
      node_, Endpoint("execute"));
  event_subscription_ = node_->create_subscription<msg::ExecutionEvent>(
      Endpoint("events"), rclcpp::QoS(50).reliable(),
      [gate = callback_gate_, generation](const msg::ExecutionEvent& event) {
        QueueToPanel(gate, [event, generation](OpenLmmControlPanel& panel) {
          if (generation == panel.connection_generation_) panel.ApplyEvent(event);
        });
      });
  status_timer_ = node_->create_wall_timer(
      std::chrono::milliseconds(500), [gate = callback_gate_, generation] {
        QueueToPanel(gate, [generation](OpenLmmControlPanel& panel) {
          if (generation == panel.connection_generation_) panel.RequestStatus();
        });
      });
  model_.Disconnect("Waiting for OpenLMM ROS services");
  RefreshWidgets();
  RequestStatus();
}

void OpenLmmControlPanel::RequestStatus() {
  if (!status_client_) return;
  if (!status_client_->service_is_ready()) {
    model_.Disconnect("Status service is unavailable");
    RefreshWidgets();
    return;
  }
  bool expected = false;
  if (!status_request_in_flight_.compare_exchange_strong(expected, true))
    return;
  const uint64_t generation = connection_generation_;
  auto request = std::make_shared<GetRuntimeStatus::Request>();
  status_client_->async_send_request(
      request, [gate = callback_gate_, generation](
                   rclcpp::Client<GetRuntimeStatus>::SharedFuture future) {
        try {
          const auto response = *future.get();
          QueueToPanel(gate, [response, generation](OpenLmmControlPanel& panel) {
            if (generation == panel.connection_generation_)
              panel.ApplyStatus(response);
          });
        } catch (const std::exception& error) {
          const std::string message = error.what();
          QueueToPanel(gate, [message, generation](OpenLmmControlPanel& panel) {
            if (generation != panel.connection_generation_) return;
            panel.status_request_in_flight_ = false;
            panel.model_.Disconnect("Status request failed: " + message);
            panel.RefreshWidgets();
          });
        }
      });
}

void OpenLmmControlPanel::ApplyStatus(
    const GetRuntimeStatus::Response& response) {
  status_request_in_flight_ = false;
  const std::string selected = agent_combo_->currentText().toStdString();
  model_.ApplyStatus(response);
  agent_combo_->clear();
  for (const auto& agent : model_.Agents())
    agent_combo_->addItem(QString::fromStdString(agent));
  const int previous = agent_combo_->findText(QString::fromStdString(selected));
  if (previous >= 0) agent_combo_->setCurrentIndex(previous);
  RefreshWidgets();
}

void OpenLmmControlPanel::ApplyEvent(const msg::ExecutionEvent& event) {
  if (!model_.ApplyEvent(event)) RequestStatus();
  RefreshWidgets();
}

void OpenLmmControlPanel::SendGoal(const ExecutePipeline::Goal& goal) {
  if (!action_client_ || !action_client_->action_server_is_ready()) {
    SetTransientMessage("Execute action server is unavailable");
    return;
  }
  const uint64_t generation = connection_generation_;
  rclcpp_action::Client<ExecutePipeline>::SendGoalOptions options;
  options.goal_response_callback =
      [gate = callback_gate_, generation](GoalHandle::SharedPtr handle) {
        QueueToPanel(gate, [accepted = static_cast<bool>(handle), generation](
                               OpenLmmControlPanel& panel) {
          if (generation != panel.connection_generation_) return;
          panel.SetTransientMessage(accepted ? "Goal accepted" : "Goal rejected");
          panel.RequestStatus();
        });
  };
  options.feedback_callback =
      [gate = callback_gate_, generation](GoalHandle::SharedPtr,
             const std::shared_ptr<const ExecutePipeline::Feedback> feedback) {
        if (!feedback) return;
        const auto event = feedback->event;
        QueueToPanel(gate, [event, generation](OpenLmmControlPanel& panel) {
          if (generation == panel.connection_generation_) panel.ApplyEvent(event);
        });
      };
  options.result_callback = [gate = callback_gate_, generation](
                                const GoalHandle::WrappedResult& result) {
    const QString message = result.result
                                ? QString::fromStdString(result.result->message)
                                : QString("Goal ended without a result payload");
    QueueToPanel(gate, [message, generation](OpenLmmControlPanel& panel) {
      if (generation != panel.connection_generation_) return;
      panel.SetTransientMessage(message.isEmpty() ? "Goal completed" : message);
      panel.RequestStatus();
    });
  };
  action_client_->async_send_goal(goal, options);
  SetTransientMessage("Sending goal...");
}

void OpenLmmControlPanel::CancelActiveGoal() {
  if (!action_client_ || !action_client_->action_server_is_ready()) {
    SetTransientMessage("Execute action server is unavailable");
    return;
  }
  const uint64_t generation = connection_generation_;
  action_client_->async_cancel_all_goals(
      [gate = callback_gate_, generation](
          rclcpp_action::Client<ExecutePipeline>::CancelResponse::SharedPtr response) {
        const bool accepted = response && !response->goals_canceling.empty();
        QueueToPanel(gate, [accepted, generation](OpenLmmControlPanel& panel) {
          if (generation != panel.connection_generation_) return;
          panel.SetTransientMessage(
              accepted ? "Cancellation requested"
                       : "No active goal accepted cancellation");
          panel.RequestStatus();
        });
      });
}

void OpenLmmControlPanel::QueueToPanel(
    const std::shared_ptr<CallbackGate>& gate,
    std::function<void(OpenLmmControlPanel&)> callback) {
  std::lock_guard lock(gate->mutex);
  if (!gate->owner) return;
  QMetaObject::invokeMethod(
      gate->owner,
      [gate, callback = std::move(callback)] {
        std::lock_guard inner_lock(gate->mutex);
        if (gate->owner) callback(*gate->owner);
      },
      Qt::QueuedConnection);
}

void OpenLmmControlPanel::SetTransientMessage(const QString& message) {
  message_label_->setText(message);
}

void OpenLmmControlPanel::RefreshWidgets() {
  connection_label_->setText(model_.Connected() ? "Connected" : "Disconnected");
  runtime_label_->setText("Runtime: " + RuntimeStateName(model_.RuntimeState()));
  revision_label_->setText(
      QString("Revisions: runtime=%1 config=%2")
          .arg(model_.RuntimeRevision())
          .arg(model_.ConfigRevision()));
  if (model_.HasJob()) {
    QString job = QString("Job: %1 (%2)")
                      .arg(model_.JobId())
                      .arg(JobStateName(model_.JobState()));
    if (model_.ActiveStage())
      job += " / " + StageName(*model_.ActiveStage());
    job_label_->setText(job);
  } else {
    job_label_->setText("Job: none");
  }
  progress_label_->setText(
      model_.ProgressTotal() == 0
          ? "Progress: -"
          : QString("Progress: %1 / %2")
                .arg(model_.ProgressCurrent())
                .arg(model_.ProgressTotal()));
  if (!model_.Message().empty())
    message_label_->setText(QString::fromStdString(model_.Message()));

  const bool submit = model_.Connected() && model_.CanSubmit() &&
                      action_client_ && action_client_->action_server_is_ready();
  const bool has_agent = agent_combo_->count() != 0;
  run_all_button_->setEnabled(submit);
  run_stage_button_->setEnabled(submit);
  const auto selected_node = static_cast<Node>(node_combo_->currentIndex());
  run_node_button_->setEnabled(submit &&
      (!NodeRequiresAgent(selected_node) || has_agent));
  optimize_button_->setEnabled(submit && has_agent);
  cancel_button_->setEnabled(model_.Connected() && model_.CanCancel() &&
                             action_client_ &&
                             action_client_->action_server_is_ready());
}

}  // namespace open_lmm_ros::control

PLUGINLIB_EXPORT_CLASS(open_lmm_ros::control::OpenLmmControlPanel,
                       rviz_common::Panel)
