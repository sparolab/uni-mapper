#include "open_lmm_ros.hpp"
#include "action_terminal.hpp"
#include "ros_visualization_bridge.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <filesystem>
#include <stdexcept>
#include <utility>

namespace open_lmm {
namespace {

Result<ExecutionRequest> DecodeGoal(
    const open_lmm_ros::action::ExecutePipeline::Goal& goal) {
  ExecutionRequest request;
  if (goal.kind > static_cast<uint8_t>(ExecutionRequestKind::kOptimizeThrough)) {
    return Result<ExecutionRequest>::Failure(
        Error::InvalidArgument("unknown execution request kind"));
  }
  request.kind = static_cast<ExecutionRequestKind>(goal.kind);
  if (goal.has_stage) {
    if (goal.stage > static_cast<uint8_t>(StageId::kSave)) {
      return Result<ExecutionRequest>::Failure(
          Error::InvalidArgument("unknown pipeline stage"));
    }
    request.stage = static_cast<StageId>(goal.stage);
  }
  if (goal.has_node) {
    if (goal.node > static_cast<uint8_t>(NodeId::kFallbackMapSave)) {
      return Result<ExecutionRequest>::Failure(
          Error::InvalidArgument("unknown pipeline node"));
    }
    request.node = static_cast<NodeId>(goal.node);
  }
  if (goal.has_agent) {
    auto agent = AgentId::Parse(goal.agent);
    if (!agent) return Result<ExecutionRequest>::Failure(agent.GetError());
    request.agent = std::move(agent).Value();
  }

  const bool valid_shape =
      (request.kind == ExecutionRequestKind::kRunAll && !request.stage &&
       !request.node && !request.agent) ||
      (request.kind == ExecutionRequestKind::kStage && request.stage &&
       !request.node && !request.agent) ||
      (request.kind == ExecutionRequestKind::kNode && !request.stage &&
       request.node) ||
      (request.kind == ExecutionRequestKind::kOptimizeThrough &&
       !request.stage && !request.node && request.agent);
  if (!valid_shape) {
    return Result<ExecutionRequest>::Failure(Error::InvalidArgument(
        "execution request fields do not match the selected kind"));
  }
  return Result<ExecutionRequest>::Ok(std::move(request));
}

open_lmm_ros::msg::ExecutionEvent ToRosEvent(const ExecutionEvent& event) {
  open_lmm_ros::msg::ExecutionEvent result;
  result.job_id = event.job_id;
  result.sequence = event.sequence;
  result.event_type = static_cast<uint8_t>(event.type);
  result.has_stage = event.stage.has_value();
  if (event.stage) result.stage = static_cast<uint8_t>(*event.stage);
  result.has_node = event.node.has_value();
  if (event.node) result.node = static_cast<uint8_t>(*event.node);
  result.has_agent = event.agent.has_value();
  if (event.agent) result.agent = event.agent->Value();
  result.progress_current = event.progress_current;
  result.progress_total = event.progress_total;
  result.has_algorithm_progress = event.algorithm_progress.has_value();
  if (event.algorithm_progress) {
    result.progress_phase =
        static_cast<uint8_t>(event.algorithm_progress->phase);
    result.progress_total_known = event.algorithm_progress->total.has_value();
    result.progress_operation = event.algorithm_progress->operation;
  }
  result.message = event.message;
  result.has_error = event.error.has_value();
  if (event.error) result.error = event.error->Message();
  result.affected_agents.reserve(event.affected_agents.size());
  for (const auto& agent : event.affected_agents) {
    result.affected_agents.push_back(agent.Value());
  }
  return result;
}

}  // namespace

OpenLMMROS::OpenLMMROS(const rclcpp::NodeOptions& options)
    : Node("open_lmm_ros", options),
      runtime_(std::make_shared<RuntimeClient>()) {
  this->declare_parameter<std::string>("config_path", "config");
  std::string config_path = this->get_parameter("config_path").as_string();
  if (config_path.empty()) {
    throw std::invalid_argument("config_path must not be empty");
  }
  if (config_path.front() != '/') {
    config_path =
        (std::filesystem::path(
             ament_index_cpp::get_package_share_directory("open_lmm")) /
         config_path)
            .string();
  }

  config_path_ = std::filesystem::path(config_path);
  auto opened = runtime_->Open(BootstrapRequest{config_path_, "ros"});
  if (!opened) {
    throw std::runtime_error("Runtime initialization failed: " +
                             opened.GetError().Message());
  }

  event_publisher_ =
      this->create_publisher<open_lmm_ros::msg::ExecutionEvent>(
          "~/events", rclcpp::QoS(50).reliable());
  execute_action_ = rclcpp_action::create_server<ExecutePipeline>(
      this, "~/execute",
      [this](const auto& uuid, const auto& goal) {
        return HandleGoal(uuid, goal);
      },
      [this](const auto& goal_handle) { return HandleCancel(goal_handle); },
      [this](const auto& goal_handle) { HandleAccepted(goal_handle); });
  status_service_ = this->create_service<GetRuntimeStatus>(
      "~/status",
      [this](std::shared_ptr<GetRuntimeStatus::Request> request,
             std::shared_ptr<GetRuntimeStatus::Response> response) {
        HandleStatus(request, response);
      });
  visualization_bridge_ =
      std::make_unique<RosVisualizationBridge>(*this, runtime_);
  auto subscribed = runtime_->SubscribeEvents(
      [this](const ExecutionEvent& event) { PublishEvent(event); });
  if (!subscribed) {
    throw std::runtime_error("Runtime event subscription failed: " +
                             subscribed.GetError().Message());
  }
  event_subscription_ = std::move(subscribed).Value();
  visualization_bridge_->Start();

  RCLCPP_INFO(get_logger(), "ready; send goals to ~/execute");
}

OpenLMMROS::~OpenLMMROS() {
  event_subscription_.Reset();
  if (visualization_bridge_) visualization_bridge_->Stop();
  std::optional<JobHandle> active_job;
  {
    std::lock_guard lock(action_mutex_);
    active_job = active_job_;
  }
  if (active_job) (void)runtime_->Cancel(*active_job);
  if (action_worker_.joinable()) action_worker_.join();
  (void)runtime_->Close(CloseMode::kCancelAndWait);
}

PipelineSnapshot OpenLMMROS::Snapshot() const {
  if (!runtime_) return {};
  auto snapshot = runtime_->Snapshot();
  return snapshot ? std::move(snapshot).Value().pipeline : PipelineSnapshot{};
}

rclcpp_action::GoalResponse OpenLMMROS::HandleGoal(
    const rclcpp_action::GoalUUID& uuid,
    std::shared_ptr<const ExecutePipeline::Goal> goal) {
  (void)uuid;
  if (!goal || !DecodeGoal(*goal)) {
    return rclcpp_action::GoalResponse::REJECT;
  }
  return goal_admission_.TryReserve()
             ? rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE
             : rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse OpenLMMROS::HandleCancel(
    const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle) {
  (void)goal_handle;
  std::optional<JobHandle> job;
  {
    std::lock_guard lock(action_mutex_);
    job = active_job_;
  }
  if (!job) return rclcpp_action::CancelResponse::REJECT;
  return runtime_->Cancel(*job)
             ? rclcpp_action::CancelResponse::ACCEPT
             : rclcpp_action::CancelResponse::REJECT;
}

void OpenLMMROS::HandleAccepted(
    const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle) {
  if (action_worker_.joinable()) action_worker_.join();
  {
    std::lock_guard lock(action_mutex_);
    active_goal_ = goal_handle;
  }
  action_worker_ =
      std::jthread([this, goal_handle] { ExecuteGoal(goal_handle); });
}

void OpenLMMROS::ExecuteGoal(
    const std::shared_ptr<GoalHandleExecutePipeline>& goal_handle) {
  auto result = std::make_shared<ExecutePipeline::Result>();
  auto request = DecodeGoal(*goal_handle->get_goal());
  auto submitted = request && runtime_
                       ? runtime_->Submit(request.Value())
                       : Result<JobHandle>::Failure(
                             request ? Error::InvalidArgument("runtime missing")
                                     : request.GetError());
  if (!submitted) {
    result->message = submitted.GetError().Message();
    goal_handle->abort(result);
    std::lock_guard lock(action_mutex_);
    goal_admission_.Release();
    active_goal_.reset();
    return;
  }
  const JobHandle job = submitted.Value();
  result->job_id = job.value;
  {
    std::lock_guard lock(action_mutex_);
    active_job_ = job;
  }

  auto waited = runtime_->Wait(job);
  auto snapshot = runtime_->Snapshot();
  std::optional<JobSnapshot> authoritative_job;
  if (snapshot) {
    const auto& value = snapshot.Value();
    result->runtime_state = static_cast<uint8_t>(value.state);
    result->config_revision = value.pipeline.config_revision;
    result->output_directory = value.output_directory.string();
    if (value.pipeline.job && value.pipeline.job->id == job.value) {
      authoritative_job = value.pipeline.job;
    }
  }
  const auto terminal =
      ResolveRosActionTerminal(waited, job.value, authoritative_job);
  result->success = terminal == RosActionTerminal::kSucceeded;
  result->message = terminal == RosActionTerminal::kSucceeded
                        ? std::string{}
                        : waited.GetError().Message();
  if (terminal == RosActionTerminal::kSucceeded) {
    goal_handle->succeed(result);
  } else if (terminal == RosActionTerminal::kCanceled) {
    goal_handle->canceled(result);
  } else {
    goal_handle->abort(result);
  }
  std::lock_guard lock(action_mutex_);
  active_job_.reset();
  goal_admission_.Release();
  active_goal_.reset();
}

void OpenLMMROS::HandleStatus(
    const std::shared_ptr<GetRuntimeStatus::Request>& request,
    std::shared_ptr<GetRuntimeStatus::Response> response) const {
  (void)request;
  if (!runtime_) {
    response->error = "runtime missing";
    return;
  }
  auto snapshot = runtime_->Snapshot();
  if (!snapshot) {
    response->error = snapshot.GetError().Message();
    return;
  }
  const auto& value = snapshot.Value();
  response->success = true;
  response->runtime_state = static_cast<uint8_t>(value.state);
  response->output_directory = value.output_directory.string();
  response->runtime_revision = value.pipeline.runtime_revision;
  response->config_revision = value.pipeline.config_revision;
  response->has_job = value.pipeline.job.has_value();
  if (value.pipeline.job) {
    response->job_id = value.pipeline.job->id;
    response->job_state = static_cast<uint8_t>(value.pipeline.job->state);
    response->has_active_stage = value.pipeline.job->active_stage.has_value();
    if (value.pipeline.job->active_stage) {
      response->active_stage =
          static_cast<uint8_t>(*value.pipeline.job->active_stage);
    }
    response->job_message = value.pipeline.job->message;
  }
  response->agents.reserve(value.pipeline.agents.size());
  for (const auto& agent : value.pipeline.agents) {
    response->agents.push_back(agent.Value());
  }
}

void OpenLMMROS::PublishEvent(const ExecutionEvent& event) {
  auto message = ToRosEvent(event);
  event_publisher_->publish(message);
  if (visualization_bridge_) visualization_bridge_->RequestRefresh(event);
  std::shared_ptr<GoalHandleExecutePipeline> goal;
  {
    std::lock_guard lock(action_mutex_);
    if (active_job_ && active_job_->value == event.job_id) {
      goal = active_goal_.lock();
    }
  }
  if (!goal || !goal->is_active()) return;
  auto feedback = std::make_shared<ExecutePipeline::Feedback>();
  feedback->event = std::move(message);
  goal->publish_feedback(feedback);
}

}  // namespace open_lmm

RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS)
