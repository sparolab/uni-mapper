// STL
#include <filesystem>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <utility>

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
namespace {

const char* JobStateName(JobState state) {
  switch (state) {
    case JobState::kQueued: return "queued";
    case JobState::kWaitingForDependency: return "waiting_for_dependency";
    case JobState::kRunning: return "running";
    case JobState::kCancelling: return "cancelling";
    case JobState::kWaitingForAlignmentFeedback:
      return "waiting_for_alignment_feedback";
    case JobState::kSucceeded: return "succeeded";
    case JobState::kFailed: return "failed";
    case JobState::kCancelled: return "cancelled";
  }
  return "unknown";
}

const char* EventTypeName(EventType type) {
  switch (type) {
    case EventType::kJobQueued: return "job_queued";
    case EventType::kJobStarted: return "job_started";
    case EventType::kStageStarted: return "stage_started";
    case EventType::kNodeStarted: return "node_started";
    case EventType::kProgressUpdated: return "progress_updated";
    case EventType::kArtifactCommitted: return "artifact_committed";
    case EventType::kArtifactInvalidated: return "artifact_invalidated";
    case EventType::kNodeFailed: return "node_failed";
    case EventType::kStageCompleted: return "stage_completed";
    case EventType::kStageFailed: return "stage_failed";
    case EventType::kCancellationRequested: return "cancellation_requested";
    case EventType::kAlignmentFeedbackRequested:
      return "alignment_feedback_requested";
    case EventType::kAlignmentProposalAccepted:
      return "alignment_proposal_accepted";
    case EventType::kAlignmentProposalRejected:
      return "alignment_proposal_rejected";
    case EventType::kAlignmentFeedbackCancelled:
      return "alignment_feedback_cancelled";
    case EventType::kJobCompleted: return "job_completed";
    case EventType::kJobCancelled: return "job_cancelled";
  }
  return "unknown";
}

const char* StageName(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad: return "data_load";
    case StageId::kAlignment: return "alignment";
    case StageId::kMapUpdate: return "map_update";
    case StageId::kSave: return "save";
  }
  return "unknown";
}

void AppendCancellation(std::ostringstream& output,
                        const CancellationTelemetry& cancellation) {
  output << ";cancel_cooperative="
         << (cancellation.capability.cooperative ? "true" : "false")
         << ";cancel_pending=" << (cancellation.Pending() ? "true" : "false");
  if (cancellation.cancel_requested_at_unix_ns) {
    output << ";cancel_requested_at_unix_ns="
           << *cancellation.cancel_requested_at_unix_ns;
  }
  if (cancellation.cancel_observed_at_unix_ns) {
    output << ";cancel_observed_at_unix_ns="
           << *cancellation.cancel_observed_at_unix_ns;
  }
  if (cancellation.cancel_completed_at_unix_ns) {
    output << ";cancel_completed_at_unix_ns="
           << *cancellation.cancel_completed_at_unix_ns;
  }
}

std::string SnapshotText(const PipelineSnapshot& snapshot) {
  std::ostringstream output;
  output << "config_revision=" << snapshot.config_revision;
  if (!snapshot.job) {
    output << ";state=idle";
    return output.str();
  }
  output << ";job_id=" << snapshot.job->id
         << ";state=" << JobStateName(snapshot.job->state);
  if (snapshot.job->active_stage) {
    output << ";stage=" << StageName(*snapshot.job->active_stage);
  }
  if (!snapshot.job->message.empty()) {
    output << ";message=" << snapshot.job->message;
  }
  AppendCancellation(output, snapshot.job->cancellation);
  return output.str();
}

std::string EventText(const ExecutionEvent& event) {
  std::ostringstream output;
  output << "job_id=" << event.job_id
         << ";sequence=" << event.sequence
         << ";event=" << EventTypeName(event.type);
  if (event.stage) output << ";stage=" << StageName(*event.stage);
  if (event.node) {
    output << ";node=" << DescribeNode(*event.node).name;
  }
  if (event.agent) output << ";agent=" << *event.agent;
  if (event.progress_total != 0) {
    output << ";progress=" << event.progress_current << '/'
           << event.progress_total;
  }
  if (!event.message.empty()) output << ";message=" << event.message;
  if (event.error) output << ";error=" << event.error->Message();
  if (event.cancellation) AppendCancellation(output, *event.cancellation);
  return output.str();
}

bool IsTerminal(EventType type) {
  return type == EventType::kJobCompleted ||
         type == EventType::kJobCancelled;
}

}  // namespace

OpenLMMROS::OpenLMMROS(const rclcpp::NodeOptions &options)
    : Node("open_lmm_ros", options) {
  InitializeLogging();
  std::string config_path;
  this->declare_parameter<std::string>("config_path", "config");
  this->get_parameter<std::string>("config_path", config_path);

  if (config_path.empty()) {
    throw std::invalid_argument("config_path must not be empty");
  }
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
  auto gui_plugin_path =
      this->get_parameter("gui_plugin_path").as_string();
  const bool gui_enabled = this->get_parameter("gui_enabled").as_bool() ||
                           !gui_plugin_path.empty();

  auto map_server = std::make_shared<open_lmm::MapServer>();
  controller_ = std::make_shared<open_lmm::PipelineController>(map_server);

  progress_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "~/progress", rclcpp::QoS(50).reliable());
  result_publisher_ = this->create_publisher<std_msgs::msg::String>(
      "~/result", rclcpp::QoS(10).reliable().transient_local());
  start_service_ = this->create_service<Trigger>(
      "~/start",
      [this](const std::shared_ptr<Trigger::Request>& request,
             std::shared_ptr<Trigger::Response> response) {
        HandleStart(request, std::move(response));
      });
  cancel_service_ = this->create_service<Trigger>(
      "~/cancel",
      [this](const std::shared_ptr<Trigger::Request>& request,
             std::shared_ptr<Trigger::Response> response) {
        HandleCancel(request, std::move(response));
      });
  status_service_ = this->create_service<Trigger>(
      "~/status",
      [this](const std::shared_ptr<Trigger::Request>& request,
             std::shared_ptr<Trigger::Response> response) {
        HandleStatus(request, std::move(response));
      });
  event_subscription_ = controller_->SubscribeEvents(
      [this](const ExecutionEvent& event) { PublishEvent(event); });

  RCLCPP_INFO(get_logger(),
              "ready; call ~/start to submit a pipeline job");

  if (!gui_enabled) return;

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
}

OpenLMMROS::~OpenLMMROS() { event_subscription_.Reset(); }

PipelineSnapshot OpenLMMROS::Snapshot() const {
  return controller_ ? controller_->Snapshot() : PipelineSnapshot{};
}

void OpenLMMROS::HandleStart(
    const std::shared_ptr<Trigger::Request>& request,
    std::shared_ptr<Trigger::Response> response) {
  (void)request;
  auto submitted = controller_->SubmitRunAll();
  response->success = submitted.IsOk();
  response->message = submitted
                          ? "job_id=" +
                                std::to_string(std::move(submitted).Value())
                          : submitted.GetError().Message();
}

void OpenLMMROS::HandleCancel(
    const std::shared_ptr<Trigger::Request>& request,
    std::shared_ptr<Trigger::Response> response) {
  (void)request;
  const auto snapshot = controller_->Snapshot();
  if (!snapshot.job) {
    response->success = false;
    response->message = "no pipeline job exists";
    return;
  }
  auto cancelled = controller_->Cancel(snapshot.job->id);
  response->success = cancelled.IsOk();
  response->message = cancelled ? "job_id=" +
                                      std::to_string(snapshot.job->id) +
                                      ";state=cancelling"
                                : cancelled.GetError().Message();
}

void OpenLMMROS::HandleStatus(
    const std::shared_ptr<Trigger::Request>& request,
    std::shared_ptr<Trigger::Response> response) const {
  (void)request;
  response->success = true;
  response->message = SnapshotText(controller_->Snapshot());
}

void OpenLMMROS::PublishEvent(const ExecutionEvent& event) {
  std_msgs::msg::String progress;
  progress.data = EventText(event);
  progress_publisher_->publish(progress);

  if (!IsTerminal(event.type)) return;
  std_msgs::msg::String result;
  result.data = SnapshotText(controller_->Snapshot());
  result_publisher_->publish(result);
}

} // namespace open_lmm
RCLCPP_COMPONENTS_REGISTER_NODE(open_lmm::OpenLMMROS);
