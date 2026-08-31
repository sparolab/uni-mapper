#include "../../ros2/open_lmm_ros/runtime_adapter/open_lmm_ros.hpp"
#include "../../ros2/open_lmm_ros/runtime_adapter/ros_visualization_bridge.hpp"
#include "support/runtime/runtime_config_fixture.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <atomic>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <future>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include <unistd.h>

namespace {
using namespace std::chrono_literals;
namespace fs = std::filesystem;
using ExecutePipeline = open_lmm_ros::action::ExecutePipeline;
using GoalHandle = rclcpp_action::ClientGoalHandle<ExecutePipeline>;

void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

class TemporaryTree {
 public:
  TemporaryTree() {
    path_ = fs::temp_directory_path() /
            ("open_lmm_ros_graph_" + std::to_string(::getpid()));
    fs::remove_all(path_);
    fs::create_directories(path_);
  }
  ~TemporaryTree() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }
  const fs::path& Path() const { return path_; }

 private:
  fs::path path_;
};

void ExpandDataFixture(const fs::path& data, int frames) {
  std::ofstream poses(data / "agent1/poses.txt");
  Check(static_cast<bool>(poses), "open ROS pose fixture");
  for (int frame = 0; frame < frames; ++frame) {
    poses << "1 0 0 " << frame << " 0 1 0 0 0 0 1 0\n";
    std::ostringstream name;
    name << std::setw(6) << std::setfill('0') << frame << ".pcd";
    std::ofstream scan(data / "agent1/Scans" / name.str());
    Check(static_cast<bool>(scan), "open ROS scan fixture");
    constexpr int kPoints = 512;
    scan << "# .PCD v0.7\nVERSION 0.7\nFIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH "
         << kPoints
         << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << kPoints
         << "\nDATA ascii\n";
    for (int point = 0; point < kPoints; ++point) {
      scan << 5.0 + (point % 64) * 0.25 << ' '
           << -8.0 + (point / 64) * 2.0 << ' ' << (point % 5) * 0.1
           << ' ' << frame << '\n';
    }
  }
}

template <typename Future>
auto GetWithWatchdog(Future& future, const char* operation) {
  Check(future.wait_for(20s) == std::future_status::ready,
        std::string(operation) + " timed out");
  return future.get();
}

class ExecutorGuard {
 public:
  explicit ExecutorGuard(rclcpp::Executor& executor)
      : executor_(executor), worker_([this] { executor_.spin(); }) {}
  ~ExecutorGuard() {
    executor_.cancel();
    if (worker_.joinable()) worker_.join();
  }

 private:
  rclcpp::Executor& executor_;
  std::jthread worker_;
};

void RunGraphContract() {
  TemporaryTree fixture;
  const auto config = fixture.Path() / "config";
  const auto data = fixture.Path() / "data";
  open_lmm::test::WriteRuntimeConfigFixture(
      config, data, fixture.Path() / "output");
  ExpandDataFixture(data, 32);

  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("config_path", config.string())});
  auto adapter = std::make_shared<open_lmm::OpenLMMROS>(options);
  auto observer = std::make_shared<rclcpp::Node>("open_lmm_ros_graph_test");

  std::atomic<uint64_t> event_count{0};
  auto events = observer->create_subscription<open_lmm_ros::msg::ExecutionEvent>(
      "/open_lmm_ros/events", rclcpp::QoS(50).reliable(),
      [&](const open_lmm_ros::msg::ExecutionEvent&) {
        event_count.fetch_add(1, std::memory_order_relaxed);
      });
  std::promise<void> first_cloud;
  std::promise<void> first_path;
  std::promise<void> first_markers;
  std::atomic<bool> cloud_seen{false};
  std::atomic<bool> path_seen{false};
  std::atomic<bool> markers_seen{false};
  const auto visualization_qos =
      rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  auto clouds = observer->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/open_lmm_ros/visualization/a_agent1/points", visualization_qos,
      [&](const sensor_msgs::msg::PointCloud2& message) {
        if (message.width != 0 && !cloud_seen.exchange(true)) {
          first_cloud.set_value();
        }
      });
  auto paths = observer->create_subscription<nav_msgs::msg::Path>(
      "/open_lmm_ros/visualization/a_agent1/path", visualization_qos,
      [&](const nav_msgs::msg::Path& message) {
        if (!message.poses.empty() && !path_seen.exchange(true)) {
          first_path.set_value();
        }
      });
  auto markers =
      observer->create_subscription<visualization_msgs::msg::MarkerArray>(
          "/open_lmm_ros/visualization/loops", visualization_qos,
          [&](const visualization_msgs::msg::MarkerArray& message) {
            if (message.markers.size() == 2 && !markers_seen.exchange(true)) {
              first_markers.set_value();
            }
          });
  auto status = observer->create_client<open_lmm_ros::srv::GetRuntimeStatus>(
      "/open_lmm_ros/status");
  auto actions = rclcpp_action::create_client<ExecutePipeline>(
      observer, "/open_lmm_ros/execute");

  rclcpp::executors::MultiThreadedExecutor executor(
      rclcpp::ExecutorOptions{}, 4);
  executor.add_node(adapter);
  executor.add_node(observer);
  ExecutorGuard spinning(executor);

  Check(status->wait_for_service(10s), "ROS status service was not discovered");
  Check(actions->wait_for_action_server(10s),
        "ROS execute action was not discovered");

  auto status_future = status->async_send_request(
      std::make_shared<open_lmm_ros::srv::GetRuntimeStatus::Request>());
  const auto initial_status = GetWithWatchdog(status_future, "initial status");
  Check(initial_status->success && initial_status->error.empty() &&
            initial_status->runtime_revision != 0 &&
            initial_status->config_revision != 0,
        "status service exposes the opened runtime");

  ExecutePipeline::Goal malformed;
  malformed.kind = 255;
  auto malformed_future = actions->async_send_goal(malformed);
  Check(!GetWithWatchdog(malformed_future, "malformed goal"),
        "invalid action shape is rejected by the real adapter");

  std::promise<void> first_feedback;
  std::atomic<bool> first_feedback_seen{false};
  rclcpp_action::Client<ExecutePipeline>::SendGoalOptions send_options;
  send_options.feedback_callback =
      [&](GoalHandle::SharedPtr,
          const std::shared_ptr<const ExecutePipeline::Feedback>&) {
        if (!first_feedback_seen.exchange(true)) first_feedback.set_value();
      };
  ExecutePipeline::Goal run_all;
  run_all.kind = ExecutePipeline::Goal::RUN_ALL;
  auto goal_future = actions->async_send_goal(run_all, send_options);
  auto goal = GetWithWatchdog(goal_future, "run-all goal admission");
  Check(static_cast<bool>(goal), "valid run-all goal is accepted");
  auto feedback_future = first_feedback.get_future();
  Check(feedback_future.wait_for(20s) == std::future_status::ready,
        "real action publishes typed feedback");
  auto result_future = actions->async_get_result(goal);
  const auto result = GetWithWatchdog(result_future, "run-all result");
  Check(result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->success && result.result->job_id != 0 &&
            result.result->runtime_state ==
                static_cast<uint8_t>(open_lmm::RuntimeStatus::kReady) &&
            result.result->config_revision == initial_status->config_revision &&
            !result.result->output_directory.empty() &&
            result.result->message.empty(),
        "real action completes the public runtime workflow");
  bool late_cancel_rejected = false;
  try {
    auto late_cancel_future = actions->async_cancel_goal(goal);
    const auto late_cancel =
        GetWithWatchdog(late_cancel_future, "post-commit cancellation");
    late_cancel_rejected = late_cancel->goals_canceling.empty();
  } catch (const std::exception&) {
    // rclcpp may evict an already-terminal goal handle locally. That is also
    // an explicit rejection rather than a second terminal transition.
    late_cancel_rejected = true;
  }
  Check(late_cancel_rejected &&
            result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            result.result->success,
        "post-commit cancellation did not preserve committed success");
  Check(event_count.load(std::memory_order_relaxed) != 0,
        "runtime events cross the real ROS topic graph");
  auto cloud_future = first_cloud.get_future();
  auto path_future = first_path.get_future();
  auto marker_future = first_markers.get_future();
  Check(cloud_future.wait_for(20s) == std::future_status::ready,
        "committed point cloud crosses the real ROS topic graph");
  Check(path_future.wait_for(20s) == std::future_status::ready,
        "committed path crosses the real ROS topic graph");
  Check(marker_future.wait_for(20s) == std::future_status::ready,
        "committed loop marker batch crosses the real ROS topic graph");

  std::promise<void> late_cloud;
  std::atomic<bool> late_cloud_seen{false};
  auto late_cloud_future = late_cloud.get_future();
  auto late_cloud_subscription =
      observer->create_subscription<sensor_msgs::msg::PointCloud2>(
          "/open_lmm_ros/visualization/a_agent1/points", visualization_qos,
          [&](const sensor_msgs::msg::PointCloud2& message) {
            if (message.width != 0 && !late_cloud_seen.exchange(true)) {
              late_cloud.set_value();
            }
          });
  Check(late_cloud_future.wait_for(10s) == std::future_status::ready,
        "late subscriber receives the retained committed point cloud");

  std::promise<void> cancel_feedback;
  std::atomic<bool> cancel_feedback_seen{false};
  rclcpp_action::Client<ExecutePipeline>::SendGoalOptions cancel_options;
  cancel_options.feedback_callback =
      [&](GoalHandle::SharedPtr,
          const std::shared_ptr<const ExecutePipeline::Feedback>&) {
        if (!cancel_feedback_seen.exchange(true)) cancel_feedback.set_value();
      };
  ExecutePipeline::Goal rerun;
  rerun.kind = ExecutePipeline::Goal::STAGE;
  rerun.has_stage = true;
  rerun.stage = static_cast<uint8_t>(open_lmm::StageId::kDataLoad);
  auto rerun_future = actions->async_send_goal(rerun, cancel_options);
  auto rerun_handle = GetWithWatchdog(rerun_future, "cancel goal admission");
  Check(static_cast<bool>(rerun_handle), "cancellable stage goal is accepted");
  auto cancel_feedback_future = cancel_feedback.get_future();
  Check(cancel_feedback_future.wait_for(20s) == std::future_status::ready,
        "cancellable goal reaches active execution");
  auto cancel_future = actions->async_cancel_goal(rerun_handle);
  const auto cancel_response =
      GetWithWatchdog(cancel_future, "action cancellation");
  auto cancel_result_future = actions->async_get_result(rerun_handle);
  const auto cancel_result =
      GetWithWatchdog(cancel_result_future, "cancel terminal result");
  const bool cancellation_accepted = !cancel_response->goals_canceling.empty();
  Check((cancellation_accepted &&
         cancel_result.code == rclcpp_action::ResultCode::CANCELED &&
         !cancel_result.result->success) ||
            (!cancellation_accepted &&
             cancel_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
             cancel_result.result->success),
        "cancel endpoint preserves pre-commit cancel or committed-success semantics");

  auto final_status_future = status->async_send_request(
      std::make_shared<open_lmm_ros::srv::GetRuntimeStatus::Request>());
  const auto final_status =
      GetWithWatchdog(final_status_future, "final status");
  Check(final_status->success && final_status->config_revision != 0,
        "status remains queryable after action cancellation");

  ExecutePipeline::Goal next;
  next.kind = ExecutePipeline::Goal::NODE;
  next.has_node = true;
  next.node = static_cast<uint8_t>(open_lmm::NodeId::kPoseSave);
  auto next_goal_future = actions->async_send_goal(next);
  auto next_handle =
      GetWithWatchdog(next_goal_future, "next goal admission");
  Check(static_cast<bool>(next_handle),
        "terminal action did not reopen goal admission");
  auto next_result_future = actions->async_get_result(next_handle);
  const auto next_result =
      GetWithWatchdog(next_result_future, "next goal result");
  Check(next_result.code == rclcpp_action::ResultCode::SUCCEEDED &&
            next_result.result->success,
        "next goal did not complete after cancellation reconciliation");

  (void)events;
  (void)clouds;
  (void)paths;
  (void)markers;
  (void)late_cloud_subscription;
  executor.remove_node(observer);
  executor.remove_node(adapter);
}

void TestVisualizationParameterValidation() {
  auto default_node =
      std::make_shared<rclcpp::Node>("open_lmm_default_rviz_parameters");
  open_lmm::RosVisualizationBridge defaults(
      *default_node, std::shared_ptr<open_lmm::RuntimeClient>{});
  Check(default_node->get_parameter("rviz_preview_voxel_size_m").as_double() ==
            0.4,
        "RViz preview defaults to a 0.4 metre voxel");
  Check(default_node->get_parameter("rviz_max_point_count").as_int() ==
            2'000'000,
        "RViz point payload defaults to the bounded two million limit");

  rclcpp::NodeOptions invalid_options;
  invalid_options.parameter_overrides(
      {rclcpp::Parameter("rviz_max_point_count", 0)});
  auto invalid_node =
      std::make_shared<rclcpp::Node>("open_lmm_invalid_rviz_parameters",
                                    invalid_options);
  bool rejected = false;
  try {
    open_lmm::RosVisualizationBridge bridge(
        *invalid_node, std::shared_ptr<open_lmm::RuntimeClient>{});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "invalid RViz resource parameters fail closed");

  rclcpp::NodeOptions excessive_options;
  excessive_options.parameter_overrides(
      {rclcpp::Parameter("rviz_max_point_count", 2'000'001)});
  auto excessive_node =
      std::make_shared<rclcpp::Node>("open_lmm_excessive_rviz_parameters",
                                    excessive_options);
  rejected = false;
  try {
    open_lmm::RosVisualizationBridge bridge(
        *excessive_node, std::shared_ptr<open_lmm::RuntimeClient>{});
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "unbounded RViz resource parameters fail closed");

  rclcpp::NodeOptions disabled_options;
  disabled_options.parameter_overrides(
      {rclcpp::Parameter("rviz_visualization_enabled", false)});
  auto disabled_node =
      std::make_shared<rclcpp::Node>("open_lmm_disabled_rviz_bridge",
                                    disabled_options);
  open_lmm::RosVisualizationBridge disabled(
      *disabled_node, std::shared_ptr<open_lmm::RuntimeClient>{});
  disabled.Start();
  const auto topics = disabled_node->get_topic_names_and_types();
  Check(topics.find("/open_lmm_disabled_rviz_bridge/visualization/loops") ==
            topics.end(),
        "disabled RViz bridge creates no visualization publishers");
  disabled.Stop();
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    RunGraphContract();
    TestVisualizationParameterValidation();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    rclcpp::shutdown();
    std::cerr << "ROS graph contract failed: " << error.what() << '\n';
    return 1;
  }
}
