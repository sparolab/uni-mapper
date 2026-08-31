#include "../../ros2/open_lmm_ros/runtime_adapter/open_lmm_ros.hpp"
#include "support/runtime/runtime_config_fixture.hpp"

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

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
  Check(initial_status->success && initial_status->error.empty(),
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
            result.result->success && result.result->job_id != 0,
        "real action completes the public runtime workflow");
  Check(event_count.load(std::memory_order_relaxed) != 0,
        "runtime events cross the real ROS topic graph");

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
         cancel_result.code == rclcpp_action::ResultCode::CANCELED) ||
            (!cancellation_accepted &&
             cancel_result.code == rclcpp_action::ResultCode::SUCCEEDED),
        "cancel endpoint preserves pre-commit cancel or committed-success semantics");

  auto final_status_future = status->async_send_request(
      std::make_shared<open_lmm_ros::srv::GetRuntimeStatus::Request>());
  const auto final_status =
      GetWithWatchdog(final_status_future, "final status");
  Check(final_status->success && final_status->config_revision != 0,
        "status remains queryable after action cancellation");

  (void)events;
  executor.remove_node(observer);
  executor.remove_node(adapter);
}

}  // namespace

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  try {
    RunGraphContract();
    rclcpp::shutdown();
    return 0;
  } catch (const std::exception& error) {
    rclcpp::shutdown();
    std::cerr << "ROS graph contract failed: " << error.what() << '\n';
    return 1;
  }
}
