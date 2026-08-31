#include "../../ros2/open_lmm_ros/rviz_control/control_panel_model.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << message << '\n';
    std::exit(EXIT_FAILURE);
  }
}
}  // namespace

int main() {
  using namespace open_lmm_ros::control;
  Check(!NodeGoal(Node::kDataLoad, ""),
        "per-agent node accepted an empty agent");
  auto runtime_node = NodeGoal(Node::kPoseSave, "ignored");
  Check(runtime_node && !runtime_node->has_agent,
        "runtime node incorrectly retained an agent");
  auto optimized = OptimizeThroughGoal("agent1");
  Check(optimized && optimized->has_agent && optimized->agent == "agent1",
        "optimize-through goal was encoded incorrectly");

  ControlPanelModel model;
  open_lmm_ros::srv::GetRuntimeStatus::Response ready;
  ready.success = true;
  ready.runtime_state = 1;
  ready.runtime_revision = 11;
  ready.config_revision = 7;
  ready.agents = {"agent1", "agent2"};
  model.ApplyStatus(ready);
  Check(model.Connected() && model.CanSubmit() && !model.CanCancel() &&
            model.RuntimeRevision() == 11 && model.Agents().size() == 2,
        "ready status was not applied");

  ready.has_job = true;
  ready.job_id = 42;
  ready.job_state = 2;
  ready.has_active_stage = true;
  ready.active_stage = 1;
  model.ApplyStatus(ready);
  Check(!model.CanSubmit() && model.CanCancel() && model.JobId() == 42,
        "active status did not gate commands");

  open_lmm_ros::msg::ExecutionEvent first;
  first.job_id = 42;
  first.sequence = 10;
  first.event_type = static_cast<uint8_t>(Event::kJobStarted);
  first.progress_current = 1;
  first.progress_total = 4;
  first.message = "working";
  Check(model.ApplyEvent(first), "first event was treated as a gap");
  Check(!model.CanSubmit() && model.CanCancel(),
        "active event did not gate commands");
  auto duplicate = first;
  Check(model.ApplyEvent(duplicate), "duplicate event was not ignored");
  auto out_of_order = first;
  out_of_order.sequence = 9;
  out_of_order.progress_current = 99;
  Check(model.ApplyEvent(out_of_order) && model.ProgressCurrent() == 1,
        "out-of-order event changed the presentation model");
  auto gap = first;
  gap.sequence = 12;
  Check(!model.ApplyEvent(gap), "event gap did not request resynchronization");

  auto terminal = gap;
  terminal.sequence = 13;
  terminal.event_type = static_cast<uint8_t>(Event::kJobCompleted);
  Check(!model.ApplyEvent(terminal) && !model.CanSubmit() && !model.CanCancel(),
        "terminal event did not require authoritative resynchronization");

  ready.has_job = false;
  model.ApplyStatus(ready);
  first.sequence = 2;
  Check(model.ApplyEvent(first),
        "authoritative status did not reset the event watermark");

  model.Disconnect("gone");
  Check(!model.Connected() && !model.CanSubmit() && !model.CanCancel(),
        "disconnect did not fail closed");
  return EXIT_SUCCESS;
}
