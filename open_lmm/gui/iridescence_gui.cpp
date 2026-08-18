#include <open_lmm/gui/iridescence_gui.hpp>
#include <open_lmm/gui/config_editor.hpp>
#include <open_lmm/utils/logging.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/colormap.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <imgui.h>
#include <iridescence/portable-file-dialogs.h>
#include <algorithm>
#include <chrono>
#include <exception>
#include <map>
#include <cstdio>
#include <sstream>
#include <utility>

namespace open_lmm {
namespace {
constexpr float kAlignmentReviewPointScale = 6.0F;
constexpr float kAlignmentGizmoClipScale = 0.1F;

const char* StageName(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad: return "DataLoad";
    case StageId::kAlignment: return "Alignment";
    case StageId::kMapUpdate: return "MapUpdate";
    case StageId::kSave: return "Save";
  }
  return "Unknown";
}
const char* ArtifactName(ArtifactType type) {
  switch (type) {
    case ArtifactType::kConfigSnapshot: return "ConfigSnapshot";
    case ArtifactType::kAgentInput: return "AgentInput";
    case ArtifactType::kRawData: return "RawData";
    case ArtifactType::kDescriptorState: return "DescriptorState";
    case ArtifactType::kLoopCandidates: return "LoopCandidates";
    case ArtifactType::kMapAlignment: return "MapAlignment";
    case ArtifactType::kOptimizerState: return "OptimizerState";
    case ArtifactType::kOptimizedPoses: return "OptimizedPoses";
    case ArtifactType::kGlobalMap: return "GlobalMap";
    case ArtifactType::kPoseFile: return "PoseFile";
    case ArtifactType::kPcdFile: return "PcdFile";
    case ArtifactType::kProfileRecord: return "ProfileRecord";
  }
  return "Unknown";
}
const char* ArtifactStateName(ArtifactState state) {
  switch (state) {
    case ArtifactState::kMissing: return "Missing";
    case ArtifactState::kReady: return "Ready";
    case ArtifactState::kStale: return "Stale";
    case ArtifactState::kFailed: return "Failed";
  }
  return "Unknown";
}
Eigen::Vector4f AgentColor(const AgentId& agent) {
  static const std::array<Eigen::Vector4f, 8> colors = {
      Eigen::Vector4f(0.90F, 0.20F, 0.20F, 1.0F),
      Eigen::Vector4f(0.20F, 0.65F, 1.00F, 1.0F),
      Eigen::Vector4f(0.20F, 0.85F, 0.35F, 1.0F),
      Eigen::Vector4f(1.00F, 0.65F, 0.10F, 1.0F),
      Eigen::Vector4f(0.65F, 0.35F, 1.00F, 1.0F),
      Eigen::Vector4f(0.10F, 0.85F, 0.85F, 1.0F),
      Eigen::Vector4f(1.00F, 0.35F, 0.75F, 1.0F),
      Eigen::Vector4f(0.75F, 0.75F, 0.20F, 1.0F),
  };
  return colors[std::hash<std::string>{}(agent.Value()) % colors.size()];
}
template <std::size_t N>
void SetBuffer(std::array<char, N>& buffer, const std::string& value) {
  std::snprintf(buffer.data(), buffer.size(), "%s", value.c_str());
}
}  // namespace
IridescenceGui::~IridescenceGui() { RequestStop(); Join(); }

Result<void> IridescenceGui::Start(GuiServices services) {
  if (thread_.joinable()) return Result<void>::Failure(Error::InvalidArgument("Iridescence GUI is already started"));
  services_ = std::move(services);
  LoadConfigEditor();
  if (services_.node_descriptors) {
    node_descriptors_ = services_.node_descriptors();
  }
  if (services_.visualization_snapshot) {
    visualization_worker_ = std::make_unique<VisualizationSnapshotWorker>(
        services_.visualization_snapshot);
  }
  event_queue_ = std::make_shared<GuiEventQueue>();
  if (services_.subscribe_events) {
    std::weak_ptr<GuiEventQueue> weak_queue = event_queue_;
    event_subscription_ = services_.subscribe_events(
        [weak_queue](const ExecutionEvent& event) {
          if (auto queue = weak_queue.lock()) queue->Push(event);
        });
  }
  stop_requested_ = false;
  open_ = false;
  { std::lock_guard lock(start_mutex_); start_finished_ = false; start_error_.clear(); }
  thread_ = std::thread([this] { ViewerLoop(); });
  std::unique_lock lock(start_mutex_);
  if (!start_condition_.wait_for(lock, std::chrono::seconds(2), [this] { return start_finished_; })) {
    lock.unlock(); RequestStop(); Join();
    return Result<void>::Failure(Error::PluginLoadFailed("Iridescence viewer start timed out"));
  }
  if (!start_error_.empty()) {
    const auto error = start_error_; lock.unlock(); Join();
    return Result<void>::Failure(Error::PluginLoadFailed(error));
  }
  return Result<void>::Ok();
}

bool IridescenceGui::IsOpen() const { return open_.load(); }
void IridescenceGui::RequestStop() {
  event_subscription_.Reset();
  if (visualization_worker_) visualization_worker_->Stop();
  stop_requested_ = true;
}
void IridescenceGui::Join() {
  if (thread_.joinable()) thread_.join();
  if (visualization_worker_) visualization_worker_->Join();
}

void IridescenceGui::ViewerLoop() {
  try {
    auto viewer = guik::LightViewer::instance();
    viewer->enable_vsync();
    viewer->shader_setting().set_point_scale_metric();
    viewer->shader_setting().set_point_size(0.025F);
    viewer->shader_setting().set_point_shape_circle();
    SynchronizeModel();
    viewer->register_ui_callback("open_lmm.pipeline",
                                 [this] { DrawPipelineUi(); });
    open_ = true;
    { std::lock_guard lock(start_mutex_); start_finished_ = true; }
    start_condition_.notify_all();
    while (!stop_requested_ && viewer->spin_once()) {
      const auto gui_work_begin = std::chrono::steady_clock::now();
      DrainVisualizationSnapshots();
      SynchronizeAlignmentFeedback();
      const auto events = event_queue_->Drain(128);
      bool needs_resync = false;
      for (const auto& event : events) {
        needs_resync |= !model_.Apply(event);
        if (event.type == EventType::kArtifactCommitted && event.agent) {
          RequestVisualization(*event.agent);
        }
        if (event.type == EventType::kStageCompleted && event.stage &&
            (*event.stage == StageId::kAlignment ||
             *event.stage == StageId::kMapUpdate)) {
          for (const AgentId& agent : model_.Agents()) RequestVisualization(agent);
        }
      }
      if (event_queue_->Stats().resync_required && services_.snapshot) {
        needs_resync = true;
      }
      if (needs_resync) {
        SynchronizeModel();
        event_queue_->MarkResynchronized();
      }
      last_gui_work_ms_ = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - gui_work_begin)
                              .count();
      max_gui_work_ms_ = std::max(max_gui_work_ms_, last_gui_work_ms_);
    }
    if (model_.CanCancel() && model_.Job() && services_.cancel_job) {
      (void)services_.cancel_job(model_.Job()->id);
    }
    const auto final_queue_stats = event_queue_->Stats();
    std::ostringstream profile;
    profile << "[GUI_PROFILE] max_gui_work_ms=" << max_gui_work_ms_
            << " event_backlog=" << final_queue_stats.queued
            << " evicted_events=" << final_queue_stats.evicted_events
            << " visualization_cache_bytes="
            << visualization_.ApproximateBytes();
    LogInfo(profile.str());
    open_ = false;
    guik::LightViewer::destroy();
  } catch (const std::exception& e) {
    open_ = false;
    { std::lock_guard lock(start_mutex_); start_error_ = e.what(); start_finished_ = true; }
    start_condition_.notify_all();
  } catch (...) {
    open_ = false;
    { std::lock_guard lock(start_mutex_); start_error_ = "unknown Iridescence viewer exception"; start_finished_ = true; }
    start_condition_.notify_all();
  }
}

void IridescenceGui::SynchronizeModel() {
  if (services_.snapshot) {
    model_.Synchronize(services_.snapshot());
    config_revision_draft_ = model_.ConfigRevision() + 1;
    for (const AgentId& agent : model_.Agents()) RequestVisualization(agent);
  }
}

void IridescenceGui::RequestVisualization(AgentId agent) {
  if (visualization_worker_) visualization_worker_->Request(agent);
}

void IridescenceGui::DrainVisualizationSnapshots() {
  if (!visualization_worker_) return;
  for (auto& completed : visualization_worker_->Drain()) {
    auto& result = completed.result;
    if (!result) continue;  // Pose artifacts may not exist during initial sync.
    auto snapshot = std::make_shared<const VisualizationSnapshot>(
        std::move(result).Value());
    auto update = visualization_.Commit(snapshot);
    if (update.changed) UpdateDrawables(snapshot, update);
  }
}

void IridescenceGui::UpdateDrawables(
    const std::shared_ptr<const VisualizationSnapshot>& snapshot,
    const VisualizationUpdate& update) {
  auto viewer = guik::LightViewer::instance();
  for (const auto& name : update.remove_drawables) viewer->remove_drawable(name);

  bool has_map_bounds = false;
  float min_z = 0.0F;
  float max_z = 0.0F;
  for (const auto& current : visualization_.Snapshots()) {
    if (!current->has_bounds) continue;
    if (!has_map_bounds) {
      min_z = current->min_bound.z();
      max_z = current->max_bound.z();
      has_map_bounds = true;
    } else {
      min_z = std::min(min_z, current->min_bound.z());
      max_z = std::max(max_z, current->max_bound.z());
    }
  }
  if (has_map_bounds) {
    if (max_z - min_z < 0.01F) {
      min_z -= 0.5F;
      max_z += 0.5F;
    }
    viewer->shader_setting().add<Eigen::Vector2f>(
        "z_range", Eigen::Vector2f(min_z, max_z));
  }

  if (!snapshot->points.empty()) {
    std::vector<Eigen::Vector3f> points;
    std::vector<Eigen::Vector4f> colors;
    points.reserve(snapshot->points.size());
    colors.reserve(snapshot->points.size());
    const Eigen::Vector4f static_color(0.10F, 0.55F, 1.00F, 1.0F);
    const Eigen::Vector4f dynamic_color(1.00F, 0.10F, 0.05F, 1.0F);
    for (const auto& point : snapshot->points) {
      points.emplace_back(point.x, point.y, point.z);
      const float dynamic_weight = std::clamp(point.intensity, 0.0F, 1.0F);
      colors.push_back(static_color * (1.0F - dynamic_weight) +
                       dynamic_color * dynamic_weight);
    }
    auto cloud = std::make_shared<glk::PointCloudBuffer>(points);
    cloud->add_color(colors);
    guik::Rainbow shader_setting;
    if (visualization_color_mode_ == 1) {
      shader_setting.set_color_mode(guik::ColorMode::VERTEX_COLOR);
    } else if (visualization_color_mode_ == 2) {
      shader_setting.set_color_mode(guik::ColorMode::FLAT_COLOR);
      shader_setting.set_color(AgentColor(snapshot->agent));
    }
    viewer->update_drawable(
        VisualizationRepository::MapName(snapshot->agent, snapshot->revision),
        cloud, shader_setting);
  }

  std::vector<Eigen::Vector3f> trajectory;
  trajectory.reserve(snapshot->poses.size());
  for (const auto& pose : snapshot->poses) {
    trajectory.push_back(pose.transform.translation());
  }
  if (trajectory.size() > 1) {
    viewer->update_drawable(
        VisualizationRepository::TrajectoryName(snapshot->agent,
                                                snapshot->revision),
        std::make_shared<glk::ThinLines>(trajectory, true),
        guik::FlatGreen());
  }
  for (std::size_t i = 0; i < snapshot->poses.size(); ++i) {
    viewer->update_drawable(
        VisualizationRepository::PoseName(snapshot->agent, i,
                                          snapshot->revision),
        glk::Primitives::coordinate_system(),
        guik::VertexColor(snapshot->poses[i].transform.matrix()).scale(0.25F));
  }

  std::map<std::pair<AgentId, std::size_t>, Eigen::Vector3f> pose_lookup;
  for (const auto& current : visualization_.Snapshots()) {
    for (const auto& pose : current->poses) {
      pose_lookup[{current->agent, static_cast<std::size_t>(pose.index)}] =
          pose.transform.translation();
    }
  }
  std::vector<Eigen::Vector3f> intra_loops;
  std::vector<Eigen::Vector3f> inter_loops;
  for (const auto& edge : snapshot->edges) {
    if (edge.type == VisualizationEdgeType::kTrajectory) continue;
    const auto from = pose_lookup.find({edge.from_agent, edge.from_index});
    const auto to = pose_lookup.find({edge.to_agent, edge.to_index});
    if (from == pose_lookup.end() || to == pose_lookup.end()) continue;
    auto& lines = edge.type == VisualizationEdgeType::kIntraLoop
                      ? intra_loops
                      : inter_loops;
    lines.push_back(from->second);
    lines.push_back(to->second);
  }
  if (!intra_loops.empty()) {
    viewer->update_drawable(
        VisualizationRepository::IntraLoopName(snapshot->agent,
                                               snapshot->revision),
        std::make_shared<glk::ThinLines>(intra_loops, false),
        guik::FlatOrange());
  }
  if (!inter_loops.empty()) {
    viewer->update_drawable(
        VisualizationRepository::InterLoopName(snapshot->agent,
                                               snapshot->revision),
        std::make_shared<glk::ThinLines>(inter_loops, false), guik::FlatRed());
  }
}

void IridescenceGui::ApplyVisualizationColorMode() {
  auto viewer = guik::LightViewer::instance();
  for (const auto& snapshot : visualization_.Snapshots()) {
    const auto drawable = viewer->find_drawable(
        VisualizationRepository::MapName(snapshot->agent, snapshot->revision));
    if (!drawable.first) continue;
    if (visualization_color_mode_ == 0) {
      drawable.first->set_color_mode(guik::ColorMode::RAINBOW);
    } else if (visualization_color_mode_ == 1) {
      drawable.first->set_color_mode(guik::ColorMode::VERTEX_COLOR);
    } else {
      drawable.first->set_color_mode(guik::ColorMode::FLAT_COLOR);
      drawable.first->set_color(AgentColor(snapshot->agent));
    }
  }
}

void IridescenceGui::DrawPipelineUi() {
  if (auto point = guik::LightViewer::instance()->pick_point()) {
    picked_point_ = *point;
  }
  ImGui::Begin("OpenLMM Pipeline");
  ImGui::Text("Config revision: %llu",
              static_cast<unsigned long long>(model_.ConfigRevision()));
  ImGui::Text("Agents: %zu", model_.Agents().size());
  ImGui::Text("Artifacts: %zu", model_.Artifacts().size());
  const auto queue_stats = event_queue_->Stats();
  ImGui::Text("GUI work: %.3f ms (max %.3f ms)", last_gui_work_ms_,
              max_gui_work_ms_);
  ImGui::Text("Event backlog: %zu, evicted: %llu", queue_stats.queued,
              static_cast<unsigned long long>(queue_stats.evicted_events));
  ImGui::Text("Visualization cache: %.2f MiB",
              static_cast<double>(visualization_.ApproximateBytes()) /
                  (1024.0 * 1024.0));
  const char* color_modes[] = {"RAINBOW (height)", "INTENSITY", "AGENT"};
  if (ImGui::Combo("Map color", &visualization_color_mode_, color_modes,
                   IM_ARRAYSIZE(color_modes))) {
    ApplyVisualizationColorMode();
  }

  if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
  if (ImGui::Button("Run All") && services_.submit_run_all) {
    auto result = services_.submit_run_all();
    command_error_ = result ? std::string{} : result.GetError().Message();
  }
  if (!model_.CanSubmitCommand()) ImGui::EndDisabled();

  if (model_.CanCancel() && model_.Job()) {
    ImGui::SameLine();
    if (ImGui::Button("Cancel") && services_.cancel_job) {
      auto result = services_.cancel_job(model_.Job()->id);
      command_error_ = result ? std::string{} : result.GetError().Message();
    }
  }

  for (StageId stage : {StageId::kDataLoad, StageId::kAlignment,
                        StageId::kMapUpdate, StageId::kSave}) {
    const auto& view = model_.Stage(stage);
    ImGui::Separator();
    ImGui::Text("%s", StageName(stage));
    ImGui::SameLine();
    if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
    if (stage != StageId::kSave) {
      const std::string configure = std::string("Configure##") + StageName(stage);
      if (ImGui::Button(configure.c_str())) {
        config_stage_ = stage;
        ImGui::OpenPopup("Stage Configuration");
      }
      ImGui::SameLine();
    }
    const std::string button = std::string("Run##") + StageName(stage);
    if (ImGui::Button(button.c_str()) && services_.submit_stage) {
      auto result = services_.submit_stage(stage);
      command_error_ = result ? std::string{} : result.GetError().Message();
    }
    if (!model_.CanSubmitCommand()) ImGui::EndDisabled();
    if (view.progress_total > 0) {
      const float fraction = static_cast<float>(view.progress_current) /
                             static_cast<float>(view.progress_total);
      ImGui::ProgressBar(fraction);
    }
    if (!view.message.empty()) ImGui::TextWrapped("%s", view.message.c_str());
  }

  DrawStageConfigModal();

  ImGui::Separator();
  DrawRuntimeLogsSection();

  if (!command_error_.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Error: %s", command_error_.c_str());
  }
  ImGui::End();

  DrawAlignmentUi();

  ImGui::Begin("OpenLMM Agents and Artifacts");
  if (picked_point_) {
    ImGui::Text("Picked: %.3f %.3f %.3f", (*picked_point_).x(),
                (*picked_point_).y(), (*picked_point_).z());
  }
  ImGui::Text("Session nodes");
  for (const auto& descriptor : node_descriptors_) {
    if (descriptor.scope != ExecutionScope::kSession) continue;
    if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
    const std::string label = std::string(descriptor.name) + "##session_node";
    if (ImGui::SmallButton(label.c_str()) && services_.submit_node) {
      auto result = services_.submit_node(descriptor.id, std::nullopt);
      command_error_ = result ? std::string{} : result.GetError().Message();
    }
    if (!model_.CanSubmitCommand()) ImGui::EndDisabled();
    ImGui::SameLine();
  }
  ImGui::NewLine();
  ImGui::Separator();
  ImGui::Text("Agents");
  for (const AgentId& agent : model_.Agents()) {
    ImGui::BulletText("Agent %s", agent.Value().c_str());
    ImGui::PushID(agent.Value().c_str());
    for (const auto& descriptor : node_descriptors_) {
      if (descriptor.scope != ExecutionScope::kPerAgent) continue;
      if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
      const std::string label = std::string(descriptor.name) + "##node";
      if (ImGui::SmallButton(label.c_str()) && services_.submit_node) {
        auto result = services_.submit_node(descriptor.id, agent);
        command_error_ = result ? std::string{} : result.GetError().Message();
      }
      if (!model_.CanSubmitCommand()) ImGui::EndDisabled();
      ImGui::SameLine();
    }
    ImGui::NewLine();
    if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
    if (ImGui::SmallButton("Optimize through##replay") &&
        services_.submit_optimize_through) {
      auto result = services_.submit_optimize_through(agent);
      command_error_ = result ? std::string{} : result.GetError().Message();
    }
    if (!model_.CanSubmitCommand()) ImGui::EndDisabled();
    ImGui::PopID();
  }
  ImGui::Separator();
  if (ImGui::BeginTable("artifacts", 5,
                        ImGuiTableFlags_Borders | ImGuiTableFlags_RowBg)) {
    ImGui::TableSetupColumn("Artifact");
    ImGui::TableSetupColumn("Agent");
    ImGui::TableSetupColumn("State");
    ImGui::TableSetupColumn("Revision");
    ImGui::TableSetupColumn("Detail");
    ImGui::TableHeadersRow();
    for (const auto& artifact : model_.Artifacts()) {
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(0);
      ImGui::TextUnformatted(ArtifactName(artifact.key.type));
      ImGui::TableSetColumnIndex(1);
      if (artifact.key.agent) {
        ImGui::Text("%s", artifact.key.agent->Value().c_str());
      }
      else ImGui::TextUnformatted("-");
      ImGui::TableSetColumnIndex(2);
      ImGui::TextUnformatted(ArtifactStateName(artifact.state));
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%llu", static_cast<unsigned long long>(artifact.revision));
      ImGui::TableSetColumnIndex(4);
      ImGui::TextWrapped("%s", artifact.detail.c_str());
    }
    ImGui::EndTable();
  }
  ImGui::End();

  ImGui::Begin("OpenLMM Job and Event Log");
  if (model_.Job()) {
    ImGui::Text("Job: %llu",
                static_cast<unsigned long long>(model_.Job()->id));
    ImGui::TextWrapped("%s", model_.Job()->message.c_str());
    const auto& cancellation = model_.Job()->cancellation;
    ImGui::Text("Cancellation: %s%s",
                cancellation.capability.cooperative ? "cooperative"
                                                    : "non-cooperative",
                cancellation.Pending() ? " (pending)" : "");
    if (cancellation.cancel_requested_at_unix_ns) {
      ImGui::Text("requested=%lld observed=%lld completed=%lld",
                  static_cast<long long>(
                      *cancellation.cancel_requested_at_unix_ns),
                  static_cast<long long>(
                      cancellation.cancel_observed_at_unix_ns.value_or(0)),
                  static_cast<long long>(
                      cancellation.cancel_completed_at_unix_ns.value_or(0)));
    }
  } else {
    ImGui::TextUnformatted("No job submitted");
  }
  ImGui::Separator();
  ImGui::BeginChild("events");
  for (const auto& event : model_.EventLog()) {
    ImGui::Text("#%llu job=%llu %s",
                static_cast<unsigned long long>(event.sequence),
                static_cast<unsigned long long>(event.job_id),
                event.message.c_str());
  }
  ImGui::EndChild();
  ImGui::End();

}

void IridescenceGui::DrawRuntimeLogsSection() {
  if (!ImGui::CollapsingHeader("Runtime Logs",
                               ImGuiTreeNodeFlags_DefaultOpen)) return;
  const auto now = std::chrono::steady_clock::now();
  bool refreshed = false;
  if (now >= next_runtime_log_refresh_) {
    runtime_logs_ = RecentRuntimeLogs();
    next_runtime_log_refresh_ = now + std::chrono::milliseconds(250);
    refreshed = true;
  }
  ImGui::Text("Buffered lines: %zu", runtime_logs_.size());
  ImGui::SameLine();
  ImGui::Checkbox("Auto-scroll", &runtime_logs_auto_scroll_);
  ImGui::Separator();
  ImGui::BeginChild("runtime_logs", ImVec2(0, 180.0F), true,
                    ImGuiWindowFlags_HorizontalScrollbar);
  for (const auto& line : runtime_logs_) ImGui::TextUnformatted(line.c_str());
  if (refreshed && runtime_logs_auto_scroll_) ImGui::SetScrollHereY(1.0F);
  ImGui::EndChild();
}

void IridescenceGui::SynchronizeAlignmentFeedback() {
  if (!services_.alignment_feedback_snapshot) return;
  auto snapshot = services_.alignment_feedback_snapshot();
  auto viewer = guik::LightViewer::instance();
  if (!snapshot) {
    if (alignment_request_id_ != 0) {
      viewer->remove_drawable("open_lmm.alignment.target");
      viewer->remove_drawable("open_lmm.alignment.source");
      viewer->remove_drawable("open_lmm.alignment.target_trajectory");
      viewer->remove_drawable("open_lmm.alignment.source_trajectory");
      viewer->remove_drawable("open_lmm.alignment.inlier_loops");
      viewer->remove_drawable("open_lmm.alignment.outlier_loops");
      alignment_feedback_.reset();
      alignment_model_control_.reset();
      alignment_request_id_ = 0;
      alignment_manual_mode_ = false;
    }
    return;
  }
  if (snapshot->proposal.request_id != alignment_request_id_) {
    alignment_feedback_ = std::move(snapshot);
    alignment_request_id_ = alignment_feedback_->proposal.request_id;
    alignment_manual_mode_ =
        alignment_feedback_->proposal.method == AlignmentMethod::kManual;
    alignment_model_control_ = std::make_unique<guik::ModelControl>(
        "Map alignment",
        alignment_feedback_->proposal.target_T_source.matrix().cast<float>());
    alignment_model_control_->set_gizmo_clip_scale(
        kAlignmentGizmoClipScale);
    alignment_manual_transform_ =
        alignment_feedback_->proposal.target_T_source;
    alignment_previous_render_matrix_ =
        alignment_manual_transform_.matrix().cast<float>();
    alignment_gizmo_operation_ = 0;
    alignment_gizmo_mode_ = 0;
    alignment_model_control_->set_gizmo_operation("TRANSLATE");
    alignment_model_control_->set_gizmo_mode(alignment_gizmo_mode_);
    if (alignment_feedback_->proposal.method == AlignmentMethod::kKissMatcher) {
      alignment_kiss_transform_ =
          alignment_feedback_->proposal.target_T_source.matrix();
      alignment_descriptor_transform_.reset();
    } else if (alignment_feedback_->proposal.method ==
               AlignmentMethod::kDescriptor) {
      alignment_descriptor_transform_ =
          alignment_feedback_->proposal.target_T_source.matrix();
    }
    if (alignment_manual_mode_) alignment_model_control_->enable_gizmo();
    else alignment_model_control_->disable_gizmo();

    std::vector<Eigen::Vector3f> target;
    std::vector<Eigen::Vector3f> source;
    target.reserve(alignment_feedback_->target_points.size());
    source.reserve(alignment_feedback_->source_points.size());
    for (const auto& point : alignment_feedback_->target_points) {
      target.emplace_back(point.x, point.y, point.z);
    }
    for (const auto& point : alignment_feedback_->source_points) {
      source.emplace_back(point.x, point.y, point.z);
    }
    viewer->update_drawable(
        "open_lmm.alignment.target",
        std::make_shared<glk::PointCloudBuffer>(target),
        guik::FlatBlue().set_point_scale(kAlignmentReviewPointScale));
    viewer->update_drawable(
        "open_lmm.alignment.source",
        std::make_shared<glk::PointCloudBuffer>(source),
        guik::FlatOrange(alignment_model_control_->model_matrix())
            .set_point_scale(kAlignmentReviewPointScale));

    viewer->remove_drawable("open_lmm.alignment.target_trajectory");
    viewer->remove_drawable("open_lmm.alignment.source_trajectory");
    viewer->remove_drawable("open_lmm.alignment.inlier_loops");
    viewer->remove_drawable("open_lmm.alignment.outlier_loops");
    if (alignment_feedback_->proposal.method == AlignmentMethod::kDescriptor) {
      const auto to_points = [](const auto& values) {
        std::vector<Eigen::Vector3f> points;
        points.reserve(values.size());
        for (const auto& point : values) {
          points.emplace_back(point.x, point.y, point.z);
        }
        return points;
      };
      const auto target_trajectory =
          to_points(alignment_feedback_->diagnostics.target_trajectory);
      const auto source_trajectory =
          to_points(alignment_feedback_->diagnostics.source_trajectory);
      if (target_trajectory.size() > 1) {
        viewer->update_drawable(
            "open_lmm.alignment.target_trajectory",
            std::make_shared<glk::ThinLines>(target_trajectory, true),
            guik::FlatBlue());
      }
      if (source_trajectory.size() > 1) {
        viewer->update_drawable(
            "open_lmm.alignment.source_trajectory",
            std::make_shared<glk::ThinLines>(source_trajectory, true),
            guik::FlatOrange());
      }
      std::vector<Eigen::Vector3f> inlier_lines;
      std::vector<Eigen::Vector3f> outlier_lines;
      for (const auto& loop :
           alignment_feedback_->diagnostics.descriptor_loops) {
        auto& lines = loop.inlier ? inlier_lines : outlier_lines;
        lines.emplace_back(loop.target.x, loop.target.y, loop.target.z);
        lines.emplace_back(loop.source.x, loop.source.y, loop.source.z);
      }
      if (!inlier_lines.empty()) {
        viewer->update_drawable(
            "open_lmm.alignment.inlier_loops",
            std::make_shared<glk::ThinLines>(inlier_lines, false),
            guik::FlatGreen());
      }
      if (!outlier_lines.empty()) {
        viewer->update_drawable(
            "open_lmm.alignment.outlier_loops",
            std::make_shared<glk::ThinLines>(outlier_lines, false),
            guik::FlatRed());
      }
    }
  }
  if (alignment_model_control_) {
    viewer->update_drawable(
        "open_lmm.alignment.source",
        viewer->find_drawable("open_lmm.alignment.source").second,
        guik::FlatOrange(alignment_model_control_->model_matrix())
            .set_point_scale(kAlignmentReviewPointScale));
  }
}

void IridescenceGui::SetManualAlignmentTransform(
    const Eigen::Isometry3d& transform) {
  alignment_manual_transform_ = transform;
  alignment_previous_render_matrix_ = transform.matrix().cast<float>();
  if (alignment_model_control_) {
    alignment_model_control_->set_model_matrix(
        alignment_previous_render_matrix_);
  }
}

void IridescenceGui::SynchronizeManualAlignmentTransform() {
  if (!alignment_model_control_) return;
  const Eigen::Matrix4f rendered = alignment_model_control_->model_matrix();
  if (rendered.isApprox(alignment_previous_render_matrix_)) return;
  const Eigen::Isometry3f previous(alignment_previous_render_matrix_);
  const Eigen::Isometry3f current(rendered);
  const Eigen::Isometry3d delta =
      (current * previous.inverse()).cast<double>();
  alignment_manual_transform_ = delta * alignment_manual_transform_;
  Eigen::Quaterniond rotation(alignment_manual_transform_.linear());
  alignment_manual_transform_.linear() =
      rotation.normalized().toRotationMatrix();
  alignment_previous_render_matrix_ =
      alignment_manual_transform_.matrix().cast<float>();
  alignment_model_control_->set_model_matrix(
      alignment_previous_render_matrix_);
}

void IridescenceGui::DrawAlignmentUi() {
  if (!alignment_feedback_ || !alignment_model_control_) return;
  ImGui::Begin("Map Alignment Review");
  const auto& proposal = alignment_feedback_->proposal;
  const char* method = proposal.method == AlignmentMethod::kPending
                           ? "Not attempted"
                           : proposal.method == AlignmentMethod::kKissMatcher
                           ? "KISS Matcher"
                           : proposal.method == AlignmentMethod::kDescriptor
                                 ? "Descriptor"
                                 : "Manual";
  ImGui::Text("Agent: %s <- %s", proposal.target_agent.Value().c_str(),
              proposal.source_agent.Value().c_str());
  ImGui::Text("Method: %s", method);
  if (proposal.method == AlignmentMethod::kDescriptor) {
    const auto& loops = alignment_feedback_->diagnostics.descriptor_loops;
    std::size_t inliers = 0;
    for (const auto& loop : loops) {
      if (loop.inlier) ++inliers;
    }
    ImGui::TextColored(ImVec4(0.2F, 0.9F, 0.3F, 1.0F),
                       "Inlier loops: %zu", inliers);
    ImGui::SameLine();
    ImGui::TextColored(ImVec4(0.95F, 0.2F, 0.2F, 1.0F),
                       "Outliers: %zu", loops.size() - inliers);
  }
  ImGui::Text("Rotation inliers: %zu", proposal.metrics.rotation_inliers);
  ImGui::Text("Final inliers: %zu", proposal.metrics.final_inliers);
  ImGui::Text("Map correspondences: %zu",
              proposal.metrics.correspondence_count);
  ImGui::Text("Consensus: %zu", proposal.metrics.consensus_size);
  if (proposal.metrics.overlap_ratio) {
    ImGui::Text("Map overlap: %.1f%%",
                *proposal.metrics.overlap_ratio * 100.0);
  } else {
    ImGui::TextUnformatted("Map overlap: unavailable");
  }
  if (proposal.metrics.fitness) {
    ImGui::Text("NN fitness (mean squared distance): %.4f",
                *proposal.metrics.fitness);
  } else {
    ImGui::TextUnformatted("NN fitness: unavailable");
  }
  if ((proposal.metrics.overlap_ratio &&
       *proposal.metrics.overlap_ratio < 0.2) ||
      (proposal.metrics.fitness && *proposal.metrics.fitness > 4.0)) {
    ImGui::TextColored(ImVec4(1.0F, 0.65F, 0.0F, 1.0F),
                       "Warning: weak map-level alignment quality");
  }

  if (alignment_manual_mode_) {
    alignment_model_control_->draw_gizmo();
    SynchronizeManualAlignmentTransform();
    ImGui::Separator();
    ImGui::TextUnformatted(
        "Move the orange source cloud onto the blue target cloud.");
    if (ImGui::RadioButton("Translate", alignment_gizmo_operation_ == 0)) {
      alignment_gizmo_operation_ = 0;
      alignment_model_control_->set_gizmo_operation("TRANSLATE");
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Rotate", alignment_gizmo_operation_ == 1)) {
      alignment_gizmo_operation_ = 1;
      alignment_model_control_->set_gizmo_operation("ROTATE");
    }
    if (ImGui::RadioButton("Local axes", alignment_gizmo_mode_ == 0)) {
      alignment_gizmo_mode_ = 0;
      alignment_model_control_->set_gizmo_mode(alignment_gizmo_mode_);
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("World axes", alignment_gizmo_mode_ == 1)) {
      alignment_gizmo_mode_ = 1;
      alignment_model_control_->set_gizmo_mode(alignment_gizmo_mode_);
    }
  }

  const auto respond = [this](AlignmentDecision decision,
                              std::optional<Eigen::Isometry3d> transform =
                                  std::nullopt) {
    if (!services_.respond_to_alignment || !model_.Job()) return;
    AlignmentResponse response;
    response.request_id = alignment_request_id_;
    response.decision = decision;
    response.manual_target_T_source = std::move(transform);
    auto result = services_.respond_to_alignment(model_.Job()->id,
                                                  std::move(response));
    command_error_ = result ? std::string{} : result.GetError().Message();
  };

  if (!alignment_manual_mode_) {
    if (proposal.method != AlignmentMethod::kPending) {
      if (ImGui::Button("Accept")) respond(AlignmentDecision::kAccept);
      ImGui::SameLine();
    }
    if (proposal.method != AlignmentMethod::kKissMatcher) {
      if (ImGui::Button("Try KISS Matcher")) {
        respond(AlignmentDecision::kTryKissMatcher);
      }
      ImGui::SameLine();
    }
    if (proposal.method != AlignmentMethod::kDescriptor) {
      if (ImGui::Button("Try Descriptor")) {
        respond(AlignmentDecision::kTryDescriptor);
      }
      ImGui::SameLine();
    }
    if (ImGui::Button("Manual Align")) {
      alignment_manual_mode_ = true;
      SetManualAlignmentTransform(proposal.target_T_source);
      alignment_model_control_->enable_gizmo();
    }
  } else {
    if (ImGui::Button("Apply Manual Transform")) {
      SynchronizeManualAlignmentTransform();
      respond(AlignmentDecision::kManual, alignment_manual_transform_);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Proposal")) {
      SetManualAlignmentTransform(proposal.target_T_source);
    }
    ImGui::SameLine();
    if (ImGui::Button("Reset Identity")) {
      const Eigen::Matrix4d identity = Eigen::Matrix4d::Identity();
      SetManualAlignmentTransform(Eigen::Isometry3d(identity));
    }
    if (alignment_kiss_transform_) {
      ImGui::SameLine();
      if (ImGui::Button("Reset KISS")) {
        SetManualAlignmentTransform(
            Eigen::Isometry3d(*alignment_kiss_transform_));
      }
    }
    if (alignment_descriptor_transform_) {
      ImGui::SameLine();
      if (ImGui::Button("Reset Descriptor")) {
        SetManualAlignmentTransform(
            Eigen::Isometry3d(*alignment_descriptor_transform_));
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Back")) {
      alignment_manual_mode_ = false;
      alignment_model_control_->disable_gizmo();
      SetManualAlignmentTransform(proposal.target_T_source);
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel Alignment")) {
    respond(AlignmentDecision::kCancel);
  }
  ImGui::End();
}

void IridescenceGui::LoadConfigEditor() {
  if (services_.config_file_path.empty()) return;
  auto loaded = ConfigEditorDocument::Load(services_.config_file_path);
  if (!loaded) { config_editor_status_ = loaded.GetError().Message(); return; }
  auto values = loaded.Value().Values();
  if (!values) { config_editor_status_ = values.GetError().Message(); return; }
  const auto& value = values.Value();
  SetBuffer(config_map_server_, value.config_map_server);
  SetBuffer(config_data_loader_, value.config_data_loader);
  SetBuffer(config_loop_detector_, value.config_loop_detector);
  SetBuffer(config_backend_optimizer_, value.config_backend_optimizer);
  SetBuffer(config_dynamic_remover_, value.config_dynamic_remover);
  SetBuffer(root_dir_path_, value.root_dir_path);
  SetBuffer(root_save_dir_, value.root_save_dir);
  std::ostringstream agents;
  for (std::size_t i = 0; i < value.sub_dir_list.size(); ++i) {
    if (i) agents << '\n';
    agents << value.sub_dir_list[i];
  }
  SetBuffer(sub_dir_list_, agents.str());
  selected_datasets_ = value.sub_dir_list;
  RefreshDatasetCatalog();
  LoadAlignmentEditor();
  config_editor_status_ = "Loaded " + services_.config_file_path;
}

void IridescenceGui::RefreshDatasetCatalog() {
  auto discovered = DiscoverDatasetDirectories(root_dir_path_.data());
  if (!discovered) {
    dataset_catalog_.clear();
    config_editor_status_ = discovered.GetError().Message();
    return;
  }
  dataset_catalog_ = std::move(discovered).Value();
}

void IridescenceGui::LoadAlignmentEditor() {
  const auto config_file = std::filesystem::path(services_.config_file_path);
  auto alignment = LoadAlignmentConfig(
      config_file.parent_path() / config_loop_detector_.data());
  if (!alignment) {
    config_editor_status_ = alignment.GetError().Message();
    return;
  }
  kiss_voxel_size_ = static_cast<float>(alignment.Value().kiss_voxel_size);
  kiss_use_quatro_ = alignment.Value().kiss_use_quatro;
  pose_nn_distance_threshold_ =
      static_cast<float>(alignment.Value().pose_nn_distance_threshold);
}

Result<void> IridescenceGui::SaveAndApplyConfig() {
  if (!config_stage_) {
    return Result<void>::Failure(
        Error::InvalidArgument("configuration stage is not selected"));
  }
  auto loaded = ConfigEditorDocument::Load(services_.config_file_path);
  if (!loaded) return Result<void>::Failure(loaded.GetError());
  auto document = std::move(loaded).Value();
  ConfigEditorValues value{
      config_map_server_.data(), config_data_loader_.data(),
      config_loop_detector_.data(), config_backend_optimizer_.data(),
      config_dynamic_remover_.data(), root_dir_path_.data(),
      selected_datasets_, root_save_dir_.data()};
  auto result = document.SetValues(value);
  if (result) result = document.Save();
  if (result && *config_stage_ == StageId::kAlignment) {
    result = SaveAlignmentConfig(
        std::filesystem::path(services_.config_file_path).parent_path() /
            config_loop_detector_.data(),
        AlignmentConfigValues{kiss_voxel_size_, kiss_use_quatro_,
                              pose_nn_distance_threshold_});
  }
  if (result && *config_stage_ == StageId::kDataLoad) {
    if (!services_.create_session) {
      return Result<void>::Failure(
          Error::InvalidArgument("session creation is unavailable"));
    }
    result = services_.create_session(services_.config_file_path);
  } else if (result) {
    if (!services_.apply_config) {
      return Result<void>::Failure(
          Error::InvalidArgument("stage reconfiguration is unavailable"));
    }
    const auto domain = *config_stage_ == StageId::kAlignment
                            ? ConfigDomain::kLoopDetector
                            : ConfigDomain::kDynamicRemover;
    result = services_.apply_config(domain, model_.ConfigRevision() + 1);
  }
  if (result) SynchronizeModel();
  return result;
}

void IridescenceGui::DrawStageConfigModal() {
  if (!config_stage_) return;
  ImGui::SetNextWindowSize(ImVec2(620.0F, 520.0F), ImGuiCond_FirstUseEver);
  bool open = true;
  if (!ImGui::BeginPopupModal("Stage Configuration", &open,
                              ImGuiWindowFlags_NoCollapse)) {
    if (!open) config_stage_.reset();
    return;
  }
  ImGui::Text("%s Configuration", StageName(*config_stage_));
  ImGui::TextDisabled("Active config: %s", services_.config_file_path.c_str());
  ImGui::Separator();

  if (*config_stage_ == StageId::kDataLoad) {
  ImGui::TextUnformatted("Module: File based");
  ImGui::TextUnformatted("Dataset root");
  if (ImGui::Button("Select Dataset Root...") &&
      !dataset_dialog_future_.valid()) {
    const std::string initial = root_dir_path_.data();
    dataset_dialog_future_ = std::async(std::launch::async, [initial] {
      return pfd::select_folder("Select dataset root", initial).result();
    });
  }
  if (dataset_dialog_future_.valid() &&
      dataset_dialog_future_.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready) {
    auto selected = dataset_dialog_future_.get();
    if (!selected.empty()) {
      SetBuffer(root_dir_path_, selected);
      selected_datasets_.clear();
      RefreshDatasetCatalog();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Refresh datasets")) RefreshDatasetCatalog();
  ImGui::SameLine();
  ImGui::Text("Selected dataset root: %s", root_dir_path_.data());
  ImGui::TextUnformatted("Datasets / agents (execution order follows this list)");
  if (ImGui::BeginTable("dataset_catalog", 2,
                        ImGuiTableFlags_SizingStretchSame |
                            ImGuiTableFlags_BordersInnerV)) {
    for (const auto& dataset : dataset_catalog_) {
      ImGui::TableNextColumn();
      bool selected = std::find(selected_datasets_.begin(),
                                selected_datasets_.end(), dataset) !=
                      selected_datasets_.end();
      if (ImGui::Checkbox((dataset + "##dataset").c_str(), &selected)) {
        if (selected) {
          selected_datasets_.push_back(dataset);
        } else {
          selected_datasets_.erase(
              std::remove(selected_datasets_.begin(), selected_datasets_.end(),
                          dataset), selected_datasets_.end());
        }
      }
    }
    ImGui::EndTable();
  }
  if (dataset_catalog_.empty())
    ImGui::TextDisabled("No child dataset directories found.");
  ImGui::Separator();
  ImGui::TextUnformatted("Output root");
  if (ImGui::Button("Select Output Root...") &&
      !output_dialog_future_.valid()) {
    const std::string initial = root_save_dir_.data();
    output_dialog_future_ = std::async(std::launch::async, [initial] {
      return pfd::select_folder("Select output root", initial).result();
    });
  }
  if (output_dialog_future_.valid() &&
      output_dialog_future_.wait_for(std::chrono::milliseconds(0)) ==
          std::future_status::ready) {
    auto selected = output_dialog_future_.get();
    if (!selected.empty()) SetBuffer(root_save_dir_, selected);
  }
  ImGui::SameLine();
  ImGui::Text("Selected output root: %s", root_save_dir_.data());
  } else if (*config_stage_ == StageId::kAlignment) {
    static constexpr const char* kDescriptors[] = {
        "Scan Context", "SOLiD", "STD"};
    static constexpr const char* kDescriptorPaths[] = {
        "core/loop_detector/scan_context.json",
        "core/loop_detector/solid.json", "core/loop_detector/std.json"};
    int selected_descriptor = 0;
    for (int i = 0; i < 3; ++i) {
      if (std::string(config_loop_detector_.data()) == kDescriptorPaths[i])
        selected_descriptor = i;
    }
    if (ImGui::Combo("Descriptor module", &selected_descriptor,
                     kDescriptors, 3)) {
      SetBuffer(config_loop_detector_, kDescriptorPaths[selected_descriptor]);
      LoadAlignmentEditor();
    }
    ImGui::DragFloat("KISS voxel size (m)", &kiss_voxel_size_, 0.05F,
                     0.05F, 10.0F, "%.2f");
    ImGui::Checkbox("KISS use Quatro", &kiss_use_quatro_);
    ImGui::DragFloat("Pose NN max distance (m)",
                     &pose_nn_distance_threshold_, 0.1F, 0.1F, 100.0F,
                     "%.1f");
    ImGui::Separator();
    ImGui::TextUnformatted("Module: Incremental backend optimizer");
  } else if (*config_stage_ == StageId::kMapUpdate) {
    static constexpr const char* kRemovers[] = {
        "ERASOR", "DUFOMap", "FreeDOM", "HMM-MOS", "OTD"};
    static constexpr const char* kRemoverPaths[] = {
        "core/dynamic_remover/erasor.json",
        "core/dynamic_remover/dufomap.json",
        "core/dynamic_remover/free_dom.json",
        "core/dynamic_remover/hmm_mos.json",
        "core/dynamic_remover/otd.json"};
    int selected_remover = 0;
    for (int i = 0; i < 5; ++i) {
      if (std::string(config_dynamic_remover_.data()) == kRemoverPaths[i])
        selected_remover = i;
    }
    if (ImGui::Combo("Dynamic remover module", &selected_remover,
                     kRemovers, 5))
      SetBuffer(config_dynamic_remover_, kRemoverPaths[selected_remover]);
  }

  ImGui::Separator();
  if (!model_.CanSubmitCommand()) ImGui::BeginDisabled();
  if (ImGui::Button("Apply")) {
    auto result = SaveAndApplyConfig();
    if (result) {
      config_editor_status_ = *config_stage_ == StageId::kDataLoad
          ? "DataLoad configuration applied to a new session."
          : "Stage configuration applied; upstream results were preserved.";
      ImGui::CloseCurrentPopup();
      config_stage_.reset();
    } else {
      config_editor_status_ = result.GetError().Message();
    }
  }
  if (!model_.CanSubmitCommand()) ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel")) {
    LoadConfigEditor();
    ImGui::CloseCurrentPopup();
    config_stage_.reset();
  }
  if (!config_editor_status_.empty())
    ImGui::TextWrapped("%s", config_editor_status_.c_str());
  ImGui::EndPopup();
  if (!open) config_stage_.reset();
}
}  // namespace open_lmm

#include <open_lmm/common/plugin_api.h>

namespace {
void* CreateGui(const OpenLmmPluginConfigV1*) noexcept {
  try {
    open_lmm::GuiPlugin* plugin = new open_lmm::IridescenceGui();
    return static_cast<void*>(plugin);
  } catch (...) {
    return nullptr;
  }
}
void DestroyGui(void* value) noexcept {
  delete static_cast<open_lmm::GuiPlugin*>(value);
}
const OpenLmmPluginApiV1 kGuiApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "gui", "iridescence",
    &CreateGui, &DestroyGui, "gui:desktop", 1, "open-lmm-1.0"};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kGuiApi;
}
