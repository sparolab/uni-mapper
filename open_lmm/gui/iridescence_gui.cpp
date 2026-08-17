#include <open_lmm/gui/iridescence_gui.hpp>
#include <glk/pointcloud_buffer.hpp>
#include <glk/primitives/primitives.hpp>
#include <glk/thin_lines.hpp>
#include <guik/viewer/light_viewer.hpp>
#include <imgui.h>
#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <map>
#include <utility>

namespace open_lmm {
namespace {
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
}  // namespace
IridescenceGui::~IridescenceGui() { RequestStop(); Join(); }

Result<void> IridescenceGui::Start(GuiServices services) {
  if (thread_.joinable()) return Result<void>::Failure(Error::InvalidArgument("Iridescence GUI is already started"));
  services_ = std::move(services);
  if (services_.visualization_snapshot) {
    visualization_worker_ = std::make_unique<VisualizationSnapshotWorker>(
        services_.visualization_snapshot, 1'000'000);
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
    SynchronizeModel();
    viewer->register_ui_callback("open_lmm.pipeline",
                                 [this] { DrawPipelineUi(); });
    open_ = true;
    { std::lock_guard lock(start_mutex_); start_finished_ = true; }
    start_condition_.notify_all();
    while (!stop_requested_ && viewer->spin_once()) {
      const auto gui_work_begin = std::chrono::steady_clock::now();
      DrainVisualizationSnapshots();
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
          for (char agent : model_.Agents()) RequestVisualization(agent);
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
    const auto final_queue_stats = event_queue_->Stats();
    std::cout << "[GUI_PROFILE] max_gui_work_ms=" << max_gui_work_ms_
              << " event_backlog=" << final_queue_stats.queued
              << " evicted_events=" << final_queue_stats.evicted_events
              << " visualization_cache_bytes="
              << visualization_.ApproximateBytes() << std::endl;
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
    for (char agent : model_.Agents()) RequestVisualization(agent);
  }
}

void IridescenceGui::RequestVisualization(char agent) {
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

  if (!snapshot->points.empty()) {
    std::vector<Eigen::Vector3f> points;
    std::vector<float> intensities;
    points.reserve(snapshot->points.size());
    intensities.reserve(snapshot->points.size());
    for (const auto& point : snapshot->points) {
      points.emplace_back(point.x, point.y, point.z);
      intensities.push_back(point.intensity);
    }
    auto cloud = std::make_shared<glk::PointCloudBuffer>(points);
    cloud->add_intensity(glk::COLORMAP::TURBO, intensities);
    viewer->update_drawable(
        VisualizationRepository::MapName(snapshot->agent, snapshot->revision),
        cloud, guik::VertexColor());
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

  std::map<std::pair<char, std::size_t>, Eigen::Vector3f> pose_lookup;
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

  if (!command_error_.empty()) {
    ImGui::Separator();
    ImGui::TextWrapped("Error: %s", command_error_.c_str());
  }
  ImGui::End();

  ImGui::Begin("OpenLMM Agents and Artifacts");
  if (picked_point_) {
    ImGui::Text("Picked: %.3f %.3f %.3f", (*picked_point_).x(),
                (*picked_point_).y(), (*picked_point_).z());
  }
  ImGui::Text("Agents");
  for (const char agent : model_.Agents()) {
    ImGui::BulletText("Agent %c", agent);
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
      if (artifact.key.agent) ImGui::Text("%c", *artifact.key.agent);
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

  ImGui::Begin("OpenLMM Config");
  static const char* domains[] = {"Global", "DataLoader", "LoopDetector",
                                  "Optimizer", "DynamicRemover", "MapSave"};
  ImGui::Combo("Domain", &config_domain_, domains, 6);
  ImGui::InputScalar("New revision", ImGuiDataType_U64,
                     &config_revision_draft_);
  const bool invalid_revision =
      config_revision_draft_ <= model_.ConfigRevision();
  if (invalid_revision || !model_.CanSubmitCommand()) ImGui::BeginDisabled();
  if (ImGui::Button("Apply config revision") && services_.apply_config) {
    auto result = services_.apply_config(
        static_cast<ConfigDomain>(config_domain_), config_revision_draft_);
    command_error_ = result ? std::string{} : result.GetError().Message();
  }
  if (invalid_revision || !model_.CanSubmitCommand()) ImGui::EndDisabled();
  ImGui::TextWrapped(
      "This panel commits an already validated config revision. Parameter "
      "editing/schema widgets require the config draft provider.");
  ImGui::End();

  ImGui::Begin("OpenLMM Job and Event Log");
  if (model_.Job()) {
    ImGui::Text("Job: %llu",
                static_cast<unsigned long long>(model_.Job()->id));
    ImGui::TextWrapped("%s", model_.Job()->message.c_str());
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
}  // namespace open_lmm

extern "C" open_lmm::GuiPlugin* create_gui_plugin() { return new open_lmm::IridescenceGui(); }
