#pragma once
#include <open_lmm/gui/gui_plugin.hpp>
#include "model/gui_event_queue.hpp"
#include "model/gui_model.hpp"
#include "presentation/alignment_presentation_state.hpp"
#include "presentation/map_presentation_state.hpp"
#include "presentation/visualization_repository.hpp"
#include "presentation/visualization_style.hpp"
#include "presentation/visualization_snapshot_worker.hpp"
#include <guik/model_control.hpp>
#include <atomic>
#include <array>
#include <condition_variable>
#include <chrono>
#include <future>
#include <map>
#include <mutex>
#include <string>
#include <thread>

namespace open_lmm {
class IridescenceGui final : public GuiPlugin {
 public:
  ~IridescenceGui() override;
  Result<void> Start(GuiServices services) override;
  [[nodiscard]] bool IsOpen() const override;
  void RequestStop() override;
  void Join() override;
 private:
  void ViewerLoop();
  void DrawPipelineUi();
  void DrawStageConfigModal();
  Result<void> SaveAndApplyConfig();
  void DrawAlignmentUi();
  void DrawRuntimeLogsSection();
  void SynchronizeAlignmentFeedback();
  [[nodiscard]] bool RenderPipelineAlignmentTransform(
      const AgentId& source_agent, const Eigen::Matrix4f& transform);
  void ApplyPipelineAlignmentPreview(const AgentId& source_agent,
                                     const Eigen::Matrix4f& transform);
  void ResetPipelineAlignmentPreview();
  void SetManualAlignmentTransform(const Eigen::Isometry3d& transform);
  void SynchronizeManualAlignmentTransform();
  void ResetVisualizationEpoch();
  void LoadConfigEditor();
  void RefreshDatasetCatalog();
  void LoadAlignmentEditor();
  void SynchronizeModel();
  void RequestVisualization(
      AgentId agent, bool include_points = false,
      VisualizationRequestIntent intent =
          VisualizationRequestIntent::kSourceChanged);
  void RequestAllVisualizationMetadata();
  void DrainVisualizationSnapshots();
  void UpdateDrawables(
      const std::shared_ptr<const VisualizationSnapshot>& snapshot,
      const VisualizationUpdate& update,
      std::optional<uint64_t> request_generation = std::nullopt);
  void UpdateLoopDrawables(
      const std::shared_ptr<const VisualizationSnapshot>& snapshot);
  void HideAlignmentLoopLayers();
  void ApplyVisualizationColorMode();
  void DrawAgentVisibilityControls();
  void SetAgentVisualizationVisible(const AgentId& agent, bool visible);
  [[nodiscard]] bool IsAgentVisualizationVisible(
      const AgentId& agent) const;
  void ReplacePipelineMapLayersWithAlignmentReview();
  GuiServices services_;
  std::shared_ptr<GuiEventQueue> event_queue_;
  ExecutionEventSubscription event_subscription_;
  GuiModel model_;
  std::vector<NodeDescriptor> node_descriptors_;
  VisualizationRepository visualization_;
  std::unique_ptr<VisualizationSnapshotWorker> visualization_worker_;
  std::string command_error_;
  std::string config_editor_status_;
  std::array<char, 512> config_map_server_{};
  std::array<char, 512> config_data_loader_{};
  std::array<char, 512> config_loop_detector_{};
  std::array<char, 512> config_backend_optimizer_{};
  std::array<char, 512> config_dynamic_remover_{};
  std::array<char, 1024> root_dir_path_{};
  std::array<char, 4096> sub_dir_list_{};
  std::array<char, 1024> root_save_dir_{};
  std::future<std::string> dataset_dialog_future_;
  std::future<std::string> output_dialog_future_;
  std::vector<std::string> dataset_catalog_;
  std::vector<std::string> selected_datasets_;
  std::optional<StageId> config_stage_;
  float kiss_voxel_size_ = 2.0F;
  bool kiss_use_quatro_ = false;
  float pose_nn_distance_threshold_ = 10.0F;
  float inter_loop_keyframe_spacing_m_ = 10.0F;
  int config_domain_ = 0;
  uint64_t config_revision_draft_ = 0;
  double last_gui_work_ms_ = 0.0;
  double max_gui_work_ms_ = 0.0;
  int visualization_color_mode_ = kDefaultVisualizationColorMode;
  MapPresentationState map_presentation_;
  AlignmentPresentationState alignment_presentation_;
  std::vector<std::string> runtime_logs_;
  bool runtime_logs_auto_scroll_ = true;
  std::chrono::steady_clock::time_point next_runtime_log_refresh_{};
  std::chrono::steady_clock::time_point next_data_load_preview_refresh_{};
  bool data_load_preview_active_ = false;
  bool alignment_stage_active_ = false;
  bool map_update_stage_active_ = false;
  std::optional<Eigen::Vector3f> picked_point_;
  std::optional<AlignmentFeedbackSnapshot> alignment_feedback_;
  std::unique_ptr<guik::ModelControl> alignment_model_control_;
  uint64_t alignment_request_id_ = 0;
  uint64_t alignment_session_revision_ = 0;
  bool alignment_manual_mode_ = false;
  int alignment_gizmo_operation_ = 0;
  int alignment_gizmo_mode_ = 0;
  std::optional<Eigen::Matrix4d> alignment_kiss_transform_;
  std::optional<Eigen::Matrix4d> alignment_descriptor_transform_;
  std::optional<AgentId> alignment_preview_agent_;
  Eigen::Isometry3d alignment_manual_transform_ = Eigen::Isometry3d::Identity();
  Eigen::Matrix4f alignment_previous_render_matrix_ = Eigen::Matrix4f::Identity();
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> open_{false};
  std::mutex start_mutex_;
  std::condition_variable start_condition_;
  bool start_finished_ = false;
  std::string start_error_;
  std::thread thread_;
};
}  // namespace open_lmm
