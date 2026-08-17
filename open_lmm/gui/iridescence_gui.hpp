#pragma once
#include <open_lmm/gui/gui_plugin.hpp>
#include <open_lmm/gui/gui_event_queue.hpp>
#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/gui/visualization_repository.hpp>
#include <open_lmm/gui/visualization_snapshot_worker.hpp>
#include <atomic>
#include <array>
#include <condition_variable>
#include <future>
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
  void DrawConfigEditorUi();
  void LoadConfigEditor();
  void SynchronizeModel();
  void RequestVisualization(char agent);
  void DrainVisualizationSnapshots();
  void UpdateDrawables(
      const std::shared_ptr<const VisualizationSnapshot>& snapshot,
      const VisualizationUpdate& update);
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
  std::array<char, 1024> session_config_path_{};
  std::future<std::vector<std::string>> file_dialog_future_;
  std::future<std::string> dataset_dialog_future_;
  std::vector<std::string> recent_config_paths_;
  int config_domain_ = 0;
  uint64_t config_revision_draft_ = 0;
  double last_gui_work_ms_ = 0.0;
  double max_gui_work_ms_ = 0.0;
  std::optional<Eigen::Vector3f> picked_point_;
  std::atomic<bool> stop_requested_{false};
  std::atomic<bool> open_{false};
  std::mutex start_mutex_;
  std::condition_variable start_condition_;
  bool start_finished_ = false;
  std::string start_error_;
  std::thread thread_;
};
}  // namespace open_lmm
