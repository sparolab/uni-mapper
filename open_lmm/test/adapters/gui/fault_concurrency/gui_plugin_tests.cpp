#include "support/synchronization.hpp"
#include "support/runtime/runtime_config_fixture.hpp"

#include <adapters/gui/config_editor.hpp>
#include <adapters/gui/gui_controller_bridge.hpp>
#include <adapters/gui/model/gui_event_queue.hpp>
#include <adapters/gui/model/gui_model.hpp>
#include <adapters/gui/gui_plugin_host.hpp>
#include <adapters/gui/presentation/alignment_presentation_state.hpp>
#include <adapters/gui/presentation/map_presentation_state.hpp>
#include <adapters/gui/presentation/visualization_repository.hpp>
#include <adapters/gui/presentation/visualization_snapshot_worker.hpp>
#include <adapters/gui/presentation/visualization_style.hpp>
#include <plugins/host/load_module.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string_view>
#include <thread>

namespace {
namespace fs = std::filesystem;

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

void Require(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

class FakeGui final : public open_lmm::GuiPlugin {
 public:
  open_lmm::Result<void> Start(open_lmm::GuiServices) override {
    started = open = true;
    return open_lmm::Result<void>::Ok();
  }
  bool IsOpen() const override { return open; }
  void RequestStop() override { requested_stop = true; open = false; }
  void Join() override { joined = true; }
  bool started = false;
  bool requested_stop = false;
  bool joined = false;
  bool open = false;
};

void TestGuiModelAndConfigEditor() {
  const auto data_load = open_lmm::DefaultVisualizationPreferences(
      open_lmm::VisualizationPhase::kDataLoad);
  const auto optimization = open_lmm::DefaultVisualizationPreferences(
      open_lmm::VisualizationPhase::kOptimization);
  const auto map_update = open_lmm::DefaultVisualizationPreferences(
      open_lmm::VisualizationPhase::kMapUpdate);
  Require(data_load.trajectory && data_load.pose_axes && data_load.points &&
              optimization.trajectory && optimization.pose_axes &&
              optimization.points &&
              optimization.intra_loops && optimization.inter_loops &&
              map_update.trajectory && map_update.pose_axes &&
              map_update.points && !map_update.intra_loops &&
              !map_update.inter_loops,
          "visualization layers retain stage-specific defaults");
  Require(open_lmm::kVisualizationTrajectoryLineWidth == 1.0F &&
              open_lmm::kVisualizationIntraLoopLineWidth == 5.0F &&
              open_lmm::kVisualizationInterLoopLineWidth == 5.0F &&
              open_lmm::kVisualizationIntraLoopLineWidth ==
                  open_lmm::kVisualizationInterLoopLineWidth &&
              open_lmm::kDefaultVisualizationColorMode == 2,
          "trajectory and loop presentation widths match the UI contract");

  open_lmm::AlignmentPresentationState alignment_presentation;
  Eigen::Matrix4f accepted_a = Eigen::Matrix4f::Identity();
  accepted_a(0, 3) = 10.0F;
  Eigen::Matrix4f accepted_b = Eigen::Matrix4f::Identity();
  accepted_b(1, 3) = 20.0F;
  alignment_presentation.RememberAccepted(Id("A"), accepted_a);
  alignment_presentation.RememberAccepted(Id("B"), accepted_b);
  alignment_presentation.ObserveSnapshot(
      Id("A"), open_lmm::VisualizationPoseKind::kOdometry);
  Require(alignment_presentation.TransformForOdometry(Id("A")) &&
              alignment_presentation.TransformForOdometry(Id("A"))
                  ->isApprox(accepted_a) &&
              alignment_presentation.TransformForOdometry(Id("B"))
                  ->isApprox(accepted_b),
          "delayed odometry redraw preserves every accepted agent transform");
  alignment_presentation.ObserveSnapshot(
      Id("A"), open_lmm::VisualizationPoseKind::kOptimized);
  Require(!alignment_presentation.TransformForOdometry(Id("A")) &&
              alignment_presentation.TransformForOdometry(Id("B")),
          "optimized installation retires only the matching accepted transform");
  alignment_presentation.Clear();
  Require(alignment_presentation.Agents().empty(),
          "alignment presentation reset clears pending accepted transforms");

  std::vector<std::string> selected_datasets{"agent1", "agent2"};
  open_lmm::ClearDatasetSelection(selected_datasets);
  Require(selected_datasets.empty(),
          "DataLoad Clear all mutates only the dataset selection draft");
  open_lmm::ClearDatasetSelection(selected_datasets);
  Require(selected_datasets.empty(), "DataLoad Clear all is idempotent");

  auto document = open_lmm::ConfigEditorDocument::Parse(R"({
    "global":{"config_map_server":"map.json","config_data_loader":"data.json",
    "config_loop_detector":"loop.json","config_backend_optimizer":"optimizer.json",
    "config_dynamic_remover":"remover.json"},
    "directory":{"root_dir_path":"/data","sub_dir_list":["a"],"root_save_dir":"/out"}})");
  Require(document && document.Value().Values(), "GUI config document is schema validated");
  auto mutable_document = std::move(document).Value();
  auto empty_values = mutable_document.Values().Value();
  open_lmm::ClearDatasetSelection(empty_values.sub_dir_list);
  const auto before_empty_apply = mutable_document.CanonicalJson();
  Require(!mutable_document.SetValues(empty_values) &&
              mutable_document.CanonicalJson().Value() ==
                  before_empty_apply.Value(),
          "empty DataLoad draft is rejected without mutating committed editor state");
  Require(!open_lmm::ConfigEditorDocument::Parse("{}").IsOk(),
          "GUI rejects incomplete root config");

  open_lmm::GuiEventQueue queue(2);
  open_lmm::ExecutionEvent progress;
  progress.job_id = 7;
  progress.type = open_lmm::EventType::kProgressUpdated;
  progress.progress_current = 1;
  Require(queue.Push(progress), "queue accepts progress");
  progress.progress_current = 2;
  Require(queue.Push(progress) && queue.Drain(2).front().progress_current == 2,
          "queue coalesces adjacent progress safely");
  progress.agent = Id("A");
  progress.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("A"), "load", open_lmm::AlgorithmProgressPhase::kReadAndFilter,
      1, 2};
  Require(queue.Push(progress), "queue accepts typed progress");
  progress.algorithm_progress->phase =
      open_lmm::AlgorithmProgressPhase::kBuildStaticMap;
  progress.algorithm_progress->total.reset();
  Require(queue.Push(progress) && queue.Drain(2).size() == 2,
          "queue preserves phase transitions as separate events");
  queue.ResetEpoch();
  Require(queue.Stats().queued == 0, "replacement epoch clears queued events");

  auto plugin = std::make_shared<FakeGui>();
  { open_lmm::GuiPluginHost host(plugin); Require(host.Start({}).IsOk(), "plugin host starts"); }
  Require(plugin->requested_stop && plugin->joined, "plugin host stops and joins");
}

void TestPresentationAndRepository() {
  open_lmm::VisualizationRepository repository;
  auto snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  snapshot->agent = Id("A");
  snapshot->revision = 1;
  Require(repository.Commit(snapshot).changed, "visualization revision commits");
  open_lmm::VisualizationRepository candidate_repository;
  Require(candidate_repository.Commit(snapshot).changed,
          "candidate repository accepts its committed baseline");
  auto intermediate = std::make_shared<open_lmm::VisualizationSnapshot>();
  intermediate->agent = Id("A");
  intermediate->revision = 1;
  intermediate->phase = open_lmm::VisualizationPhase::kOptimization;
  intermediate->pose_kind = open_lmm::VisualizationPoseKind::kOptimized;
  Eigen::Isometry3f optimized_pose = Eigen::Isometry3f::Identity();
  optimized_pose.translation().x() = 10.0F;
  intermediate->poses.push_back({0, optimized_pose});
  Require(candidate_repository.Commit(intermediate, 2).changed,
          "same-revision optimizer presentation replaces DataLoad metadata");
  auto revised_intermediate =
      std::make_shared<open_lmm::VisualizationSnapshot>(*intermediate);
  revised_intermediate->poses.front().transform.translation().x() = 11.0F;
  Require(candidate_repository.Commit(revised_intermediate, 3).changed &&
              candidate_repository.Latest(Id("A"))->poses.front()
                      .transform.translation().x() == 11.0F &&
              !candidate_repository.Commit(intermediate, 2).changed,
          "presentation generation accepts the latest same-phase pose only");
  auto rolled_back_candidate =
      std::make_shared<open_lmm::VisualizationSnapshot>(*snapshot);
  Require(candidate_repository.Commit(rolled_back_candidate, 4).changed &&
              candidate_repository.Latest(Id("A"))->phase ==
                  open_lmm::VisualizationPhase::kDataLoad,
          "newer presentation generation accepts same-revision rollback");
  auto second_snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  second_snapshot->agent = Id("B");
  second_snapshot->revision = 1;
  const auto second_update = repository.Commit(second_snapshot);
  Require(second_update.changed && second_update.remove_drawables.empty() &&
              repository.Snapshots().size() == 2 &&
              open_lmm::VisualizationRepository::MapName(Id("A"), 1) !=
                  open_lmm::VisualizationRepository::MapName(Id("B"), 1),
          "adding an agent map cannot replace the preceding agent drawable");

  open_lmm::MapPresentationState presentation;
  presentation.Begin(Id("A"), 1);
  Require(presentation.Phase(Id("A")) ==
              open_lmm::MapPresentationPhase::kPending &&
              !presentation.Visible(Id("A")),
          "presentation begins pending without inventing a visible map");
  const auto first_commit = presentation.Commit(
      Id("A"), 1, "map/A/1",
      open_lmm::VisualizationPointKind::kFilteredScanPreview);
  presentation.Begin(Id("A"), 2);
  const auto stale_commit = presentation.Commit(
      Id("A"), 1, "map/A/stale",
      open_lmm::VisualizationPointKind::kFilteredScanPreview);
  Require(first_commit.accepted && !first_commit.replaced_drawable &&
              presentation.Phase(Id("A")) ==
                  open_lmm::MapPresentationPhase::kPending &&
              !stale_commit.accepted && presentation.Visible(Id("A")) &&
              presentation.Visible(Id("A"))->drawable == "map/A/1",
          "pending replacement keeps visible map and rejects stale payload");
  presentation.FinishWithoutReplacement(Id("A"), 2);
  Require(presentation.Phase(Id("A")) ==
              open_lmm::MapPresentationPhase::kVisible &&
              presentation.Visible(Id("A"))->drawable == "map/A/1",
          "failed replacement returns to visible without clearing old map");
  presentation.Begin(Id("A"), 3);
  const auto replacement = presentation.Commit(
      Id("A"), 3, "map/A/2",
      open_lmm::VisualizationPointKind::kFinalStaticMap);
  Require(replacement.accepted && replacement.replaced_drawable &&
              *replacement.replaced_drawable == "map/A/1" &&
              presentation.Visible(Id("A"))->drawable == "map/A/2",
          "presentation commit atomically identifies old and new drawable");
  presentation.Begin(Id("A"), 30);
  presentation.Begin(Id("B"), 31);
  presentation.CancelPending();
  const auto retired_stage_commit = presentation.Commit(
      Id("A"), 30, "map/A/stale-map-update",
      open_lmm::VisualizationPointKind::kFinalStaticMap);
  Require(!retired_stage_commit.accepted &&
              presentation.Phase(Id("A")) ==
                  open_lmm::MapPresentationPhase::kVisible &&
              presentation.Visible(Id("A"))->drawable == "map/A/2",
          "Alignment transition cancels stale MapUpdate work without clearing the visible map");
  presentation.Begin(Id("B"), 32);
  Require(presentation.Commit(
              Id("B"), 32, "map/B/review-target",
              open_lmm::VisualizationPointKind::kFinalStaticMap)
              .accepted,
          "second visible map is installed for the review replacement fixture");
  const auto replaced_by_review = presentation.DiscardAllVisible();
  Require(replaced_by_review.size() == 2 &&
              std::find(replaced_by_review.begin(), replaced_by_review.end(),
                        "map/A/2") != replaced_by_review.end() &&
              std::find(replaced_by_review.begin(), replaced_by_review.end(),
                        "map/B/review-target") != replaced_by_review.end() &&
              !presentation.Visible(Id("A")) &&
              !presentation.Visible(Id("B")),
          "ready Alignment review replaces all prior pipeline maps in one presentation commit");
  presentation.Begin(Id("A"), 33);
  Require(presentation.Commit(
              Id("A"), 33, "map/A/2",
              open_lmm::VisualizationPointKind::kFinalStaticMap)
              .accepted,
          "visibility fixture restores a presented map after review replacement");
  const auto hidden = presentation.SetVisible(Id("A"), false);
  Require(hidden && *hidden == "map/A/2" &&
              !presentation.IsVisible(Id("A")) &&
              !presentation.Visible(Id("A")),
          "agent visibility removes only the presented drawable");
  presentation.Begin(Id("B"), 4);
  const auto second_agent = presentation.Commit(
      Id("B"), 4, "map/B/1",
      open_lmm::VisualizationPointKind::kFilteredScanPreview);
  const auto reset_maps = presentation.ResetEpoch();
  Require(second_agent.accepted && reset_maps.size() == 1 &&
              reset_maps.front() == "map/B/1" &&
              presentation.Phase(Id("B")) ==
                  open_lmm::MapPresentationPhase::kEmpty &&
              presentation.IsVisible(Id("A")) &&
              presentation.IsVisible(Id("B")),
          "presentation epoch reset returns installed maps and restores default visibility");

  open_lmm::VisualizationRepository epoch_repository;
  auto epoch_snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  epoch_snapshot->agent = Id("A");
  epoch_snapshot->revision = 7;
  epoch_snapshot->poses.resize(2);
  Require(epoch_repository.Commit(epoch_snapshot).changed,
          "epoch repository fixture commits visualization metadata");
  const auto reset_drawables = epoch_repository.ResetEpoch();
  Require(reset_drawables.size() == 6 &&
              std::find(reset_drawables.begin(), reset_drawables.end(),
                        open_lmm::VisualizationRepository::MapName(Id("A"), 7)) !=
                  reset_drawables.end() &&
              std::find(reset_drawables.begin(), reset_drawables.end(),
                        open_lmm::VisualizationRepository::PoseName(
                            Id("A"), 1, 7)) != reset_drawables.end() &&
              epoch_repository.Snapshots().empty() &&
              epoch_repository.ApproximateBytes() == 0,
          "repository epoch reset enumerates every layer and releases metadata");
}

void TestSnapshotWorkerConcurrency() {
  std::mutex request_mutex;
  std::condition_variable request_changed;
  bool release_first_request = false;
  std::vector<open_lmm::VisualizationQuery> observed_queries;
  open_lmm::VisualizationSnapshotWorker snapshot_worker(
      [&](const open_lmm::VisualizationQuery& query) {
        std::unique_lock lock(request_mutex);
        observed_queries.push_back(query);
        request_changed.notify_all();
        if (query.agent == Id("A")) {
          request_changed.wait(lock,
                               [&] { return release_first_request; });
        }
        open_lmm::VisualizationSnapshot result;
        result.agent = query.agent;
        return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
            std::move(result));
      });
  (void)snapshot_worker.Request({Id("A"), false});
  {
    std::unique_lock lock(request_mutex);
    request_changed.wait(lock,
                         [&] { return !observed_queries.empty(); });
  }
  const auto first_b = snapshot_worker.Request(
      {Id("B"), true},
      open_lmm::VisualizationRequestIntent::kExpandPoints);
  const auto polled_b = snapshot_worker.Request(
      {Id("B"), false}, open_lmm::VisualizationRequestIntent::kPoll);
  (void)snapshot_worker.Request(
      {Id("C"), true},
      open_lmm::VisualizationRequestIntent::kExpandPoints);
  {
    std::lock_guard lock(request_mutex);
    release_first_request = true;
  }
  request_changed.notify_all();
  {
    std::unique_lock lock(request_mutex);
    request_changed.wait(lock,
                         [&] { return observed_queries.size() >= 3; });
  }
  snapshot_worker.Stop();
  snapshot_worker.Join();
  Require(first_b && polled_b && *first_b == *polled_b &&
              observed_queries.size() == 3 &&
              observed_queries[1].agent == Id("B") &&
              observed_queries[1].include_points &&
              observed_queries[2].agent == Id("C") &&
              observed_queries[2].include_points,
          "polling preserves point requests and queued agents run FIFO");

  open_lmm::test::PhaseGate point_provider;
  open_lmm::test::ManualResetEvent point_published;
  std::atomic<bool> point_cancelled{false};
  std::atomic<unsigned> point_calls{0};
  open_lmm::VisualizationSnapshotWorker polling_worker(
      [&](const open_lmm::VisualizationQuery& query) {
        point_calls.fetch_add(1, std::memory_order_relaxed);
        const auto cancellation = open_lmm::CurrentCancellationToken();
        point_provider.ArriveAndWait("point provider release");
        if (cancellation->IsCancellationRequested()) {
          point_cancelled.store(true, std::memory_order_release);
          return open_lmm::Result<open_lmm::VisualizationSnapshot>::Failure(
              open_lmm::Error::Cancelled("unexpected poll cancellation"));
        }
        open_lmm::VisualizationSnapshot result;
        result.agent = query.agent;
        result.points_complete = query.include_points;
        return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
            std::move(result));
      },
      [&] { point_published.Signal(); });
  const auto point_generation = polling_worker.Request(
      {Id("A"), true},
      open_lmm::VisualizationRequestIntent::kExpandPoints);
  point_provider.WaitUntilEntered("point provider entered");
  std::optional<uint64_t> poll_generation;
  for (int repeat = 0; repeat != 8; ++repeat) {
    poll_generation = polling_worker.Request(
        {Id("A"), false}, open_lmm::VisualizationRequestIntent::kPoll);
  }
  point_provider.Release();
  point_published.Wait("point result publication");
  // Verify that a poll reuses a completed full request which has not yet been
  // drained.
  const auto completed_poll_generation = polling_worker.Request(
      {Id("A"), false}, open_lmm::VisualizationRequestIntent::kPoll);
  auto polling_results = polling_worker.Drain();
  polling_worker.Stop();
  polling_worker.Join();
  Require(point_generation && poll_generation && completed_poll_generation &&
              *point_generation == *poll_generation &&
              *point_generation == *completed_poll_generation &&
              !point_cancelled.load(std::memory_order_acquire) &&
              point_calls.load(std::memory_order_acquire) == 1 &&
              polling_results.size() == 1 &&
              polling_results.front().request_generation == *point_generation &&
              polling_results.front().result.IsOk() &&
              polling_results.front().result.Value().points_complete,
          "DataLoad metadata polling cannot cancel or supersede point work");

  open_lmm::test::PhaseGate first_provider;
  open_lmm::test::ManualResetEvent latest_published;
  std::atomic<bool> first_cancelled{false};
  std::atomic<unsigned> calls{0};
  open_lmm::VisualizationSnapshotWorker supersession_worker(
      [&](const open_lmm::VisualizationQuery& query) {
        const auto cancellation = open_lmm::CurrentCancellationToken();
        if (calls.fetch_add(1) == 0) {
          first_provider.ArriveAndWait("superseded provider release");
          first_cancelled.store(cancellation->IsCancellationRequested(),
                                std::memory_order_release);
          return open_lmm::Result<open_lmm::VisualizationSnapshot>::Failure(
              open_lmm::Error::Cancelled("superseded fixture"));
        }
        open_lmm::VisualizationSnapshot result;
        result.agent = query.agent;
        return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
            std::move(result));
      },
      [&] { latest_published.Signal(); });
  const auto stale_generation =
      supersession_worker.Request({Id("A"), true});
  first_provider.WaitUntilEntered("superseded provider entered");
  const auto latest_generation =
      supersession_worker.Request({Id("A"), false});
  first_provider.Release();
  latest_published.Wait("latest superseding result publication");
  auto supersession_results = supersession_worker.Drain();
  supersession_worker.Stop();
  supersession_worker.Join();
  Require(stale_generation && latest_generation &&
              first_cancelled.load(std::memory_order_acquire) &&
              supersession_results.size() == 1 &&
              supersession_results.front().request_generation ==
                  *latest_generation,
          "same-agent supersession cancels stale work and publishes only latest");

  open_lmm::test::PhaseGate epoch_old_provider;
  open_lmm::test::ManualResetEvent epoch_new_published;
  std::atomic<bool> epoch_old_cancelled{false};
  std::atomic<unsigned> epoch_pending_calls{0};
  open_lmm::VisualizationSnapshotWorker epoch_worker(
      [&](const open_lmm::VisualizationQuery& query) {
        const auto cancellation = open_lmm::CurrentCancellationToken();
        if (query.agent == Id("A")) {
          epoch_old_provider.ArriveAndWait("retired epoch provider release");
          epoch_old_cancelled.store(cancellation->IsCancellationRequested(),
                                    std::memory_order_release);
          return open_lmm::Result<open_lmm::VisualizationSnapshot>::Failure(
              open_lmm::Error::Cancelled("retired visualization epoch"));
        }
        if (query.agent == Id("B")) {
          epoch_pending_calls.fetch_add(1, std::memory_order_relaxed);
        }
        open_lmm::VisualizationSnapshot result;
        result.agent = query.agent;
        return open_lmm::Result<open_lmm::VisualizationSnapshot>::Ok(
            std::move(result));
      },
      [&] { epoch_new_published.Signal(); });
  (void)epoch_worker.Request({Id("A"), true});
  epoch_old_provider.WaitUntilEntered("retired epoch provider entered");
  (void)epoch_worker.Request({Id("B"), true});
  epoch_worker.ResetEpoch();
  const auto epoch_generation = epoch_worker.Request({Id("C"), false});
  epoch_old_provider.Release();
  epoch_new_published.Wait("new epoch result publication");
  auto epoch_results = epoch_worker.Drain();
  epoch_worker.Stop();
  epoch_worker.Join();
  Require(epoch_generation &&
              epoch_old_cancelled.load(std::memory_order_acquire) &&
              epoch_pending_calls.load(std::memory_order_acquire) == 0 &&
              epoch_results.size() == 1 &&
              epoch_results.front().query.agent == Id("C") &&
              epoch_results.front().request_generation == *epoch_generation,
          "worker epoch reset cancels active work, drops pending work, and publishes only the new epoch");
}

void TestGuiProgressModel() {
  open_lmm::GuiModel progress_model;
  open_lmm::ExecutionEvent started;
  started.sequence = 1;
  started.job_id = 9;
  started.type = open_lmm::EventType::kStageStarted;
  started.stage = open_lmm::StageId::kMapUpdate;
  Require(progress_model.Apply(started), "GUI accepts stage start");
  open_lmm::ExecutionEvent phase;
  phase.sequence = 2;
  phase.job_id = 9;
  phase.type = open_lmm::EventType::kProgressUpdated;
  phase.stage = open_lmm::StageId::kMapUpdate;
  phase.agent = Id("A");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("A"), "map_remove",
      open_lmm::AlgorithmProgressPhase::kInitializeRemover, 0,
      std::nullopt};
  Require(progress_model.Apply(phase) &&
              !progress_model.Stage(open_lmm::StageId::kMapUpdate)
                   .agent_progress.at(Id("A"))
                   .total,
          "GUI preserves an indeterminate remover phase without fake total");
  Require(progress_model.Stage(open_lmm::StageId::kMapUpdate)
                  .latest_progress_agent == Id("A"),
          "MapUpdate selects the latest remover stream for display");
  Require(open_lmm::FormatAlgorithmProgressStatus(
              progress_model.Stage(open_lmm::StageId::kMapUpdate)
                  .agent_progress.at(Id("A"))) ==
              "initialize remover (in progress; total unavailable)",
          "GUI and terminal formatting consume the same progress DTO");

  open_lmm::GuiModel data_load_model;
  started.stage = open_lmm::StageId::kDataLoad;
  Require(data_load_model.Apply(started), "GUI accepts DataLoad stage start");
  phase.stage = open_lmm::StageId::kDataLoad;
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("A"), "load", open_lmm::AlgorithmProgressPhase::kReadAndFilter,
      2, 10};
  Require(data_load_model.Apply(phase), "GUI accepts first DataLoad stream");
  phase.sequence = 3;
  phase.agent = Id("B");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("B"), "load", open_lmm::AlgorithmProgressPhase::kReadAndFilter,
      1, 12};
  Require(data_load_model.Apply(phase) &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                      .agent_progress.size() == 2,
          "DataLoad retains every concurrently active agent stream");
  phase.sequence = 4;
  phase.agent = Id("A");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("A"), "data_load", open_lmm::AlgorithmProgressPhase::kBuildPreview,
      1, 1};
  Require(data_load_model.Apply(phase) &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                      .agent_progress.size() == 1 &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                  .agent_progress.contains(Id("B")),
          "completed DataLoad agent disappears while active peers remain");
  phase.sequence = 5;
  phase.agent = Id("C");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("C"), "load", open_lmm::AlgorithmProgressPhase::kReadAndFilter,
      1, 8};
  Require(data_load_model.Apply(phase) &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                      .agent_progress.size() == 2,
          "a newly admitted DataLoad agent adds one active stream");
  phase.sequence = 6;
  phase.agent = Id("B");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("B"), "data_load", open_lmm::AlgorithmProgressPhase::kBuildPreview,
      1, 1};
  Require(data_load_model.Apply(phase) &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                      .agent_progress.size() == 1 &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                  .agent_progress.contains(Id("C")),
          "DataLoad active stream count follows task completion");
  phase.sequence = 7;
  phase.agent = Id("C");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("C"), "data_load", open_lmm::AlgorithmProgressPhase::kBuildPreview,
      1, 1};
  Require(data_load_model.Apply(phase) &&
              data_load_model.Stage(open_lmm::StageId::kDataLoad)
                  .agent_progress.empty(),
          "DataLoad removes the final bar immediately at agent completion");

  open_lmm::ExecutionEvent completed;
  completed.sequence = 3;
  completed.job_id = 9;
  completed.type = open_lmm::EventType::kStageCompleted;
  completed.stage = open_lmm::StageId::kMapUpdate;
  Require(progress_model.Apply(completed) &&
              progress_model.Stage(open_lmm::StageId::kMapUpdate)
                  .agent_progress.empty() &&
              !progress_model.Stage(open_lmm::StageId::kMapUpdate)
                   .latest_progress_agent &&
              progress_model.Stage(open_lmm::StageId::kMapUpdate)
                      .progress_current == 0 &&
              progress_model.Stage(open_lmm::StageId::kMapUpdate)
                      .progress_total == 0,
          "GUI clears every progress field when the stage completes");

  open_lmm::GuiModel alignment_model;
  started.stage = open_lmm::StageId::kAlignment;
  Require(alignment_model.Apply(started), "GUI accepts Alignment stage start");
  phase.sequence = 2;
  phase.stage = open_lmm::StageId::kAlignment;
  phase.agent = Id("B");
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("B"), "loop_detect",
      open_lmm::AlgorithmProgressPhase::kWaitAlignmentReview, 0,
      std::nullopt};
  Require(alignment_model.Apply(phase) &&
              open_lmm::FormatAlgorithmProgressStatus(
                  alignment_model.Stage(open_lmm::StageId::kAlignment)
                      .agent_progress.at(Id("B"))) ==
                  "wait for alignment review (in progress; total unavailable)",
          "GUI preserves Alignment review as an indeterminate phase");
  phase.sequence = 3;
  phase.algorithm_progress = open_lmm::AlgorithmProgress{
      Id("B"), "loop_detect",
      open_lmm::AlgorithmProgressPhase::kDetectIntraLoops, 4, 10};
  Require(alignment_model.Apply(phase) &&
              open_lmm::FormatAlgorithmProgressStatus(
                  alignment_model.Stage(open_lmm::StageId::kAlignment)
                      .agent_progress.at(Id("B"))) ==
                  "detect intra loops 4/10",
          "GUI replaces the review state with the latest determinate phase");
}

void TestVisualizationRevisionOrdering() {
  open_lmm::VisualizationRepository repository;
  auto loop_snapshot = std::make_shared<open_lmm::VisualizationSnapshot>();
  loop_snapshot->agent = Id("A");
  loop_snapshot->revision = 1;
  loop_snapshot->phase = open_lmm::VisualizationPhase::kLoopDetection;
  Require(repository.Commit(loop_snapshot).changed,
          "newer visualization phase commits within one revision");
  auto stale_phase = std::make_shared<open_lmm::VisualizationSnapshot>();
  stale_phase->agent = Id("A");
  stale_phase->revision = 1;
  stale_phase->phase = open_lmm::VisualizationPhase::kDataLoad;
  Require(!repository.Commit(stale_phase).changed,
          "stale visualization phase cannot replace a newer phase");
  auto enriched = std::make_shared<open_lmm::VisualizationSnapshot>();
  enriched->agent = Id("A");
  enriched->revision = 1;
  enriched->phase = open_lmm::VisualizationPhase::kLoopDetection;
  enriched->points_available = true;
  enriched->points_complete = true;
  enriched->points.push_back({1.0F, 2.0F, 3.0F, 0.0F});
  Require(repository.Commit(enriched).changed,
          "lazy point payload enriches matching revision and phase");
  auto map_update_snapshot =
      std::make_shared<open_lmm::VisualizationSnapshot>(*enriched);
  map_update_snapshot->revision = 2;
  map_update_snapshot->phase = open_lmm::VisualizationPhase::kMapUpdate;
  Require(repository.Commit(map_update_snapshot).changed,
          "MapUpdate visualization revision commits without clearing repository state");
  auto alignment_reentry =
      std::make_shared<open_lmm::VisualizationSnapshot>(*enriched);
  alignment_reentry->revision = 3;
  alignment_reentry->phase = open_lmm::VisualizationPhase::kLoopDetection;
  const auto alignment_reentry_update = repository.Commit(alignment_reentry);
  Require(alignment_reentry_update.changed &&
              repository.Latest(Id("A"))->phase ==
                  open_lmm::VisualizationPhase::kLoopDetection,
          "newer Alignment revision replaces MapUpdate metadata even though its phase rank is lower");
}

void TestSessionFreeBridgeAndModelReplacement() {
  const auto root = fs::temp_directory_path() / "open_lmm_gui_single_runtime";
  fs::remove_all(root);
  open_lmm::test::WriteRuntimeConfigFixture(
      root / "config", root / "data", root / "output-one");
  auto runtime = std::make_shared<open_lmm::RuntimeClient>(1);
  auto opened = runtime->Open({root / "config", "gui"});
  if (!opened) std::cerr << "GUI runtime open: " << opened.GetError().Message() << '\n';
  Require(opened.IsOk(), "GUI opens the single RuntimeClient");
  auto services = open_lmm::MakeGuiServices(
      runtime, (root / "config/config.json").string());
  auto initial = services.snapshot();
  open_lmm::GuiModel model;
  model.Synchronize(initial);

  std::atomic<uint64_t> queued{0};
  open_lmm::test::ManualResetEvent queued_event;
  auto subscription = services.subscribe_events([&](const open_lmm::ExecutionEvent& event) {
    if (event.type == open_lmm::EventType::kJobQueued) {
      queued.store(event.job_id);
      queued_event.Signal();
    }
  });
  Require(subscription.IsOk(), "bridge subscribes without a runtime key");
  auto events = std::move(subscription).Value();
  auto job = services.submit_run_all();
  Require(job && job.Value() != 0, "bridge submits through unkeyed API");
  queued_event.Wait("GUI bridge queued event");
  Require(queued.load() == job.Value() && runtime->Wait({job.Value()}).IsOk(),
          "submitted handle and every queued event use one namespace");
  model.Synchronize(services.snapshot());

  auto root_document = open_lmm::ConfigEditorDocument::Load(root / "config/config.json");
  Require(root_document.IsOk(), "root replacement document loads");
  auto values = root_document.Value().Values();
  Require(values.IsOk(), "root replacement values project");
  auto replacement_values = std::move(values).Value();
  replacement_values.root_save_dir = (root / "output-two").string();
  auto candidate_document = std::move(root_document).Value();
  Require(candidate_document.SetValues(replacement_values).IsOk(),
          "replacement root values validate");
  auto json = candidate_document.CanonicalJson();
  Require(json.IsOk(), "replacement root serializes canonically");
  open_lmm::ConfigCandidate root_candidate;
  root_candidate.domain = open_lmm::ConfigDomain::kGlobal;
  root_candidate.document_json = std::move(json).Value();
  const auto replacement_expected = services.runtime_snapshot().Value().pipeline;
  Require(services.replace_root_config(
              std::move(root_candidate),
              {replacement_expected.runtime_revision,
               replacement_expected.config_revision})
              .IsOk(),
          "GUI uses atomic root replacement rather than config pre-write");

  // Public counters remain monotonic, while state reconstruction is scoped to
  // the new runtime epoch and cannot replay the retired controller's job.
  model.Synchronize(services.snapshot());
  const auto sequence_after_replacement = model.LastSequence();
  Require(sequence_after_replacement == 0 && !model.Job() &&
              services.runtime_snapshot().Value().output_directory.parent_path() ==
                  root / "output-two",
          "replacement clears retired state replay and exposes new output");
  auto second = services.submit_run_all();
  Require(second && second.Value() != job.Value() &&
              runtime->Wait({second.Value()}).IsOk(),
          "job identity remains unique after GUI replacement");
  model.Synchronize(services.snapshot());
  Require(model.LastSequence() > sequence_after_replacement,
          "GUI accepts replacement runtime events after synchronization");
  events.Reset();
  Require(runtime->Close().IsOk(), "GUI-owned runtime closes");
  fs::remove_all(root);
}

}  // namespace

int main(int argc, char** argv) {
  Require(argc == 2, "GUI test suite selector is required");
  const std::string_view suite(argv[1]);
  if (suite == "model") {
    TestGuiModelAndConfigEditor();
    TestGuiProgressModel();
  } else if (suite == "presentation") {
    TestPresentationAndRepository();
    TestVisualizationRevisionOrdering();
  } else if (suite == "worker") {
    TestSnapshotWorkerConcurrency();
  } else if (suite == "bridge") {
    TestSessionFreeBridgeAndModelReplacement();
  } else {
    Require(false, "unknown GUI test suite selector");
  }
  std::cout << "GUI owner suite passed: " << suite << '\n';
  return 0;
}
