#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <thread>
#include <vector>

namespace open_lmm {

struct VisualizationSnapshotResult {
  VisualizationQuery query;
  uint64_t request_generation = 0;
  Result<VisualizationSnapshot> result;
};

// Describes why the GUI is asking for another projection. Polling must not
// retire useful point work, while a source change must invalidate work built
// from the previous runtime state.
enum class VisualizationRequestIntent : uint8_t {
  kPoll,
  kExpandPoints,
  kSourceChanged,
};

// Serializes potentially expensive PCD snapshot copies away from the viewer
// thread. Polls and point expansions reuse compatible work; only an explicit
// source change supersedes a compatible active generation.
class VisualizationSnapshotWorker {
 public:
  using Provider =
      std::function<Result<VisualizationSnapshot>(const VisualizationQuery&)>;
  using CompletionNotification = std::function<void()>;

  explicit VisualizationSnapshotWorker(
      Provider provider, CompletionNotification completion_notification = {});
  ~VisualizationSnapshotWorker();
  VisualizationSnapshotWorker(const VisualizationSnapshotWorker&) = delete;
  VisualizationSnapshotWorker& operator=(const VisualizationSnapshotWorker&) =
      delete;

  [[nodiscard]] std::optional<uint64_t> Request(
      VisualizationQuery query,
      VisualizationRequestIntent intent =
          VisualizationRequestIntent::kSourceChanged);
  [[nodiscard]] std::vector<VisualizationSnapshotResult> Drain();
  // Cancels work from the retired runtime epoch without stopping the worker.
  // A later Request() starts normally and keeps the monotonically increasing
  // generation namespace.
  void ResetEpoch();
  void Stop();
  void Join();

 private:
  void Run();

  Provider provider_;
  CompletionNotification completion_notification_;
  std::mutex mutex_;
  std::condition_variable ready_;
  struct PendingRequest {
    VisualizationQuery query;
    uint64_t generation = 0;
  };
  std::map<AgentId, PendingRequest> pending_;
  std::vector<VisualizationSnapshotResult> completed_;
  std::optional<VisualizationQuery> active_query_;
  uint64_t active_generation_ = 0;
  std::shared_ptr<CancellationToken> active_cancellation_;
  uint64_t next_generation_ = 1;
  bool stopping_ = false;
  std::thread thread_;
};

}  // namespace open_lmm
