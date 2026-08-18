#pragma once

#include <open_lmm/common/result.hpp>
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
  AgentId agent;
  uint64_t request_generation = 0;
  Result<VisualizationSnapshot> result;
};

// Serializes potentially expensive PCD snapshot copies away from the viewer
// thread. Per-agent pending requests are coalesced, while a request arriving
// during an active copy is retained as the next generation.
class VisualizationSnapshotWorker {
 public:
  using Provider = std::function<Result<VisualizationSnapshot>(const AgentId&)>;

  explicit VisualizationSnapshotWorker(Provider provider);
  ~VisualizationSnapshotWorker();
  VisualizationSnapshotWorker(const VisualizationSnapshotWorker&) = delete;
  VisualizationSnapshotWorker& operator=(const VisualizationSnapshotWorker&) =
      delete;

  void Request(AgentId agent);
  [[nodiscard]] std::vector<VisualizationSnapshotResult> Drain();
  void Stop();
  void Join();

 private:
  void Run();

  Provider provider_;
  std::mutex mutex_;
  std::condition_variable ready_;
  std::map<AgentId, uint64_t> pending_;
  std::vector<VisualizationSnapshotResult> completed_;
  uint64_t next_generation_ = 1;
  bool stopping_ = false;
  std::thread thread_;
};

}  // namespace open_lmm
