#include <open_lmm/gui/visualization_snapshot_worker.hpp>

#include <utility>

namespace open_lmm {

VisualizationSnapshotWorker::VisualizationSnapshotWorker(
    Provider provider, std::size_t max_points)
    : provider_(std::move(provider)),
      max_points_(max_points),
      thread_([this] { Run(); }) {}

VisualizationSnapshotWorker::~VisualizationSnapshotWorker() {
  Stop();
  Join();
}

void VisualizationSnapshotWorker::Request(char agent) {
  if (agent == 0) return;
  {
    std::lock_guard lock(mutex_);
    if (stopping_) return;
    pending_[agent] = next_generation_++;
  }
  ready_.notify_one();
}

std::vector<VisualizationSnapshotResult> VisualizationSnapshotWorker::Drain() {
  std::lock_guard lock(mutex_);
  std::vector<VisualizationSnapshotResult> results;
  results.swap(completed_);
  return results;
}

void VisualizationSnapshotWorker::Stop() {
  {
    std::lock_guard lock(mutex_);
    stopping_ = true;
    pending_.clear();
  }
  ready_.notify_all();
}

void VisualizationSnapshotWorker::Join() {
  if (thread_.joinable()) thread_.join();
}

void VisualizationSnapshotWorker::Run() {
  while (true) {
    char agent = 0;
    uint64_t generation = 0;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
      if (stopping_) return;
      const auto request = pending_.begin();
      agent = request->first;
      generation = request->second;
      pending_.erase(request);
    }
    auto result = provider_
                      ? provider_(agent, max_points_)
                      : Result<VisualizationSnapshot>::Failure(
                            Error::InvalidArgument(
                                "visualization snapshot provider is missing"));
    {
      std::lock_guard lock(mutex_);
      if (!stopping_) {
        completed_.push_back(
            {agent, generation, std::move(result)});
      }
    }
  }
}

}  // namespace open_lmm
