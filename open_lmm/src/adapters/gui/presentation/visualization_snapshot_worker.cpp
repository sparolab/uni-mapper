#include <adapters/gui/presentation/visualization_snapshot_worker.hpp>

#include <algorithm>
#include <utility>

namespace open_lmm {

VisualizationSnapshotWorker::VisualizationSnapshotWorker(
    Provider provider)
    : provider_(std::move(provider)),
      thread_([this] { Run(); }) {}

VisualizationSnapshotWorker::~VisualizationSnapshotWorker() {
  Stop();
  Join();
}

std::optional<uint64_t> VisualizationSnapshotWorker::Request(
    VisualizationQuery query) {
  if (!query.agent.IsValid()) return std::nullopt;
  uint64_t generation = 0;
  {
    std::lock_guard lock(mutex_);
    if (stopping_) return std::nullopt;
    // Preserve the key before moving query into the pending value. Using
    // query.agent on the assignment LHS and std::move(query) on its RHS lets
    // assignment evaluation move the AgentId before the map lookup, causing
    // different agents to collapse onto the same moved-from key.
    const AgentId agent = query.agent;
    if (active_agent_ && *active_agent_ == agent && active_cancellation_) {
      active_cancellation_->Request();
    }
    const auto pending = pending_.find(agent);
    if (pending != pending_.end()) {
      // A metadata refresh must never downgrade an already queued point copy.
      // Stage-completion refreshes can otherwise replace the full request for
      // an earlier agent, leaving that map absent until the user toggles it.
      query.include_points =
          query.include_points || pending->second.query.include_points;
    }
    generation = next_generation_++;
    pending_[agent] = {std::move(query), generation};
  }
  ready_.notify_one();
  return generation;
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
    if (active_cancellation_) active_cancellation_->Request();
  }
  ready_.notify_all();
}

void VisualizationSnapshotWorker::Join() {
  if (thread_.joinable()) thread_.join();
}

void VisualizationSnapshotWorker::Run() {
  while (true) {
    VisualizationQuery query;
    uint64_t generation = 0;
    std::shared_ptr<CancellationToken> cancellation;
    {
      std::unique_lock lock(mutex_);
      ready_.wait(lock, [this] { return stopping_ || !pending_.empty(); });
      if (stopping_) return;
      const auto request = std::max_element(
          pending_.begin(), pending_.end(), [](const auto& left, const auto& right) {
            return left.second.generation < right.second.generation;
          });
      query = request->second.query;
      generation = request->second.generation;
      active_agent_ = query.agent;
      cancellation = std::make_shared<CancellationToken>();
      active_cancellation_ = cancellation;
      pending_.erase(request);
    }
    Result<VisualizationSnapshot> result =
        Result<VisualizationSnapshot>::Failure(Error::InvalidArgument(
            "visualization snapshot provider is missing"));
    {
      CancellationContextScope cancellation_scope(cancellation);
      if (provider_) result = provider_(query);
    }
    {
      std::lock_guard lock(mutex_);
      if (active_cancellation_ == cancellation) {
        active_agent_.reset();
        active_cancellation_.reset();
      }
      if (!stopping_ && !cancellation->IsCancellationRequested()) {
        completed_.push_back(
            {std::move(query), generation, std::move(result)});
      }
    }
  }
}

}  // namespace open_lmm
