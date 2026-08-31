#include <adapters/gui/presentation/visualization_snapshot_worker.hpp>

#include <algorithm>
#include <cmath>
#include <utility>

namespace open_lmm {
namespace {

bool Satisfies(const VisualizationQuery& available,
               const VisualizationQuery& requested) {
  if (available.agent != requested.agent) return false;
  if (!requested.include_points) return true;
  return available.include_points &&
         std::abs(available.preview_voxel_size_m -
                  requested.preview_voxel_size_m) <= 1e-6F;
}

}  // namespace

VisualizationSnapshotWorker::VisualizationSnapshotWorker(
    Provider provider, CompletionNotification completion_notification)
    : provider_(std::move(provider)),
      completion_notification_(std::move(completion_notification)),
      thread_([this] { Run(); }) {}

VisualizationSnapshotWorker::~VisualizationSnapshotWorker() {
  Stop();
  Join();
}

std::optional<uint64_t> VisualizationSnapshotWorker::Request(
    VisualizationQuery query, VisualizationRequestIntent intent) {
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
    if (intent != VisualizationRequestIntent::kSourceChanged) {
      if (active_query_ && Satisfies(*active_query_, query)) {
        return active_generation_;
      }
      const auto pending = pending_.find(agent);
      if (pending != pending_.end() &&
          Satisfies(pending->second.query, query)) {
        return pending->second.generation;
      }
      const auto completed = std::find_if(
          completed_.rbegin(), completed_.rend(),
          [&query](const VisualizationSnapshotResult& result) {
            return Satisfies(result.query, query);
          });
      if (completed != completed_.rend()) {
        return completed->request_generation;
      }
    }
    if (active_query_ && active_query_->agent == agent &&
        active_cancellation_) {
      active_cancellation_->Request();
    }
    const auto pending = pending_.find(agent);
    if (pending != pending_.end()) {
      if (intent == VisualizationRequestIntent::kPoll) {
        return pending->second.generation;
      }
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

void VisualizationSnapshotWorker::ResetEpoch() {
  {
    std::lock_guard lock(mutex_);
    if (stopping_) return;
    pending_.clear();
    completed_.clear();
    if (active_cancellation_) active_cancellation_->Request();
  }
  ready_.notify_all();
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
      const auto request = std::min_element(
          pending_.begin(), pending_.end(), [](const auto& left, const auto& right) {
            return left.second.generation < right.second.generation;
          });
      query = request->second.query;
      generation = request->second.generation;
      active_query_ = query;
      active_generation_ = generation;
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
    bool published = false;
    {
      std::lock_guard lock(mutex_);
      if (active_cancellation_ == cancellation) {
        active_query_.reset();
        active_generation_ = 0;
        active_cancellation_.reset();
      }
      if (!stopping_ && !cancellation->IsCancellationRequested()) {
        completed_.push_back(
            {std::move(query), generation, std::move(result)});
        published = true;
      }
    }
    if (published && completion_notification_) completion_notification_();
  }
}

}  // namespace open_lmm
