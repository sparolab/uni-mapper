#include "runtime_session_client.hpp"

#include <open_lmm/utils/logging.hpp>

#include <map>
#include <mutex>
#include <condition_variable>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace open_lmm {
namespace {
struct SessionSubscriberSlot {
  std::mutex mutex;
  std::condition_variable idle;
  bool active = true;
  std::size_t callbacks_in_flight = 0;
  std::function<void(const SessionExecutionEvent&)> callback;
};

thread_local const SessionSubscriberSlot* active_session_subscriber = nullptr;
thread_local const void* active_session_dispatch_state = nullptr;
}  // namespace

struct RuntimeSessionClient::Impl {
  struct State {
    explicit State(SessionId session) : session_id(std::move(session)) {}
    std::mutex mutex;
    SessionId session_id;
    bool replacing = false;
    std::size_t operations_in_progress = 0;
    uint64_t next_subscriber_id = 1;
    std::map<uint64_t, std::shared_ptr<SessionSubscriberSlot>>
        subscribers;
  };

  struct OperationLease {
    OperationLease() = default;
    OperationLease(std::shared_ptr<State> owner, SessionId captured)
        : state(std::move(owner)), session(std::move(captured)) {}
    OperationLease(OperationLease&& other) noexcept
        : state(std::move(other.state)), session(std::move(other.session)) {}
    OperationLease& operator=(OperationLease&&) = delete;
    OperationLease(const OperationLease&) = delete;
    OperationLease& operator=(const OperationLease&) = delete;
    ~OperationLease() {
      if (!state) return;
      std::lock_guard lock(state->mutex);
      --state->operations_in_progress;
    }
    std::shared_ptr<State> state;
    SessionId session;
  };

  Result<OperationLease> Acquire(
      std::optional<SessionId> expected_session = std::nullopt) const {
    std::lock_guard lock(state->mutex);
    if (state->replacing) {
      return Result<OperationLease>::Failure(
          Error::InvalidArgument("session replacement is in progress"));
    }
    if (expected_session && *expected_session != state->session_id) {
      return Result<OperationLease>::Failure(
          Error::InvalidArgument("operation belongs to a retired session"));
    }
    ++state->operations_in_progress;
    return Result<OperationLease>::Ok(
        OperationLease(state, state->session_id));
  }

  Impl(std::shared_ptr<RuntimeClient> client, SessionId session)
      : runtime(std::move(client)),
        state(std::make_shared<State>(std::move(session))) {}

  static void Dispatch(const std::shared_ptr<State>& state,
                       const SessionExecutionEvent& event) {
    std::vector<std::shared_ptr<SessionSubscriberSlot>> callbacks;
    {
      std::lock_guard lock(state->mutex);
      if (event.session_id != state->session_id) return;
      callbacks.reserve(state->subscribers.size());
      for (const auto& [id, callback] : state->subscribers) {
        (void)id;
        callbacks.push_back(callback);
      }
    }
    for (const auto& subscriber : callbacks) {
      std::function<void(const SessionExecutionEvent&)> callback;
      {
        std::lock_guard lock(subscriber->mutex);
        if (!subscriber->active) continue;
        ++subscriber->callbacks_in_flight;
        callback = subscriber->callback;
      }
      const auto* previous_subscriber = active_session_subscriber;
      const void* previous_state = active_session_dispatch_state;
      active_session_subscriber = subscriber.get();
      active_session_dispatch_state = state.get();
      try {
        callback(event);
      } catch (const std::exception& error) {
        LogWarning(std::string("runtime adapter event observer failed: ") +
                   error.what());
      } catch (...) {
        LogWarning(
            "runtime adapter event observer failed with unknown exception");
      }
      active_session_subscriber = previous_subscriber;
      active_session_dispatch_state = previous_state;
      {
        std::lock_guard lock(subscriber->mutex);
        --subscriber->callbacks_in_flight;
        if (subscriber->callbacks_in_flight == 0) subscriber->idle.notify_all();
      }
    }
  }

  std::shared_ptr<RuntimeClient> runtime;
  std::shared_ptr<State> state;
  ExecutionEventSubscription runtime_subscription;
};

RuntimeSessionClient::RuntimeSessionClient(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
RuntimeSessionClient::~RuntimeSessionClient() = default;

Result<std::shared_ptr<RuntimeSessionClient>> RuntimeSessionClient::Create(
    std::shared_ptr<RuntimeClient> runtime, SessionId session_id) {
  if (!runtime) {
    return Result<std::shared_ptr<RuntimeSessionClient>>::Failure(
        Error::InvalidArgument("runtime client must not be null"));
  }
  auto client = std::shared_ptr<RuntimeSessionClient>(
      new RuntimeSessionClient(std::make_unique<Impl>(
          std::move(runtime), std::move(session_id))));
  auto feedback_enabled = client->impl_->runtime->SetAlignmentFeedbackEnabled(
      client->impl_->state->session_id, true);
  if (!feedback_enabled) {
    return Result<std::shared_ptr<RuntimeSessionClient>>::Failure(
        feedback_enabled.GetError());
  }
  std::weak_ptr<Impl::State> weak = client->impl_->state;
  auto subscribed = client->impl_->runtime->SubscribeEvents(
      client->impl_->state->session_id,
      [weak](const SessionExecutionEvent& event) {
        if (auto locked = weak.lock()) Impl::Dispatch(locked, event);
      });
  if (!subscribed) {
    return Result<std::shared_ptr<RuntimeSessionClient>>::Failure(
        subscribed.GetError());
  }
  client->impl_->runtime_subscription = std::move(subscribed).Value();
  return Result<std::shared_ptr<RuntimeSessionClient>>::Ok(std::move(client));
}

SessionId RuntimeSessionClient::CurrentSession() const {
  std::lock_guard lock(impl_->state->mutex);
  return impl_->state->session_id;
}

Result<void> RuntimeSessionClient::ReplaceSession(
    const BootstrapRequest& request, const ConfigCandidate& root_candidate) {
  {
    std::lock_guard lock(impl_->state->mutex);
    if (active_session_dispatch_state == impl_->state.get()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "cannot replace a session from its event callback"));
    }
    if (impl_->state->replacing) {
      return Result<void>::Failure(
          Error::InvalidArgument("session replacement is already in progress"));
    }
    if (impl_->state->operations_in_progress != 0) {
      return Result<void>::Failure(Error::InvalidArgument(
          "cannot replace a session while an operation is in progress"));
    }
    impl_->state->replacing = true;
  }
  struct ReplacementGuard {
    std::shared_ptr<Impl::State> state;
    ~ReplacementGuard() {
      std::lock_guard lock(state->mutex);
      state->replacing = false;
    }
  } replacement_guard{impl_->state};
  const SessionId previous_session = CurrentSession();
  std::weak_ptr<Impl::State> weak = impl_->state;
  auto created = impl_->runtime->ReplaceSession(
      previous_session, request, root_candidate,
      [weak](const SessionExecutionEvent& event) {
        if (auto locked = weak.lock()) Impl::Dispatch(locked, event);
      });
  if (!created) return Result<void>::Failure(created.GetError());
  auto replacement_result = std::move(created).Value();
  const SessionId replacement = replacement_result.session_id;

  SessionId previous = replacement;
  ExecutionEventSubscription previous_subscription;
  {
    std::lock_guard lock(impl_->state->mutex);
    previous = impl_->state->session_id;
    impl_->state->session_id = replacement;
    previous_subscription = std::move(impl_->runtime_subscription);
    impl_->runtime_subscription =
        std::move(replacement_result.event_subscription);
  }
  previous_subscription.Reset();
  (void)previous;
  return Result<void>::Ok();
}

Result<BoundJob> RuntimeSessionClient::Submit(
    const ExecutionRequest& request) {
  auto lease = impl_->Acquire();
  if (!lease) return Result<BoundJob>::Failure(lease.GetError());
  auto submitted = impl_->runtime->Submit(lease.Value().session, request);
  if (!submitted) return Result<BoundJob>::Failure(submitted.GetError());
  return Result<BoundJob>::Ok({lease.Value().session, submitted.Value()});
}

Result<void> RuntimeSessionClient::Cancel(const BoundJob& job) {
  auto lease = impl_->Acquire(job.session_id);
  return lease ? impl_->runtime->Cancel(lease.Value().session, job.job_id)
               : Result<void>::Failure(lease.GetError());
}

Result<void> RuntimeSessionClient::CancelCurrent(JobId job_id) {
  auto lease = impl_->Acquire();
  return lease ? impl_->runtime->Cancel(lease.Value().session, job_id)
               : Result<void>::Failure(lease.GetError());
}

Result<void> RuntimeSessionClient::Wait(const BoundJob& job) {
  auto lease = impl_->Acquire(job.session_id);
  return lease ? impl_->runtime->Wait(lease.Value().session, job.job_id)
               : Result<void>::Failure(lease.GetError());
}

Result<RuntimeSessionSnapshot> RuntimeSessionClient::Snapshot() const {
  auto lease = impl_->Acquire();
  return lease ? impl_->runtime->RuntimeSnapshot(lease.Value().session)
               : Result<RuntimeSessionSnapshot>::Failure(lease.GetError());
}

Result<std::vector<NodeDescriptor>> RuntimeSessionClient::NodeDescriptors()
    const {
  auto lease = impl_->Acquire();
  return lease ? impl_->runtime->NodeDescriptors(lease.Value().session)
               : Result<std::vector<NodeDescriptor>>::Failure(lease.GetError());
}

Result<VisualizationSnapshot> RuntimeSessionClient::VisualizationSnapshotFor(
    const AgentId& agent) const {
  auto lease = impl_->Acquire();
  return lease ? impl_->runtime->VisualizationSnapshot(lease.Value().session,
                                                        agent)
               : Result<VisualizationSnapshot>::Failure(lease.GetError());
}

Result<std::optional<AlignmentFeedbackSnapshot>>
RuntimeSessionClient::AlignmentFeedbackSnapshotFor() const {
  auto lease = impl_->Acquire();
  return lease ? impl_->runtime->AlignmentFeedbackSnapshot(lease.Value().session)
               : Result<std::optional<AlignmentFeedbackSnapshot>>::Failure(
                     lease.GetError());
}

Result<void> RuntimeSessionClient::RespondToAlignment(
    const BoundJob& job, AlignmentResponse response) {
  auto lease = impl_->Acquire(job.session_id);
  return lease ? impl_->runtime->RespondToAlignment(
                     lease.Value().session, job.job_id, std::move(response))
               : Result<void>::Failure(lease.GetError());
}

Result<ConfigApplyReceipt> RuntimeSessionClient::ApplyConfig(
    const SessionId& expected_session, const ConfigCandidate& candidate,
    const ExpectedRevision& expected) {
  auto lease = impl_->Acquire(expected_session);
  return lease ? impl_->runtime->ApplyConfig(lease.Value().session, candidate,
                                             expected)
               : Result<ConfigApplyReceipt>::Failure(lease.GetError());
}

Result<ExecutionEventSubscription> RuntimeSessionClient::Subscribe(
    std::function<void(const SessionExecutionEvent&)> callback) {
  if (!callback) {
    return Result<ExecutionEventSubscription>::Failure(
        Error::InvalidArgument("runtime event callback must not be empty"));
  }
  uint64_t id = 0;
  {
    std::lock_guard lock(impl_->state->mutex);
    id = impl_->state->next_subscriber_id++;
    auto subscriber = std::make_shared<SessionSubscriberSlot>();
    subscriber->callback = std::move(callback);
    impl_->state->subscribers.emplace(id, std::move(subscriber));
  }
  std::weak_ptr<Impl::State> weak = impl_->state;
  return Result<ExecutionEventSubscription>::Ok(ExecutionEventSubscription(
      [weak, id] {
        if (auto state = weak.lock()) {
          std::shared_ptr<SessionSubscriberSlot> subscriber;
          {
            std::lock_guard lock(state->mutex);
            const auto found = state->subscribers.find(id);
            if (found == state->subscribers.end()) return;
            subscriber = found->second;
            state->subscribers.erase(found);
          }
          std::unique_lock lock(subscriber->mutex);
          subscriber->active = false;
          if (active_session_subscriber != subscriber.get()) {
            subscriber->idle.wait(lock, [&] {
              return subscriber->callbacks_in_flight == 0;
            });
          }
        }
      }));
}

}  // namespace open_lmm
