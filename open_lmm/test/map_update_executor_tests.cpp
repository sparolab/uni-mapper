#include <runtime/execution/stages/map_update_executor.hpp>

#include <domain/data_loader/data_loader_base.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <domain/optimization/backend_optimizer_base.hpp>
#include <plugins/host/algorithm_factory.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <thread>
#include <vector>

#include <unistd.h>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

fs::path TempRoot(const char* label) {
  static std::atomic<uint64_t> nonce{0};
  return fs::temp_directory_path() /
         (std::string("open_lmm_map_") + label + "_" +
          std::to_string(static_cast<unsigned long long>(::getpid())) + "_" +
          std::to_string(nonce.fetch_add(1, std::memory_order_relaxed)));
}

template <typename Predicate>
void WaitUntil(Predicate predicate, const char* message) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  Check(predicate(), message);
}

class FixtureOptimizer final : public BackendOptimizerBase {
 public:
  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext&,
      const BackendOptimizerInput&) override {
    return Result<BackendOptimizerOutput>::Ok({});
  }
  void Reset() override {}
  bool HasProcessedAgent(const AgentId&) const override { return false; }
  std::size_t ProcessedAgentCount() const override { return 0; }
};

enum class RemoverBehavior {
  kSuccess,
  kFailure,
  kEmpty,
  kDrainFirstFailure,
  kWaitForCancellation,
};

struct OperationProbe {
  std::shared_ptr<ResourceGovernor> governor;
  RemoverBehavior behavior = RemoverBehavior::kSuccess;
  std::size_t expected_parallel_entries = 1;
  bool fail_loader_factory = false;
  bool fail_remover_factory = false;

  std::atomic<unsigned> loader_factory_calls{0};
  std::atomic<unsigned> remover_factory_calls{0};
  std::atomic<unsigned> remover_entries{0};
  std::atomic<unsigned> reservations_observed{0};
  std::atomic<uint64_t> observed_reservation_bytes{0};
  std::atomic<unsigned> heavy_callbacks_observed{0};
  std::atomic<unsigned> heavy_active{0};
  std::atomic<unsigned> max_heavy_active{0};
  std::atomic<unsigned> operation_active{0};
  std::atomic<unsigned> max_operation_active{0};
  std::atomic<bool> first_failed{false};
  std::atomic<bool> second_blocked{false};
  std::atomic<bool> release_second{false};
  std::atomic<bool> second_completed{false};
  std::atomic<bool> cancellation_blocked{false};
};

class OperationActivity {
 public:
  explicit OperationActivity(OperationProbe& probe) : probe_(probe) {
    const unsigned active =
        probe_.operation_active.fetch_add(1, std::memory_order_acq_rel) + 1;
    unsigned previous =
        probe_.max_operation_active.load(std::memory_order_acquire);
    while (previous < active &&
           !probe_.max_operation_active.compare_exchange_weak(
               previous, active, std::memory_order_acq_rel)) {
    }
  }
  ~OperationActivity() {
    probe_.operation_active.fetch_sub(1, std::memory_order_acq_rel);
  }

 private:
  OperationProbe& probe_;
};

class FixtureLoader final : public DataLoaderBase {
 public:
  Result<AgentRawData> Process(const AlgorithmExecutionContext&,
                               const DataLoaderInput&) override {
    return Result<AgentRawData>::Failure(
        Error::InvalidArgument("MapUpdate fixture does not call Process"));
  }

  Result<std::size_t> VisitRawScanData(
      const AlgorithmExecutionContext&, const fs::path&,
      const RawScanVisitor& visitor, AlgorithmProgressPhase) override {
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->push_back({1.0F, 2.0F, 3.0F, 0.5F});
    auto visited = visitor(0, scan);
    return visited ? Result<std::size_t>::Ok(1)
                   : Result<std::size_t>::Failure(visited.GetError());
  }
};

class FixtureRemover final : public DynamicRemoverBase {
 public:
  explicit FixtureRemover(std::shared_ptr<OperationProbe> probe)
      : probe_(std::move(probe)) {}

  Result<PointCloud::Ptr> Process(const AlgorithmExecutionContext&,
                                  DynamicRemoverInput) override {
    return Result<PointCloud::Ptr>::Failure(
        Error::InvalidArgument("MapUpdate fixture uses streaming"));
  }

  Result<PointCloud::Ptr> ProcessStreaming(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input) override {
    OperationActivity operation(*probe_);
    const uint64_t reserved = probe_->governor->ReservedMemoryBytes(
        MemoryClass::kHeavyMap);
    if (reserved != 0) {
      probe_->reservations_observed.fetch_add(1, std::memory_order_relaxed);
      probe_->observed_reservation_bytes.store(reserved,
                                               std::memory_order_release);
    }
    probe_->remover_entries.fetch_add(1, std::memory_order_acq_rel);

    if (probe_->behavior == RemoverBehavior::kDrainFirstFailure) {
      while (probe_->remover_entries.load(std::memory_order_acquire) < 2) {
        std::this_thread::yield();
      }
      if (context.agent.id == Id("A")) {
        probe_->first_failed.store(true, std::memory_order_release);
        return Result<PointCloud::Ptr>::Failure(
            Error::InvalidArgument("first agent failure"));
      }
      probe_->second_blocked.store(true, std::memory_order_release);
      while (!probe_->release_second.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      probe_->second_completed.store(true, std::memory_order_release);
    }

    auto loaded = input.source([](std::size_t,
                                  const PointCloud::Ptr&) {
      return Result<void>::Ok();
    });
    if (!loaded) return Result<PointCloud::Ptr>::Failure(loaded.GetError());

    if (probe_->behavior == RemoverBehavior::kWaitForCancellation) {
      Check(static_cast<bool>(input.heavy_phase_admission),
            "mid-operation cancellation fixture receives heavy admission");
      probe_->heavy_callbacks_observed.fetch_add(1,
                                                 std::memory_order_relaxed);
      auto phase = input.heavy_phase_admission();
      if (!phase) return Result<PointCloud::Ptr>::Failure(phase.GetError());
      probe_->cancellation_blocked.store(true, std::memory_order_release);
      while (!context.cancellation->IsCancellationRequested()) {
        std::this_thread::yield();
      }
      return Result<PointCloud::Ptr>::Failure(
          Error::Cancelled("fixture mid-operation cancellation"));
    }

    if (probe_->behavior != RemoverBehavior::kDrainFirstFailure) {
      while (probe_->remover_entries.load(std::memory_order_acquire) <
             probe_->expected_parallel_entries) {
        std::this_thread::yield();
      }
    }
    if (input.heavy_phase_admission) {
      probe_->heavy_callbacks_observed.fetch_add(1,
                                                 std::memory_order_relaxed);
      auto phase = input.heavy_phase_admission();
      if (!phase) {
        return Result<PointCloud::Ptr>::Failure(phase.GetError());
      }
      const unsigned active =
          probe_->heavy_active.fetch_add(1, std::memory_order_acq_rel) + 1;
      unsigned previous =
          probe_->max_heavy_active.load(std::memory_order_acquire);
      while (previous < active &&
             !probe_->max_heavy_active.compare_exchange_weak(
                 previous, active, std::memory_order_acq_rel)) {
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
      probe_->heavy_active.fetch_sub(1, std::memory_order_acq_rel);
    }

    if (probe_->behavior == RemoverBehavior::kFailure) {
      return Result<PointCloud::Ptr>::Failure(
          Error::InvalidArgument("fixture remover failure"));
    }
    auto map = std::make_shared<PointCloud>();
    if (probe_->behavior != RemoverBehavior::kEmpty) {
      map->push_back({1.0F, 2.0F, 3.0F, 0.5F});
      map->push_back({2.0F, 3.0F, 4.0F, 0.6F});
    }
    return Result<PointCloud::Ptr>::Ok(std::move(map));
  }

 private:
  std::shared_ptr<OperationProbe> probe_;
};

class FixtureFactory final : public AlgorithmFactory {
 public:
  explicit FixtureFactory(std::shared_ptr<OperationProbe> probe)
      : probe_(std::move(probe)) {}

 protected:
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoaderImpl(
      const DataLoaderConfig&) const override {
    probe_->loader_factory_calls.fetch_add(1, std::memory_order_relaxed);
    const auto reserved =
        probe_->governor->ReservedMemoryBytes(MemoryClass::kHeavyMap);
    if (reserved != 0) {
      probe_->reservations_observed.fetch_add(1, std::memory_order_relaxed);
      probe_->observed_reservation_bytes.store(reserved,
                                               std::memory_order_release);
    }
    if (probe_->fail_loader_factory) {
      return Result<std::unique_ptr<DataLoaderBase>>::Failure(
          Error::InvalidArgument("fixture loader factory failure"));
    }
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(
        std::make_unique<FixtureLoader>());
  }

  Result<std::shared_ptr<DynamicRemoverBase>> CreateDynamicRemoverImpl(
      const DynamicRemoverConfig&) const override {
    probe_->remover_factory_calls.fetch_add(1, std::memory_order_relaxed);
    const auto reserved =
        probe_->governor->ReservedMemoryBytes(MemoryClass::kHeavyMap);
    if (reserved != 0) {
      probe_->reservations_observed.fetch_add(1, std::memory_order_relaxed);
      probe_->observed_reservation_bytes.store(reserved,
                                               std::memory_order_release);
    }
    if (probe_->fail_remover_factory) {
      return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
          Error::InvalidArgument("fixture remover factory failure"));
    }
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<FixtureRemover>(probe_));
  }

 private:
  std::shared_ptr<OperationProbe> probe_;
};

std::shared_ptr<const RuntimeState> MakeState(
    const fs::path& data_root, const std::vector<AgentId>& agents) {
  auto config = std::make_shared<RuntimeConfig>();
  config->data_loader = std::make_shared<const DataLoaderConfig>();
  auto remover = std::make_shared<DynamicRemoverConfig>();
  remover->type = "offline";
  remover->model = "fixture";
  remover->thread_safety = PluginThreadSafety::kInstanceIsolatedParallel;
  remover->internal_cpu_threads = 1;
  config->dynamic_remover = std::move(remover);
  auto documents = std::make_shared<RuntimeConfigDocuments>();
  documents->data_loader.canonical_json = "{}";
  documents->dynamic_remover.canonical_json = "{}";
  config->documents = std::move(documents);

  auto database = std::make_shared<SharedDatabase>();
  auto payload = std::make_shared<RuntimePayload>();
  payload->optimizer = std::make_shared<FixtureOptimizer>();
  for (std::size_t index = 0; index < agents.size(); ++index) {
    const auto& agent = agents[index];
    const auto directory = data_root / agent.Value();
    fs::create_directories(directory);
    auto raw = std::make_shared<AgentRawData>();
    raw->agent_id = agent;
    raw->odom_poses.push_back(Eigen::Isometry3d::Identity());
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->reserve(8);
    scan->push_back({1.0F, 2.0F, 3.0F, 0.5F});
    raw->filtered_scans.push_back(scan);
    database->raw_data.emplace(agent, raw);
    auto optimized = std::make_shared<AgentOptimizedData>();
    optimized->agent_id = agent;
    optimized->optimized_poses.emplace_back(
        0, Eigen::Isometry3d::Identity());
    database->optimized_data.emplace(agent, std::move(optimized));
    AgentPipelineCtx context;
    context.agent = {.id = agent,
                     .role = index == 0 ? AgentRole::kAnchor
                                        : AgentRole::kFollower,
                     .order = static_cast<int>(index)};
    context.data_dir = directory;
    context.raw_data = std::move(raw);
    payload->contexts.push_back(std::move(context));
  }
  payload->database = std::move(database);

  auto state = std::make_shared<RuntimeState>();
  state->revision = 7;
  state->config = std::move(config);
  state->ordered_agents = agents;
  state->payload = std::move(payload);
  return state;
}

MapUpdateExecutionContext Context(
    const std::shared_ptr<const RuntimeState>& state,
    const std::shared_ptr<ResourceGovernor>& governor,
    const std::shared_ptr<CancellationToken>& cancellation,
    const std::shared_ptr<const AlgorithmProvider>& algorithms,
    const fs::path& output, bool parallel, std::size_t max_parallel,
    PendingOutputSet& pending, double save_voxel_size = 0.2) {
  return {.committed = state,
          .governor = governor,
          .cancellation = cancellation,
          .progress = {},
          .algorithms = algorithms,
          .output_directory = output,
          .save_voxel_size = save_voxel_size,
          .parallel = parallel,
          .max_parallel_agents = max_parallel,
          .pending_files = &pending};
}

void TestBothModesShareAdmissionAndIgnoreUnrelatedDirectoryBytes() {
  uint64_t sequential_estimate = 0;
  for (const bool parallel : {false, true}) {
    const auto root =
        TempRoot(parallel ? "admission_parallel" : "admission_sequential");
    fs::remove_all(root);
    auto state = MakeState(root / "data", {Id("A")});
    {
      std::ofstream unrelated(root / "data/A/unrelated.bin");
      unrelated << 'x';
    }
    fs::resize_file(root / "data/A/unrelated.bin", 1ULL << 30);
    fs::create_directories(root / "output");
    auto governor = std::make_shared<ResourceGovernor>(
        ResourceBudget{2, 2, 4096});
    auto probe = std::make_shared<OperationProbe>();
    probe->governor = governor;
    auto factory = std::make_shared<FixtureFactory>(probe);
    auto cancellation = std::make_shared<CancellationToken>();
    PendingOutputSet pending;
    MapUpdateExecutor executor;
    auto result = executor.Execute(Context(
        state, governor, cancellation, factory, root / "output", parallel, 2,
        pending));
    const uint64_t observed =
        probe->observed_reservation_bytes.load(std::memory_order_acquire);
    if (!parallel) sequential_estimate = observed;
    Check(result.IsOk(), "admitted MapUpdate succeeds in both modes");
    if (!(observed > 1 && (!parallel || observed == sequential_estimate))) {
      std::cerr << "admission estimate mode="
                << (parallel ? "parallel" : "sequential")
                << " observed=" << observed
                << " sequential=" << sequential_estimate << '\n';
    }
    Check(observed > 1 && (!parallel || observed == sequential_estimate),
          "both modes reserve the same committed-payload estimate");
    Check(probe->loader_factory_calls.load() == 1 &&
              probe->remover_factory_calls.load() == 1 &&
              probe->reservations_observed.load() >= 3,
          "admission is held across both algorithm factories and execution");
    Check(probe->heavy_callbacks_observed.load() == 1 &&
              probe->max_heavy_active.load() == 1,
          "both modes pass the heavy-phase admission callback");
    Check(governor->ReservedMemoryBytes(MemoryClass::kHeavyMap) == 0,
          "successful MapUpdate releases transient admission");
    Check(result.Value().execution_agents ==
              std::vector<AgentId>{Id("A")} &&
              pending.Files().size() == 1 &&
              fs::is_regular_file(pending.Files().front().first),
          "successful MapUpdate preserves output and agent ordering");
    pending.Rollback();
    fs::remove_all(root);
  }
}

void TestBothModesRejectBeforeFactoryAndPublishNothing() {
  for (const bool parallel : {false, true}) {
    const auto root =
        TempRoot(parallel ? "reject_parallel" : "reject_sequential");
    fs::remove_all(root);
    auto state = MakeState(root / "data", {Id("A")});
    fs::create_directories(root / "output");
    auto governor = std::make_shared<ResourceGovernor>(
        ResourceBudget{2, 2, 1});
    auto probe = std::make_shared<OperationProbe>();
    probe->governor = governor;
    auto factory = std::make_shared<FixtureFactory>(probe);
    PendingOutputSet pending;
    MapUpdateExecutor executor;
    auto result = executor.Execute(Context(
        state, governor, std::make_shared<CancellationToken>(), factory,
        root / "output", parallel, 2, pending));
    Check(!result &&
              result.GetError().Message().find(
                  "memory admission rejected: class=heavy_map") !=
                  std::string::npos &&
              probe->loader_factory_calls.load() == 0 &&
              governor->ReservedMemoryBytes() == 0 &&
              pending.Files().size() == 1 &&
              !fs::exists(pending.Files().front().first) &&
              !fs::exists(pending.Files().front().second),
          "admission rejection precedes factories and file publication in both modes");
    fs::remove_all(root);
  }
}

void TestFailuresAndCancellationReleaseAdmission() {
  const auto run = [](const char* suffix, bool parallel,
                      RemoverBehavior behavior, bool fail_loader,
                      bool fail_remover, bool cancel_before,
                      bool invalid_output, double save_voxel_size) {
    const auto root = TempRoot(suffix);
    fs::remove_all(root);
    auto state = MakeState(root / "data", {Id("A")});
    const fs::path output = root / "output";
    if (invalid_output) {
      fs::create_directories(root);
      std::ofstream(output) << "not a directory\n";
    } else {
      fs::create_directories(output);
    }
    auto governor = std::make_shared<ResourceGovernor>(
        ResourceBudget{2, 2, 4096});
    auto probe = std::make_shared<OperationProbe>();
    probe->governor = governor;
    probe->behavior = behavior;
    probe->fail_loader_factory = fail_loader;
    probe->fail_remover_factory = fail_remover;
    auto factory = std::make_shared<FixtureFactory>(probe);
    auto cancellation = std::make_shared<CancellationToken>();
    if (cancel_before) cancellation->Request();
    PendingOutputSet pending;
    MapUpdateExecutor executor;
    auto result = executor.Execute(Context(
        state, governor, cancellation, factory, output, parallel, 2, pending,
        save_voxel_size));
    Check(!result && governor->ReservedMemoryBytes() == 0,
          "every MapUpdate failure releases its admission reservation");
    for (const auto& file : pending.Files()) {
      Check(!fs::exists(file.first) && !fs::exists(file.second),
            "failed MapUpdate does not publish temporary or final files");
    }
    auto reacquired = governor->AcquireHeavyMemoryPhase({});
    Check(reacquired.IsOk(),
          "failed MapUpdate leaves the heavy phase reusable");
    governor->ReleaseHeavyMemoryPhase();
    fs::remove_all(root);
  };

  run("loader_factory", false, RemoverBehavior::kSuccess, true, false, false,
      false, 0.2);
  run("remover_factory", true, RemoverBehavior::kSuccess, false, true, false,
      false, 0.2);
  run("remover", true, RemoverBehavior::kFailure, false, false, false, false,
      0.2);
  run("empty", false, RemoverBehavior::kEmpty, false, false, false, false,
      0.2);
  run("cancel_before", true, RemoverBehavior::kSuccess, false, false, true,
      false, 0.2);
  run("downsample", false, RemoverBehavior::kSuccess, false, false, false,
      false, 0.0);
  run("write", false, RemoverBehavior::kSuccess, false, false, false, true,
      0.2);
}

void TestMidOperationCancellationReleasesAdmissionAndHeavyPhase() {
  const auto root = TempRoot("mid_operation_cancel");
  fs::remove_all(root);
  auto state = MakeState(root / "data", {Id("A")});
  fs::create_directories(root / "output");
  auto governor = std::make_shared<ResourceGovernor>(
      ResourceBudget{2, 2, 4096});
  auto probe = std::make_shared<OperationProbe>();
  probe->governor = governor;
  probe->behavior = RemoverBehavior::kWaitForCancellation;
  auto factory = std::make_shared<FixtureFactory>(probe);
  auto cancellation = std::make_shared<CancellationToken>();
  PendingOutputSet pending;
  MapUpdateExecutor executor;
  Result<ExecutionCandidate> result = Result<ExecutionCandidate>::Failure(
      Error::InvalidArgument("not run"));
  std::thread execution([&] {
    result = executor.Execute(Context(state, governor, cancellation, factory,
                                      root / "output", false, 1, pending));
  });
  WaitUntil(
      [&] {
        return probe->cancellation_blocked.load(std::memory_order_acquire);
      },
      "MapUpdate reaches cancellation while reservation and phase are held");
  Check(governor->ReservedMemoryBytes(MemoryClass::kHeavyMap) > 0,
        "mid-operation cancellation observes admitted heavy memory");
  cancellation->Request();
  execution.join();
  Check(!result && result.GetError().code == Error::Code::kCancelled &&
            governor->ReservedMemoryBytes() == 0,
        "mid-operation cancellation releases its memory admission");
  for (const auto& file : pending.Files()) {
    Check(!fs::exists(file.first) && !fs::exists(file.second),
          "mid-operation cancellation publishes no map file");
  }
  auto reacquired = governor->AcquireHeavyMemoryPhase({});
  Check(reacquired.IsOk(),
        "mid-operation cancellation releases the heavy phase gate");
  governor->ReleaseHeavyMemoryPhase();
  pending.Rollback();

  probe->behavior = RemoverBehavior::kSuccess;
  probe->expected_parallel_entries = 1;
  PendingOutputSet retry_pending;
  auto retry = executor.Execute(Context(
      state, governor, std::make_shared<CancellationToken>(), factory,
      root / "output", false, 1, retry_pending));
  Check(retry.IsOk() && governor->ReservedMemoryBytes() == 0,
        "same governor executes successfully after cancelled admission");
  retry_pending.Rollback();
  fs::remove_all(root);
}

void TestParallelFailureDrainsEverySubmittedTask() {
  const auto root = TempRoot("parallel_drain");
  fs::remove_all(root);
  auto state = MakeState(root / "data", {Id("A"), Id("B")});
  fs::create_directories(root / "output");
  auto governor = std::make_shared<ResourceGovernor>(
      ResourceBudget{2, 2, 8192});
  auto probe = std::make_shared<OperationProbe>();
  probe->governor = governor;
  probe->behavior = RemoverBehavior::kDrainFirstFailure;
  probe->expected_parallel_entries = 2;
  auto factory = std::make_shared<FixtureFactory>(probe);
  auto cancellation = std::make_shared<CancellationToken>();
  PendingOutputSet pending;
  MapUpdateExecutor executor;
  Result<ExecutionCandidate> result =
      Result<ExecutionCandidate>::Failure(Error::InvalidArgument("not run"));
  std::atomic<bool> returned{false};
  std::thread execution([&] {
    result = executor.Execute(Context(state, governor, cancellation, factory,
                                      root / "output", true, 2, pending));
    returned.store(true, std::memory_order_release);
  });
  WaitUntil([&] { return probe->second_blocked.load(std::memory_order_acquire); },
            "second MapUpdate task enters its deterministic block");
  WaitUntil([&] { return probe->first_failed.load(std::memory_order_acquire); },
            "first MapUpdate task reaches its deterministic failure");
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Check(!returned.load(std::memory_order_acquire),
        "first failed task cannot return while a sibling is still running");
  probe->release_second.store(true, std::memory_order_release);
  execution.join();
  governor->AgentExecutor().WaitIdle();
  const auto executor_state = governor->AgentExecutor().Snapshot();
  Check(!result &&
            result.GetError().Message().find("first agent failure") !=
                std::string::npos &&
            probe->second_completed.load(std::memory_order_acquire) &&
            governor->ReservedMemoryBytes() == 0 &&
            executor_state.active_tasks == 0 &&
            executor_state.queued_tasks == 0,
        "parallel MapUpdate drains siblings and preserves the first error");
  fs::remove_all(root);
}

void TestParallelHeavyPhaseIsSerialized() {
  const auto root = TempRoot("heavy_serialization");
  fs::remove_all(root);
  auto state = MakeState(root / "data", {Id("A"), Id("B")});
  fs::create_directories(root / "output");
  auto governor = std::make_shared<ResourceGovernor>(
      ResourceBudget{2, 2, 8192});
  auto probe = std::make_shared<OperationProbe>();
  probe->governor = governor;
  probe->expected_parallel_entries = 2;
  auto factory = std::make_shared<FixtureFactory>(probe);
  PendingOutputSet pending;
  MapUpdateExecutor executor;
  auto result = executor.Execute(Context(
      state, governor, std::make_shared<CancellationToken>(), factory,
      root / "output", true, 2, pending));
  Check(result && probe->heavy_callbacks_observed.load() == 2 &&
            probe->max_heavy_active.load() == 1 &&
            governor->ReservedMemoryBytes() == 0,
        "parallel agents share the one-slot heavy-map phase");
  pending.Rollback();
  fs::remove_all(root);
}

void TestParallelAgentConcurrencyIsBounded() {
  const auto root = TempRoot("parallel_bound");
  fs::remove_all(root);
  auto state = MakeState(root / "data", {Id("A"), Id("B"), Id("C")});
  fs::create_directories(root / "output");
  auto governor = std::make_shared<ResourceGovernor>(
      ResourceBudget{2, 2, 16384});
  auto probe = std::make_shared<OperationProbe>();
  probe->governor = governor;
  probe->expected_parallel_entries = 2;
  auto factory = std::make_shared<FixtureFactory>(probe);
  PendingOutputSet pending;
  MapUpdateExecutor executor;
  auto result = executor.Execute(Context(
      state, governor, std::make_shared<CancellationToken>(), factory,
      root / "output", true, 2, pending));
  Check(result && probe->remover_entries.load() == 3 &&
            probe->max_operation_active.load() == 2 &&
            probe->operation_active.load() == 0 &&
            governor->ReservedMemoryBytes() == 0,
        "three-agent MapUpdate obeys the two-agent concurrency bound");
  pending.Rollback();
  fs::remove_all(root);
}

}  // namespace

int main() {
  TestBothModesShareAdmissionAndIgnoreUnrelatedDirectoryBytes();
  TestBothModesRejectBeforeFactoryAndPublishNothing();
  TestFailuresAndCancellationReleaseAdmission();
  TestMidOperationCancellationReleasesAdmissionAndHeavyPhase();
  TestParallelFailureDrainsEverySubmittedTask();
  TestParallelHeavyPhaseIsSerialized();
  TestParallelAgentConcurrencyIsBounded();
  std::cout << "MapUpdate admission tests passed\n";
  return 0;
}
