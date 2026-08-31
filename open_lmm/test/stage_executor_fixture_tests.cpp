#include <runtime/execution/stages/data_load_executor.hpp>
#include <runtime/execution/stages/map_update_executor.hpp>
#include <plugins/host/algorithm_factory.hpp>
#include <domain/data_loader/data_loader_base.hpp>

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

class PreviewLoader final : public open_lmm::DataLoaderBase {
 public:
  open_lmm::Result<open_lmm::AgentRawData> Process(
      const open_lmm::AlgorithmExecutionContext& context,
      const open_lmm::DataLoaderInput& input) override {
    open_lmm::AgentRawData raw;
    raw.agent_id = context.agent.id;
    raw.odom_poses.push_back(Eigen::Isometry3d::Identity());
    auto scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    scan->push_back({1.0F, 2.0F, 3.0F, 0.5F});
    raw.filtered_scans.push_back(scan);
    if (input.observe_filtered_scan) {
      input.observe_filtered_scan(0, raw.odom_poses.front(), *scan);
    }
    return open_lmm::Result<open_lmm::AgentRawData>::Ok(std::move(raw));
  }
  open_lmm::Result<std::size_t> VisitRawScanData(
      const open_lmm::AlgorithmExecutionContext&, const fs::path&,
      const RawScanVisitor&, open_lmm::AlgorithmProgressPhase) override {
    return open_lmm::Result<std::size_t>::Ok(0);
  }
};

class PreviewFactory final : public open_lmm::AlgorithmFactory {
 protected:
  open_lmm::Result<std::unique_ptr<open_lmm::DataLoaderBase>>
  CreateDataLoaderImpl(const open_lmm::DataLoaderConfig&) const override {
    return open_lmm::Result<
        std::unique_ptr<open_lmm::DataLoaderBase>>::Ok(
        std::make_unique<PreviewLoader>());
  }
};

struct AdmissionBarrier {
  std::atomic<int> entered{0};
  std::atomic<int> completed{0};
};

class CoordinatedAdmissionLoader final : public open_lmm::DataLoaderBase {
 public:
  explicit CoordinatedAdmissionLoader(
      std::shared_ptr<AdmissionBarrier> barrier)
      : barrier_(std::move(barrier)) {}

  open_lmm::Result<open_lmm::AgentRawData> Process(
      const open_lmm::AlgorithmExecutionContext& context,
      const open_lmm::DataLoaderInput& input) override {
    ++barrier_->entered;
    while (barrier_->entered.load() != 2) std::this_thread::yield();
    auto admitted = input.admit_resident_bytes
                        ? input.admit_resident_bytes(1024)
                        : open_lmm::Result<void>::Failure(
                              open_lmm::Error::InvalidArgument(
                                  "resident admission callback missing"));
    ++barrier_->completed;
    while (barrier_->completed.load() != 2) std::this_thread::yield();
    if (!admitted) {
      return open_lmm::Result<open_lmm::AgentRawData>::Failure(
          admitted.GetError());
    }
    open_lmm::AgentRawData raw;
    raw.agent_id = context.agent.id;
    raw.odom_poses.push_back(Eigen::Isometry3d::Identity());
    return open_lmm::Result<open_lmm::AgentRawData>::Ok(std::move(raw));
  }

  open_lmm::Result<std::size_t> VisitRawScanData(
      const open_lmm::AlgorithmExecutionContext&, const fs::path&,
      const RawScanVisitor&, open_lmm::AlgorithmProgressPhase) override {
    return open_lmm::Result<std::size_t>::Ok(0);
  }

 private:
  std::shared_ptr<AdmissionBarrier> barrier_;
};

class CoordinatedAdmissionFactory final : public open_lmm::AlgorithmFactory {
 public:
  CoordinatedAdmissionFactory()
      : barrier_(std::make_shared<AdmissionBarrier>()) {}

 protected:
  open_lmm::Result<std::unique_ptr<open_lmm::DataLoaderBase>>
  CreateDataLoaderImpl(const open_lmm::DataLoaderConfig&) const override {
    return open_lmm::Result<
        std::unique_ptr<open_lmm::DataLoaderBase>>::Ok(
        std::make_unique<CoordinatedAdmissionLoader>(barrier_));
  }

 private:
  std::shared_ptr<AdmissionBarrier> barrier_;
};

class PreviewOptimizer final : public open_lmm::BackendOptimizerBase {
 public:
  open_lmm::Result<open_lmm::BackendOptimizerOutput> Process(
      const open_lmm::AlgorithmExecutionContext&,
      const open_lmm::BackendOptimizerInput&) override {
    return open_lmm::Result<open_lmm::BackendOptimizerOutput>::Ok({});
  }
  void Reset() override {}
  bool HasProcessedAgent(const open_lmm::AgentId&) const override {
    return false;
  }
  std::size_t ProcessedAgentCount() const override { return 0; }
};

void TestExecutorsRequireExplicitInvocationState() {
  open_lmm::DataLoadExecutor data_load;
  open_lmm::MapUpdateExecutor map_update;
  Check(!data_load.Execute({}),
        "DataLoad rejects an invocation without committed context");
  Check(!map_update.Execute({}),
        "MapUpdate rejects an invocation without committed context");
}

void TestResidentReservationMovesWithCandidatePayload() {
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{1, 1, 4096});
  auto admitted = governor->ReserveMemory(
      512, open_lmm::MemoryClass::kResidentPayload);
  Check(admitted.IsOk(), "fixture resident reservation admitted");
  auto reservation = std::make_shared<open_lmm::MemoryReservation>(
      std::move(admitted).Value());
  auto payload = std::make_shared<open_lmm::RuntimePayload>();
  payload->resident_memory_reservations.emplace(Id("A"), reservation);
  open_lmm::ExecutionCandidate candidate{
      7, payload, {Id("A")},
      open_lmm::ArtifactCompletionKind::kDataLoadStage, std::nullopt};
  reservation.reset();
  payload.reset();
  Check(governor->ReservedMemoryBytes(
            open_lmm::MemoryClass::kResidentPayload) == 512,
        "candidate payload retains resident reservation after handoff");
  candidate.payload.reset();
  Check(governor->ReservedMemoryBytes(
            open_lmm::MemoryClass::kResidentPayload) == 0,
        "reservation releases with the final candidate payload owner");
}

void TestMapUpdateCandidateSharesExistingReservation() {
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{1, 1, 4096});
  auto admitted = governor->ReserveMemory(
      768, open_lmm::MemoryClass::kResidentPayload);
  Check(admitted.IsOk(), "base resident reservation admitted");
  auto base_payload = std::make_shared<open_lmm::RuntimePayload>();
  base_payload->resident_memory_reservations.emplace(
      Id("A"), std::make_shared<open_lmm::MemoryReservation>(
                   std::move(admitted).Value()));
  open_lmm::ExecutionCandidate candidate{
      9, base_payload, {Id("A")},
      open_lmm::ArtifactCompletionKind::kMapUpdateStage, std::nullopt};
  Check(candidate.payload.get() == base_payload.get(),
        "MapUpdate candidate preserves the committed payload identity");
  base_payload.reset();
  Check(governor->ReservedMemoryBytes(
            open_lmm::MemoryClass::kResidentPayload) == 768,
        "MapUpdate candidate shares the committed raw reservation owner");
  candidate.payload.reset();
  Check(governor->ReservedMemoryBytes(
            open_lmm::MemoryClass::kResidentPayload) == 0,
        "shared reservation releases after candidate retirement");
}

void TestResidentReplacementUsesOnlyRetiringOwnershipAsCredit() {
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{1, 1, 1024});
  auto old = governor->ReserveMemory(
      1024, open_lmm::MemoryClass::kResidentPayload);
  Check(old.IsOk(), "fill the resident budget with committed ownership");
  Check(!governor->ReserveMemory(
             1024, open_lmm::MemoryClass::kResidentPayload),
        "ordinary admission cannot double the resident budget");
  auto replacement = governor->ReserveReplacementMemory(
      4096, old.Value().Bytes(), 4096);
  Check(replacement.IsOk() && governor->ReservedMemoryBytes() == 5120,
        "replacement estimate may provisionally exceed measured ownership");
  Check(!governor->ReserveReplacementMemory(1, 2049, 1),
        "replacement credit cannot exceed current resident ownership");
  const uint64_t replacement_credit = old.Value().Bytes();
  auto old_reservation = std::move(old).Value();
  auto replacement_reservation = std::move(replacement).Value();
  Check(replacement_reservation.Resize(1500).IsOk() &&
            !governor->ValidateReplacementMemory(replacement_credit),
        "replacement whose measured result exceeds the final budget is rejected");
  Check(replacement_reservation.Resize(1024).IsOk(),
        "provisional estimate can shrink to measured resident ownership");
  Check(governor->ValidateReplacementMemory(replacement_credit).IsOk(),
        "replacement result fits after retiring committed ownership");
  old_reservation.Reset();
  Check(governor->ReservedMemoryBytes() == 1024,
        "retiring the old payload returns accounting to the soft limit");
  replacement_reservation.Reset();
  Check(governor->ReservedMemoryBytes() == 0,
        "replacement ownership releases normally");
}

void TestDataLoadPublishesEachCandidateAgent() {
  auto config = std::make_shared<open_lmm::RuntimeConfig>();
  config->data_loader =
      std::make_shared<const open_lmm::DataLoaderConfig>();
  config->documents = std::make_shared<open_lmm::RuntimeConfigDocuments>();
  config->root.max_parallel_agents = 1;
  config->root.save_voxel_size = 0.25;
  auto payload = std::make_shared<open_lmm::RuntimePayload>();
  payload->database = std::make_shared<open_lmm::SharedDatabase>();
  auto committed = std::make_shared<open_lmm::RuntimeState>();
  committed->revision = 5;
  committed->config = std::move(config);
  committed->payload = std::move(payload);

  std::vector<open_lmm::AgentPipelineCtx> contexts(2);
  contexts[0].agent = {.id = Id("A"), .role = open_lmm::AgentRole::kAnchor,
                       .order = 0};
  contexts[1].agent = {.id = Id("B"), .role = open_lmm::AgentRole::kFollower,
                       .order = 1};
  std::vector<open_lmm::AgentId> published;
  std::vector<open_lmm::AlgorithmProgress> progress;
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{1, 1, 1 << 20});
  open_lmm::DataLoadExecutor executor;
  auto result = executor.Execute(
      {committed, std::move(contexts),
       std::make_shared<open_lmm::SharedDatabase>(),
       governor,
       std::make_shared<open_lmm::CancellationToken>(),
       [&progress](const open_lmm::AlgorithmProgress& update) {
         progress.push_back(update);
       },
       std::make_shared<PreviewOptimizer>(),
       std::make_shared<PreviewFactory>(), 0.25F, false, 1,
       [&published](const open_lmm::AgentId& agent,
                    const open_lmm::AgentRawDataHandle& raw,
                    const open_lmm::VisualizationPointPreviewHandle& preview) {
         Check(raw && raw->agent_id == agent && raw->odom_poses.size() == 1 &&
                   preview && preview->voxel_millimeters == 250 &&
                   preview->points.size() == 1 &&
                   preview->points.front().x == 1.0F,
               "candidate callback receives immutable loaded data");
         published.push_back(agent);
       }});
  Check(result && published == std::vector<open_lmm::AgentId>{Id("A"), Id("B")},
        "DataLoad publishes an odometry candidate after each agent finishes");
  Check(result.Value().payload->resident_memory_reservations.size() == 2 &&
            result.Value().payload->resident_memory_reservations.at(Id("A"))
                    ->Bytes() > 1 &&
            result.Value().payload->resident_memory_reservations.at(Id("B"))
                    ->Bytes() > 1,
        "custom loaders receive final measured resident calibration");
  Check(progress.size() == 4 && progress[0].agent == Id("A") &&
            progress[0].phase ==
                open_lmm::AlgorithmProgressPhase::kBuildPreview &&
            progress[0].current == 0 && progress[0].total == 1 &&
            progress[1].agent == Id("A") && progress[1].current == 1 &&
            progress[2].agent == Id("B") && progress[2].current == 0 &&
            progress[3].agent == Id("B") && progress[3].current == 1,
        "DataLoad reports one active preview interval per completed agent");

  auto agent_state = std::make_shared<open_lmm::RuntimeState>();
  agent_state->revision = 6;
  agent_state->config = committed->config;
  agent_state->payload = result.Value().payload;
  std::vector<open_lmm::AlgorithmProgress> agent_progress;
  auto agent_result = executor.ExecuteAgent(
      {agent_state, {}, {}, governor,
       std::make_shared<open_lmm::CancellationToken>(),
       [&agent_progress](const open_lmm::AlgorithmProgress& update) {
         agent_progress.push_back(update);
       },
       std::make_shared<PreviewOptimizer>(),
       std::make_shared<PreviewFactory>(), 0.25F, false, 1, {}},
      Id("A"));
  Check(agent_result && agent_progress.size() == 2 &&
            agent_progress[0].phase ==
                open_lmm::AlgorithmProgressPhase::kBuildPreview &&
            agent_progress[0].current == 0 &&
            agent_progress[1].phase ==
                open_lmm::AlgorithmProgressPhase::kBuildPreview &&
            agent_progress[1].current == 1,
        "single-agent DataLoad uses the same active progress contract");
}

void TestParallelDataLoadAdmissionFailureDrainsAllTasks() {
  auto config = std::make_shared<open_lmm::RuntimeConfig>();
  config->data_loader =
      std::make_shared<const open_lmm::DataLoaderConfig>();
  config->documents = std::make_shared<open_lmm::RuntimeConfigDocuments>();
  config->root.max_parallel_agents = 2;
  auto payload = std::make_shared<open_lmm::RuntimePayload>();
  payload->database = std::make_shared<open_lmm::SharedDatabase>();
  auto committed = std::make_shared<open_lmm::RuntimeState>();
  committed->revision = 8;
  committed->config = std::move(config);
  committed->payload = std::move(payload);

  std::vector<open_lmm::AgentPipelineCtx> contexts(2);
  contexts[0].agent = {.id = Id("A"), .role = open_lmm::AgentRole::kAnchor,
                       .order = 0};
  contexts[1].agent = {.id = Id("B"), .role = open_lmm::AgentRole::kFollower,
                       .order = 1};
  auto governor = std::make_shared<open_lmm::ResourceGovernor>(
      open_lmm::ResourceBudget{2, 2, 1500});
  open_lmm::DataLoadExecutor executor;
  auto result = executor.Execute(
      {committed, std::move(contexts),
       std::make_shared<open_lmm::SharedDatabase>(), governor,
       std::make_shared<open_lmm::CancellationToken>(),
       open_lmm::AlgorithmProgressCallback{},
       std::make_shared<PreviewOptimizer>(),
       std::make_shared<CoordinatedAdmissionFactory>(), 0.2F, true, 2, {}});
  Check(!result &&
            result.GetError().Message().find(
                "memory admission rejected: class=resident_payload") !=
                std::string::npos &&
            result.GetError().context.stage == "data_load" &&
            governor->ReservedMemoryBytes() == 0 &&
            governor->MemoryAdmissionFailures() == 1,
        "parallel admission failure drains tasks and releases all ownership");
}

}  // namespace

int main() {
  TestExecutorsRequireExplicitInvocationState();
  TestResidentReservationMovesWithCandidatePayload();
  TestMapUpdateCandidateSharesExistingReservation();
  TestResidentReplacementUsesOnlyRetiringOwnershipAsCredit();
  TestDataLoadPublishesEachCandidateAgent();
  TestParallelDataLoadAdmissionFailureDrainsAllTasks();
  std::cout << "stage executor fixture tests passed\n";
  return 0;
}
