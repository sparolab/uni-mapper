#include <runtime/execution/stages/data_load_executor.hpp>
#include <runtime/execution/stages/map_update_executor.hpp>
#include <plugins/host/algorithm_factory.hpp>
#include <domain/data_loader/data_loader_base.hpp>

#include <cstdlib>
#include <iostream>

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
      const open_lmm::DataLoaderInput&) override {
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
  open_lmm::DataLoadExecutor executor;
  auto result = executor.Execute(
      {committed, std::move(contexts),
       std::make_shared<open_lmm::SharedDatabase>(),
       std::make_shared<open_lmm::ResourceGovernor>(
           open_lmm::ResourceBudget{1, 1, 1 << 20}),
       std::make_shared<open_lmm::CancellationToken>(),
       open_lmm::AlgorithmProgressCallback{},
       std::make_shared<PreviewOptimizer>(),
       std::make_shared<PreviewFactory>(), false, 1,
       [&published](const open_lmm::AgentId& agent,
                    const open_lmm::AgentRawDataHandle& raw) {
         Check(raw && raw->agent_id == agent && raw->odom_poses.size() == 1,
               "candidate callback receives immutable loaded data");
         published.push_back(agent);
       }});
  Check(result && published == std::vector<open_lmm::AgentId>{Id("A"), Id("B")},
        "DataLoad publishes an odometry candidate after each agent finishes");
}

}  // namespace

int main() {
  TestExecutorsRequireExplicitInvocationState();
  TestResidentReservationMovesWithCandidatePayload();
  TestMapUpdateCandidateSharesExistingReservation();
  TestResidentReplacementUsesOnlyRetiringOwnershipAsCredit();
  TestDataLoadPublishesEachCandidateAgent();
  std::cout << "stage executor fixture tests passed\n";
  return 0;
}
