#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/server/session_manager.hpp>
#include <open_lmm/server/session_payload_builder.hpp>
#include <open_lmm/server/session_state.hpp>
#include <open_lmm/server/transaction/session_reconfigurer.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <string>

namespace open_lmm {
namespace {

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

class FakeOptimizer final : public BackendOptimizerBase {
 public:
  explicit FakeOptimizer(std::set<AgentId> processed = {})
      : processed_(std::move(processed)) {}

  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext&,
      const BackendOptimizerInput&) override {
    return Result<BackendOptimizerOutput>::Ok({});
  }
  void Reset() override { processed_.clear(); }
  bool HasProcessedAgent(const AgentId& agent) const override {
    return processed_.contains(agent);
  }
  std::size_t ProcessedAgentCount() const override {
    return processed_.size();
  }

 private:
  std::set<AgentId> processed_;
};

std::shared_ptr<const SessionConfig> TestConfig() {
  auto config = std::make_shared<SessionConfig>();
  config->data_loader = std::make_shared<const DataLoaderConfig>();
  config->loop_detector = std::make_shared<const LoopDetectorConfig>();
  config->optimizer = std::make_shared<const OptimizerConfig>();
  config->dynamic_remover = std::make_shared<const DynamicRemoverConfig>();
  config->map_save = std::make_shared<const MapSaveConfig>();
  return config;
}

std::shared_ptr<const SessionState> DataLoadedState() {
  auto raw = std::make_shared<AgentRawData>();
  raw->agent_id = Id("A");
  raw->odom_poses.push_back(Eigen::Isometry3d::Identity());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data[Id("A")] = raw;
  AgentPipelineCtx context;
  context.agent = {.id = Id("A"), .role = AgentRole::kAnchor, .order = 0};
  context.raw_data = raw;
  auto payload = std::make_shared<SessionPayload>();
  payload->contexts.push_back(std::move(context));
  payload->database = std::move(database);
  payload->optimizer = std::make_shared<FakeOptimizer>();
  payload->resident_memory_reservations[Id("A")] =
      std::make_shared<MemoryReservation>();
  auto state = std::make_shared<SessionState>();
  state->revision = 7;
  state->config = TestConfig();
  state->ordered_agents = {Id("A")};
  state->payload = std::move(payload);
  state->artifacts.push_back(
      {{ArtifactType::kRawData, Id("A")}, ArtifactState::kReady, 11,
       "data_load"});
  return state;
}

std::shared_ptr<const SessionState> AlignedState() {
  auto base = DataLoadedState();
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  auto optimized = std::make_shared<AgentOptimizedData>();
  optimized->agent_id = Id("A");
  optimized->optimized_poses.emplace_back(
      0, Eigen::Isometry3d::Identity());
  database->optimized_data[Id("A")] = optimized;
  AgentPipelineCtx context = base->payload->contexts.front();
  auto loop = std::make_shared<LoopDetectorOutput>();
  loop->accepted_global_T_agent = Eigen::Isometry3d::Identity();
  loop->accepted_alignment_method = AlignmentMethod::kKissMatcher;
  loop->accepted_alignment_approval = AlignmentApproval::kAutomatic;
  context.loop_output = loop;
  auto payload = std::make_shared<SessionPayload>();
  payload->contexts.push_back(std::move(context));
  payload->database = std::move(database);
  payload->optimizer = std::make_shared<FakeOptimizer>(std::set<AgentId>{Id("A")});
  payload->resident_memory_reservations =
      base->payload->resident_memory_reservations;
  auto state = std::make_shared<SessionState>();
  state->revision = 12;
  state->config = base->config;
  state->ordered_agents = {Id("A")};
  state->payload = std::move(payload);
  state->optimizer.processed_agents = {Id("A")};
  state->artifacts = {
      {{ArtifactType::kRawData, Id("A")}, ArtifactState::kReady, 20, "data_load"},
      {{ArtifactType::kLoopCandidates, Id("A")}, ArtifactState::kReady, 21,
       "loop_detect"},
      {{ArtifactType::kMapAlignment, Id("A")}, ArtifactState::kReady, 22,
       "loop_detect"},
      {{ArtifactType::kDescriptorState, std::nullopt}, ArtifactState::kReady,
       23, "loop_detect"},
      {{ArtifactType::kOptimizerState, std::nullopt}, ArtifactState::kReady,
       24, "optimize"},
      {{ArtifactType::kOptimizedPoses, Id("A")}, ArtifactState::kReady, 25,
       "optimize"},
  };
  return state;
}

void TestCancelledDataLoadPreservesCommittedRawPayload() {
  auto base = DataLoadedState();
  const auto* committed_raw =
      base->payload->database->raw_data.at(Id("A")).get();
  const auto committed_artifacts = base->artifacts;
  SessionTransaction transaction(base);
  auto replacement = std::make_shared<AgentRawData>();
  replacement->agent_id = Id("A");
  replacement->odom_poses.resize(2, Eigen::Isometry3d::Identity());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data[Id("A")] = replacement;
  auto payload = std::make_shared<SessionPayload>(*base->payload);
  payload->contexts.front().raw_data = replacement;
  payload->database = std::move(database);
  transaction.SetPayload(std::move(payload));
  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  auto result = std::move(transaction).Finalize(cancellation);
  Check(!result && result.GetError().code == Error::Code::kCancelled,
        "cancelled DataLoad transaction fails before commit");
  Check(base->revision == 7, "cancelled DataLoad preserves revision");
  Check(base->payload->database->raw_data.at(Id("A")).get() == committed_raw,
        "cancelled DataLoad preserves actual raw payload handle");
  Check(base->artifacts.front().revision == committed_artifacts.front().revision,
        "cancelled DataLoad preserves artifact metadata");
}

void TestCancelledAlignmentPreservesDescriptorPoseAndOptimizer() {
  auto base = AlignedState();
  const auto* committed_loop = base->payload->contexts.front().loop_output.get();
  const auto* committed_pose =
      base->payload->database->optimized_data.at(Id("A")).get();
  const auto* committed_optimizer = base->payload->optimizer.get();
  SessionTransaction transaction(base);
  auto payload = std::make_shared<SessionPayload>(*base->payload);
  payload->contexts.front().loop_output =
      std::make_shared<LoopDetectorOutput>();
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  auto replacement_pose = std::make_shared<AgentOptimizedData>();
  replacement_pose->agent_id = Id("A");
  database->optimized_data[Id("A")] = replacement_pose;
  payload->database = std::move(database);
  payload->optimizer = std::make_shared<FakeOptimizer>();
  transaction.SetPayload(std::move(payload));
  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  auto result = std::move(transaction).Finalize(cancellation);
  Check(!result && result.GetError().code == Error::Code::kCancelled,
        "cancelled Alignment transaction fails before commit");
  Check(base->revision == 12, "cancelled Alignment preserves revision");
  Check(base->payload->contexts.front().loop_output.get() == committed_loop,
        "cancelled Alignment preserves descriptor/loop payload");
  Check(base->payload->database->optimized_data.at(Id("A")).get() == committed_pose,
        "cancelled Alignment preserves optimized pose payload");
  Check(base->payload->optimizer.get() == committed_optimizer &&
            base->payload->optimizer->HasProcessedAgent(Id("A")),
        "cancelled Alignment preserves optimizer instance and lifecycle");
}

void TestInvalidWorkingStateCannotReplaceCommittedAlignment() {
  auto base = AlignedState();
  const auto* committed_pose =
      base->payload->database->optimized_data.at(Id("A")).get();
  SessionTransaction transaction(base);
  auto payload = std::make_shared<SessionPayload>(*base->payload);
  auto invalid_database = std::make_shared<SharedDatabase>();
  invalid_database->raw_data = base->payload->database->raw_data;
  payload->database = std::move(invalid_database);
  transaction.SetPayload(std::move(payload));
  auto result = std::move(transaction).Finalize(nullptr);
  Check(!result, "ready metadata without working pose payload is rejected");
  Check(base->revision == 12 &&
            base->payload->database->optimized_data.at(Id("A")).get() ==
                committed_pose,
        "failed Alignment validation preserves committed state");
}

std::string ReadText(const std::filesystem::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

void WriteText(const std::filesystem::path& path, const std::string& text) {
  std::ofstream output(path);
  output << text;
}

void TestOutputRepositoryRollbackAndCommit() {
  namespace fs = std::filesystem;
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path directory = fs::temp_directory_path() /
      ("open_lmm_output_transaction_" + std::to_string(nonce));
  fs::create_directories(directory);
  const fs::path temporary = directory / "value.tmp";
  const fs::path destination = directory / "value.txt";
  WriteText(temporary, "discard");
  {
    PendingOutputSet pending;
    pending.Add(temporary, destination);
  }
  Check(!fs::exists(temporary) && !fs::exists(destination),
        "uncommitted output transaction removes temporary file");

  WriteText(destination, "old");
  WriteText(temporary, "new");
  {
    PendingOutputSet pending;
    pending.Add(temporary, destination);
    Check(static_cast<bool>(pending.Commit()),
          "output transaction commits complete file set");
  }
  Check(ReadText(destination) == "new",
        "output transaction replaces destination");
  Check(!fs::exists(destination.string() + ".open_lmm_backup"),
        "output transaction removes recovery backup after commit");
  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

void TestSessionManagerRejectsStaleBaseRevision() {
  auto base = DataLoadedState();
  SessionManager manager(base);

  SessionTransaction first(base);
  auto first_candidate = std::move(first).Finalize(nullptr);
  Check(first_candidate.IsOk(), "first session candidate finalizes");
  Check(manager.Commit(base, std::move(first_candidate).Value()).IsOk(),
        "first session candidate commits");

  SessionTransaction stale(base);
  auto stale_candidate = std::move(stale).Finalize(nullptr);
  Check(stale_candidate.IsOk(), "stale candidate can finish local validation");
  auto conflict = manager.Commit(base, std::move(stale_candidate).Value());
  Check(!conflict, "stale base revision cannot replace active session");
  Check(manager.Snapshot()->revision == base->revision + 1,
        "revision conflict preserves the active committed state");
}

void TestPostFinalizeCancellationDoesNotRollbackCommit() {
  auto base = DataLoadedState();
  SessionManager manager(base);
  SessionTransaction transaction(base);
  auto candidate = std::move(transaction).Finalize(nullptr);
  Check(candidate.IsOk(), "candidate finalizes before late cancellation");
  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  Check(manager.Commit(base, std::move(candidate).Value()).IsOk(),
        "late cancellation cannot roll back an already finalized commit");
  Check(manager.Snapshot()->revision == base->revision + 1,
        "late cancellation leaves the new committed revision active");
}

void TestSessionCommitBarrierKeepsFilesAndStateInOneConflictWindow() {
  auto base = DataLoadedState();
  SessionManager manager(base);
  SessionTransaction rejected(base);
  auto rejected_candidate = std::move(rejected).Finalize(nullptr);
  bool installed = false;
  auto file_failure = manager.CommitWithBarrier(
      base, std::move(rejected_candidate).Value(), [&] {
        installed = true;
        return Result<void>::Failure(Error::IoError("injected file failure"));
      });
  Check(!file_failure && installed && manager.Snapshot().get() == base.get(),
        "file barrier failure preserves the committed session state");

  SessionTransaction accepted(base);
  auto accepted_candidate = std::move(accepted).Finalize(nullptr);
  Check(manager.Commit(base, std::move(accepted_candidate).Value()).IsOk(),
        "advance session before stale barrier attempt");
  installed = false;
  SessionTransaction stale(base);
  auto stale_candidate = std::move(stale).Finalize(nullptr);
  auto conflict = manager.CommitWithBarrier(
      base, std::move(stale_candidate).Value(), [&] {
        installed = true;
        return Result<void>::Ok();
      });
  Check(!conflict && !installed,
        "revision conflict is rejected before file installation");
}

void TestPayloadBuilderKeepsRawAndReservationLifetimeCoupled() {
  auto base = DataLoadedState();
  auto reused = SessionPayloadBuilder(base->payload).Build();
  Check(reused.IsOk(), "payload builder accepts shared raw ownership");
  Check(reused.Value()->resident_memory_reservations.at(Id("A")).get() ==
            base->payload->resident_memory_reservations.at(Id("A")).get(),
        "shared raw payload keeps the same reservation owner");

  auto replacement = std::make_shared<AgentRawData>();
  replacement->agent_id = Id("A");
  replacement->odom_poses.push_back(Eigen::Isometry3d::Identity());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data[Id("A")] = replacement;
  auto contexts = base->payload->contexts;
  contexts.front().raw_data = replacement;
  auto mismatched = SessionPayloadBuilder(base->payload)
                        .SetContexts(std::move(contexts))
                        .SetDatabase(std::move(database))
                        .Build();
  Check(!mismatched,
        "replaced raw payload cannot reuse the previous reservation owner");

  auto replacement_database = std::make_shared<SharedDatabase>();
  replacement_database->raw_data[Id("A")] = replacement;
  auto replacement_contexts = base->payload->contexts;
  replacement_contexts.front().raw_data = replacement;
  auto replaced =
      SessionPayloadBuilder(base->payload)
          .SetContexts(std::move(replacement_contexts))
          .SetDatabase(std::move(replacement_database))
          .SetResidentReservation(Id("A"),
                                  std::make_shared<MemoryReservation>())
          .Build();
  Check(replaced.IsOk(),
        "replaced raw payload accepts a distinct reservation owner");
}

void TestRejectedReconfigurePreservesCommittedResidentOwnership() {
  const auto base = DataLoadedState();
  const auto* reservation =
      base->payload->resident_memory_reservations.at(Id("A")).get();
  const auto* raw = base->payload->database->raw_data.at(Id("A")).get();
  const uint64_t revision = base->revision;
  const auto artifacts = base->artifacts;

  // The fixture intentionally has no canonical config documents, so prepare
  // must fail before a candidate can be published or committed.
  const auto rejected = SessionReconfigurer().Prepare(
      base, ConfigDomain::kOptimizer, base->config->revision + 1);
  Check(!rejected, "invalid reconfigure snapshot is rejected");
  Check(base->revision == revision && base->artifacts.size() == artifacts.size() &&
            base->payload->database->raw_data.at(Id("A")).get() == raw &&
            base->payload->resident_memory_reservations.at(Id("A")).get() ==
                reservation,
        "reconfigure failure preserves revision, artifacts, raw and resident owner");
  for (std::size_t i = 0; i < artifacts.size(); ++i) {
    Check(base->artifacts[i].key == artifacts[i].key &&
              base->artifacts[i].state == artifacts[i].state &&
              base->artifacts[i].revision == artifacts[i].revision,
          "reconfigure failure preserves artifact metadata");
  }
}

}  // namespace
}  // namespace open_lmm

int main() {
  open_lmm::TestCancelledDataLoadPreservesCommittedRawPayload();
  open_lmm::TestCancelledAlignmentPreservesDescriptorPoseAndOptimizer();
  open_lmm::TestInvalidWorkingStateCannotReplaceCommittedAlignment();
  open_lmm::TestOutputRepositoryRollbackAndCommit();
  open_lmm::TestSessionManagerRejectsStaleBaseRevision();
  open_lmm::TestPostFinalizeCancellationDoesNotRollbackCommit();
  open_lmm::TestSessionCommitBarrierKeepsFilesAndStateInOneConflictWindow();
  open_lmm::TestPayloadBuilderKeepsRawAndReservationLifetimeCoupled();
  open_lmm::TestRejectedReconfigurePreservesCommittedResidentOwnership();
  std::cout << "session transaction tests passed\n";
  return 0;
}
