#include "stage_executor.hpp"

#include <algorithm>
#include <array>
#include <map>
#include <utility>

#include <nlohmann/json.hpp>

#include <runtime/state/artifact_repository.hpp>
#include <runtime/composition/runtime_bootstrapper.hpp>
#include <config/document/config.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include <foundation/logging/logging.hpp>

namespace open_lmm {
namespace {

namespace fs = std::filesystem;

constexpr std::size_t kMaximumCatalogEntries = 64;
constexpr std::size_t kMaximumCandidatesPerDomain = 16;

bool IsWithin(const fs::path& path, const fs::path& root) {
  const fs::path relative = path.lexically_relative(root);
  return !relative.empty() && *relative.begin() != "..";
}

Result<std::string> CandidateModel(ConfigDocumentKind kind,
                                   const nlohmann::json& document) {
  try {
    if (kind == ConfigDocumentKind::kLoopDetector) {
      const auto& loop = document.at("loop_detector");
      const std::string type = loop.at("loop_detector_type").get<std::string>();
      const std::string model = loop.at("model").get<std::string>();
      if (type == "kdtree" && (model == "scan_context" || model == "solid")) {
        return Result<std::string>::Ok(model);
      }
    } else if (kind == ConfigDocumentKind::kDynamicRemover) {
      const std::string model =
          document.at("dynamic_remover").at("model").get<std::string>();
      static constexpr std::array<std::string_view, 5> kAllowed = {
          "erasor", "dufomap", "free_dom", "hmm_mos", "otd"};
      if (std::find(kAllowed.begin(), kAllowed.end(), model) != kAllowed.end()) {
        return Result<std::string>::Ok(model);
      }
    }
  } catch (const std::exception& error) {
    return Result<std::string>::Failure(Error::ParseError(
        std::string("candidate model selector is invalid: ") + error.what()));
  }
  return Result<std::string>::Failure(
      Error::InvalidArgument("candidate model is not allowed"));
}

Result<void> AppendCandidates(const RuntimeConfigDocument& committed,
                              const RuntimeConfigDocument& root_document,
                              const SchemaRegistry& registry,
                              ConfigDomain domain, ConfigDocumentKind kind,
                              ConfigCandidateCatalog& catalog) {
  std::error_code error;
  const fs::path root = fs::weakly_canonical(root_document.path.parent_path(), error);
  if (error) {
    return Result<void>::Failure(Error::IoError(
        "failed to resolve committed config root: " + error.message()));
  }
  const fs::path directory = fs::weakly_canonical(committed.path.parent_path(), error);
  if (error || !IsWithin(directory, root)) {
    return Result<void>::Failure(Error::InvalidArgument(
        "candidate directory is outside the committed config root"));
  }

  std::vector<fs::path> paths;
  fs::directory_iterator iterator(directory, error);
  fs::directory_iterator end;
  if (error) {
    return Result<void>::Failure(Error::IoError(
        "failed to enumerate config candidate directory: " + error.message()));
  }
  std::size_t entries = 0;
  for (; iterator != end; iterator.increment(error)) {
    if (error) {
      return Result<void>::Failure(Error::IoError(
          "failed to enumerate config candidate directory: " + error.message()));
    }
    if (++entries > kMaximumCatalogEntries) {
      return Result<void>::Failure(Error::InvalidArgument(
          "config candidate directory exceeds entry limit"));
    }
    if (iterator->path().extension() == ".json") paths.push_back(iterator->path());
  }
  if (error) {
    return Result<void>::Failure(Error::IoError(
        "failed to enumerate config candidate directory: " + error.message()));
  }
  std::sort(paths.begin(), paths.end());
  if (paths.size() > kMaximumCandidatesPerDomain) {
    return Result<void>::Failure(
        Error::InvalidArgument("config candidate count exceeds domain limit"));
  }

  std::map<std::string, fs::path> models;
  for (const auto& path : paths) {
    const fs::path canonical = fs::weakly_canonical(path, error);
    if (error || !IsWithin(canonical, root)) {
      return Result<void>::Failure(Error::InvalidArgument(
          "config candidate is outside the committed config root"));
    }
    if (!fs::is_regular_file(canonical, error) || error) {
      return Result<void>::Failure(
          Error::InvalidArgument("config candidate must be a regular file"));
    }
    auto loaded = LoadConfigFileBounded(
        canonical, SchemaLimits{}.maximum_document_bytes);
    if (!loaded) return Result<void>::Failure(loaded.GetError());
    nlohmann::json raw;
    try {
      raw = nlohmann::json::parse(loaded.Value().ToJson());
    } catch (const std::exception& parse_error) {
      return Result<void>::Failure(Error::ParseError(
          std::string("candidate snapshot is invalid: ") + parse_error.what()));
    }
    const char* domain_key = kind == ConfigDocumentKind::kLoopDetector
                                 ? "loop_detector"
                                 : "dynamic_remover";
    if (!raw.contains(domain_key)) continue;
    // Stale or unrelated JSON may share a directory with active module
    // documents. Only recognized production models enter the trusted catalog;
    // recognized models must then pass the complete schema below.
    auto raw_model = CandidateModel(kind, raw);
    if (!raw_model) continue;
    auto validated = registry.ParseAndValidate(
        kind, raw.dump(), canonical.string());
    if (!validated) return Result<void>::Failure(validated.GetError());
    const std::string model = std::move(raw_model).Value();
    if (!models.emplace(model, canonical).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "duplicate config candidate model: " + model));
    }
    const fs::path selector = canonical.lexically_relative(root);
    catalog.candidates.push_back(
        {domain, model, selector.generic_string(),
         validated.Value().CanonicalJson()});
  }
  return Result<void>::Ok();
}

class ExecutionLease {
 public:
  explicit ExecutionLease(std::atomic_flag& active)
      : active_(active),
        acquired_(!active_.test_and_set(std::memory_order_acquire)) {}
  ~ExecutionLease() {
    if (acquired_) active_.clear(std::memory_order_release);
  }
  [[nodiscard]] explicit operator bool() const { return acquired_; }

 private:
  std::atomic_flag& active_;
  bool acquired_;
};

Result<void> ValidateNodeTarget(const RuntimeState& state, NodeId node,
                                const std::optional<AgentId>& agent) {
  const auto& spec = ExecutionSpecFor(node);
  if (spec.scope == ExecutionScope::kPerAgent && !agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("per-agent node requires an agent target"));
  }
  if (spec.scope == ExecutionScope::kRuntime && agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime node must not have an agent target"));
  }
  if (agent &&
      std::find(state.ordered_agents.begin(), state.ordered_agents.end(),
                *agent) == state.ordered_agents.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }
  return Result<void>::Ok();
}

VisualizationPhase InferVisualizationPhase(const RuntimeState& state) {
  if (!state.payload || !state.payload->database) {
    return VisualizationPhase::kDataLoad;
  }
  if (!state.payload->database->optimized_data.empty()) {
    return VisualizationPhase::kOptimization;
  }
  if (std::any_of(state.payload->contexts.begin(), state.payload->contexts.end(),
                  [](const AgentPipelineCtx& context) {
                    return static_cast<bool>(context.loop_output);
                  })) {
    return VisualizationPhase::kLoopDetection;
  }
  return VisualizationPhase::kDataLoad;
}

std::vector<AgentId> ExcludedAlignmentAgents(
    const CommittedRuntimeSnapshot& snapshot) {
  std::vector<AgentId> result;
  for (const auto& artifact : snapshot.artifacts) {
    if (artifact.key.type == ArtifactType::kMapAlignment &&
        artifact.key.agent && artifact.state == ArtifactState::kFailed &&
        artifact.detail.find("excluded from alignment") != std::string::npos) {
      result.push_back(*artifact.key.agent);
    }
  }
  return result;
}

VisualizationSource BuildVisualizationSource(
    uint64_t revision, const RuntimeConfig& config,
    const std::vector<AgentPipelineCtx>& contexts,
    const SharedDatabase& database,
    VisualizationPhase phase) {
  VisualizationSource source;
  source.revision = revision;
  source.output_directory = config.root.output_directory;
  source.preview_voxel_size_m =
      static_cast<float>(config.root.save_voxel_size);
  source.agents.reserve(database.raw_data.size());
  for (const auto& entry : database.raw_data) {
    const auto& agent = entry.first;
    const auto& raw = entry.second;
    const auto optimized = database.optimized_data.find(agent);
    const auto context = std::find_if(
        contexts.begin(), contexts.end(),
        [agent](const AgentPipelineCtx& item) {
          return item.agent.id == agent;
        });
    if (context != contexts.end() &&
        context->flow == ControlFlow::kSkip &&
        phase != VisualizationPhase::kDataLoad &&
        phase != VisualizationPhase::kLoopDetection) {
      continue;
    }
    source.agents.push_back(
        {agent, raw,
         optimized == database.optimized_data.end() ? nullptr
                                                    : optimized->second,
         context == contexts.end() ? nullptr : context->loop_output});
  }
  return source;
}

std::optional<VisualizationSource> BuildVisualizationSource(
    const std::shared_ptr<const RuntimeState>& state,
    VisualizationPhase phase) {
  if (!state || !state->config || !state->payload ||
      !state->payload->database) {
    return std::nullopt;
  }
  return BuildVisualizationSource(state->revision, *state->config,
                                  state->payload->contexts,
                                  *state->payload->database, phase);
}

}  // namespace

StageExecutor::StageExecutor(
    BootstrapConfigSnapshot bootstrap_config,
    std::optional<std::filesystem::path> output_directory,
    std::shared_ptr<ResourceGovernor> resource_governor,
    std::function<void()> before_presentation_publish)
    : resource_governor_(std::move(resource_governor)),
      before_presentation_publish_(std::move(before_presentation_publish)) {
  InitializeLogging();
  RuntimeBootstrapper bootstrap;
  auto initialized = bootstrap.Bootstrap(
      {std::move(bootstrap_config), std::move(output_directory), {}});
  if (!initialized) {
    initialization_error_ = initialized.GetError();
    return;
  }
  auto result = std::move(initialized).Value();
  if (!resource_governor_) {
    resource_governor_ =
        std::make_shared<ResourceGovernor>(result.suggested_resource_budget);
  }
  runtime_state_store_.Initialize(std::move(result.initial_state),
                                  std::move(result.recovery_required));
  coordinator_ = std::make_unique<StageCoordinator>(
      runtime_state_store_, output_repository_, resource_governor_,
      std::move(result.algorithms),
      [this](uint64_t base_revision, const AgentId& agent,
             const AgentRawDataHandle& raw,
             const VisualizationPointPreviewHandle& preview) {
        visualization_projector_.PublishDataLoadCandidate(base_revision,
                                                          agent, raw, preview);
      },
      [this](uint64_t base_revision,
             const std::vector<AgentPipelineCtx>& contexts,
             const SharedDatabase& database) {
        const auto committed = CommittedState();
        if (!committed || !committed->config ||
            committed->revision != base_revision) {
          return;
        }
        visualization_projector_.PublishAlignmentCandidate(
            BuildVisualizationSource(base_revision, *committed->config,
                                     contexts, database,
                                     VisualizationPhase::kOptimization));
      });
}

StageExecutor::~StageExecutor() = default;

std::shared_ptr<const RuntimeState> StageExecutor::CommittedState() const {
  return runtime_state_store_.Snapshot();
}

Result<void> StageExecutor::EnsureReady() {
  if (initialization_error_) {
    Error error = *initialization_error_;
    error.MarkFatalRuntime();
    return Result<void>::Failure(std::move(error));
  }
  const auto state = CommittedState();
  if (!state || !state->config || !state->payload ||
      !state->payload->database || !state->payload->optimizer ||
      state->ordered_agents.empty()) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime state is unavailable"));
  }
  if (state->ordered_agents.size() > AgentSymbolCatalog::kMaximumAgents) {
    return Result<void>::Failure(Error::InvalidArgument(
        "At most 255 agents are supported by the GTSAM symbol catalog"));
  }
  if (!coordinator_) {
    return Result<void>::Failure(
        Error::InvalidArgument("stage coordinator is unavailable"));
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::EnsureMutationAllowed() {
  auto ready = EnsureReady();
  if (!ready) return ready;
  const auto authority = runtime_state_store_.AuthoritySnapshot();
  return authority.recovery_required
             ? Result<void>::Failure(*authority.recovery_required)
             : Result<void>::Ok();
}

Result<void> StageExecutor::ValidateReady() {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  return EnsureReady();
}

CancellationCapability StageExecutor::CancellationMetadata() const {
  return {
      .cooperative = false,
      .mode = CancellationMode::kHostSafePoints,
      .non_interruptible_operations =
          {"descriptor plugin call", "map alignment registration",
           "dynamic remover plugin call", "GTSAM optimize"},
      .requires_process_isolation = true,
  };
}

CommittedRuntimeSnapshot StageExecutor::Snapshot() const {
  const auto authority = runtime_state_store_.AuthoritySnapshot();
  const auto& state = authority.state;
  if (!state) return {};
  CommittedRuntimeSnapshot snapshot{state->revision,
                                    state->config ? state->config->revision : 0,
                                    state->ordered_agents, state->artifacts};
  if (state->payload && state->payload->database) {
    const auto& descriptors = state->payload->database->descriptor_store;
    snapshot.descriptor_count =
        descriptors.total_db ? descriptors.total_db->getSize() : 0;
    for (const auto& [agent, database] : descriptors.per_agent_db) {
      snapshot.per_agent_descriptor_count[agent] =
          database ? database->getSize() : 0;
    }
  }
  snapshot.recovery_required = authority.recovery_required;
  return snapshot;
}

Result<CommittedConfigDocuments> StageExecutor::ConfigDocuments() const {
  const auto state = CommittedState();
  if (!state || !state->config || !state->config->documents) {
    return Result<CommittedConfigDocuments>::Failure(
        Error::InvalidArgument("committed config documents are unavailable"));
  }
  const auto& source = *state->config->documents;
  nlohmann::ordered_json root;
  try {
    root = nlohmann::ordered_json::parse(source.root.canonical_json);
  } catch (const std::exception& error) {
    return Result<CommittedConfigDocuments>::Failure(
        Error::ParseError(std::string("committed root config is invalid: ") +
                          error.what()));
  }

  const auto selected = [&root](const char* key)
      -> Result<std::optional<std::string>> {
    try {
      const auto& value = root.at("global").at(key);
      if (!value.is_string() || value.get_ref<const std::string&>().empty()) {
        return Result<std::optional<std::string>>::Failure(
            Error::InvalidArgument(std::string("committed selector is invalid: ") +
                                   key));
      }
      return Result<std::optional<std::string>>::Ok(
          value.get<std::string>());
    } catch (const std::exception& error) {
      return Result<std::optional<std::string>>::Failure(
          Error::ParseError(std::string("committed selector is missing: ") +
                            key + ": " + error.what()));
    }
  };

  CommittedConfigDocuments result;
  result.runtime_revision = state->revision;
  result.config_revision = state->config->revision;
  result.documents.push_back(
      {ConfigDomain::kGlobal, source.root.canonical_json, std::nullopt});
  const struct {
    ConfigDomain domain;
    const RuntimeConfigDocument* document;
    const char* selector;
  } modules[] = {
      {ConfigDomain::kDataLoader, &source.data_loader, "config_data_loader"},
      {ConfigDomain::kLoopDetector, &source.loop_detector,
       "config_loop_detector"},
      {ConfigDomain::kOptimizer, &source.optimizer,
       "config_backend_optimizer"},
      {ConfigDomain::kDynamicRemover, &source.dynamic_remover,
       "config_dynamic_remover"},
      {ConfigDomain::kMapSave, &source.map_server, "config_map_server"},
  };
  for (const auto& module : modules) {
    auto selector = selected(module.selector);
    if (!selector) {
      return Result<CommittedConfigDocuments>::Failure(selector.GetError());
    }
    result.documents.push_back(
        {module.domain, module.document->canonical_json,
         std::move(selector).Value()});
  }
  return Result<CommittedConfigDocuments>::Ok(std::move(result));
}

Result<ConfigCandidateCatalog> StageExecutor::ConfigCandidates() const {
  const auto state = CommittedState();
  if (!state || !state->config || !state->config->documents) {
    return Result<ConfigCandidateCatalog>::Failure(
        Error::InvalidArgument("committed config documents are unavailable"));
  }
  const auto& config = *state->config;
  const auto& documents = *config.documents;
  const SchemaRegistry& registry = config.schema_registry
                                       ? *config.schema_registry
                                       : BuiltinConfigSchemaRegistry();
  ConfigCandidateCatalog result;
  result.runtime_revision = state->revision;
  result.config_revision = config.revision;
  auto loop = AppendCandidates(
      documents.loop_detector, documents.root, registry,
      ConfigDomain::kLoopDetector, ConfigDocumentKind::kLoopDetector, result);
  if (!loop) return Result<ConfigCandidateCatalog>::Failure(loop.GetError());
  auto remover = AppendCandidates(
      documents.dynamic_remover, documents.root, registry,
      ConfigDomain::kDynamicRemover, ConfigDocumentKind::kDynamicRemover,
      result);
  if (!remover) {
    return Result<ConfigCandidateCatalog>::Failure(remover.GetError());
  }
  return Result<ConfigCandidateCatalog>::Ok(std::move(result));
}

void StageExecutor::PublishEmptyVisualization() {
  const auto state = CommittedState();
  visualization_projector_.Clear(
      state ? state->revision : 0,
      state && state->config
          ? static_cast<float>(state->config->root.save_voxel_size)
          : 0.0F);
}

void StageExecutor::PublishVisualization(VisualizationPhase phase,
                                         bool include_maps) {
  const auto state = CommittedState();
  auto source = BuildVisualizationSource(state, phase);
  if (!source) {
    visualization_projector_.Clear(
        state ? state->revision : 0,
        state && state->config
            ? static_cast<float>(state->config->root.save_voxel_size)
            : 0.0F);
    return;
  }
  visualization_projector_.Publish(std::move(*source), phase, include_maps);
}

void StageExecutor::PublishVisualizationBestEffort(
    VisualizationPhase phase, bool include_maps,
    uint64_t base_revision) noexcept {
  try {
    if (before_presentation_publish_) before_presentation_publish_();
    PublishVisualization(phase, include_maps);
  } catch (const std::exception& error) {
    try {
      if (phase == VisualizationPhase::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base_revision);
      } else if (phase == VisualizationPhase::kOptimization) {
        visualization_projector_.RollbackAlignmentCandidate(base_revision);
      }
      LogWarning(std::string(
                     "committed runtime retained after presentation failure: ") +
                 error.what());
    } catch (...) {
      // Presentation and diagnostics are derived output. A secondary failure
      // must not erase the already committed runtime receipt.
    }
  } catch (...) {
    try {
      if (phase == VisualizationPhase::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base_revision);
      } else if (phase == VisualizationPhase::kOptimization) {
        visualization_projector_.RollbackAlignmentCandidate(base_revision);
      }
      LogWarning(
          "committed runtime retained after unknown presentation failure");
    } catch (...) {
      // Preserve committed authority even when rollback/logging also fail.
    }
  }
}

Result<VisualizationSnapshot> StageExecutor::Visualization(
    const VisualizationQuery& query) const {
  return visualization_projector_.Project(query);
}

StageExecutorDiagnostics StageExecutor::Diagnostics() const {
  const auto state = CommittedState();
  return {state ? state->revision : 0, resource_governor_->Diagnostics(),
          visualization_projector_.Diagnostics()};
}

Result<ExecutionReceipt> StageExecutor::Execute(
    const ExecutionCommand& command, const ExecutionContext& context) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = EnsureMutationAllowed();
  if (!ready) return Result<ExecutionReceipt>::Failure(ready.GetError());
  if (!context.cancellation) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("execution cancellation token is required"));
  }
  const auto base = CommittedState();
  if (context.base_revision != base->revision) {
    return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
        "execution base revision does not match committed runtime"));
  }

  Result<void> result = Result<void>::Failure(
      Error::InvalidArgument("unknown execution command"));
  std::optional<VisualizationPhase> presentation_phase;
  bool presentation_include_maps = false;
  switch (command.kind) {
    case ExecutionCommandKind::kStage:
      if (!command.stage) {
        return Result<ExecutionReceipt>::Failure(
            Error::InvalidArgument("stage command requires a stage"));
      }
      result = coordinator_->ExecuteStage(base, *command.stage, context);
      if (!result && *command.stage == StageId::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base->revision);
      }
      if (!result && *command.stage == StageId::kAlignment) {
        visualization_projector_.RollbackAlignmentCandidate(base->revision);
      }
      if (result) {
        presentation_phase =
            *command.stage == StageId::kDataLoad
                ? VisualizationPhase::kDataLoad
            : *command.stage == StageId::kAlignment
                ? VisualizationPhase::kOptimization
            : *command.stage == StageId::kMapUpdate
                ? VisualizationPhase::kMapUpdate
                : VisualizationPhase::kSave;
        presentation_include_maps = *command.stage == StageId::kMapUpdate ||
                                    *command.stage == StageId::kSave;
      }
      break;
    case ExecutionCommandKind::kNode: {
      if (!command.node) {
        return Result<ExecutionReceipt>::Failure(
            Error::InvalidArgument("node command requires a node"));
      }
      auto target = ValidateNodeTarget(*base, *command.node, command.agent);
      if (!target) {
        return Result<ExecutionReceipt>::Failure(target.GetError());
      }
      result = coordinator_->ExecuteNode(base, *command.node, command.agent,
                                         context);
      if (!result && *command.node == NodeId::kDataLoad) {
        visualization_projector_.RollbackDataLoadCandidate(base->revision);
      }
      if (result) {
        presentation_phase =
            *command.node == NodeId::kDataLoad
                ? VisualizationPhase::kDataLoad
            : *command.node == NodeId::kLoopDetect
                ? VisualizationPhase::kLoopDetection
            : *command.node == NodeId::kOptimize
                ? VisualizationPhase::kOptimization
            : *command.node == NodeId::kMapUpdate
                ? VisualizationPhase::kMapUpdate
                : VisualizationPhase::kSave;
        presentation_include_maps =
            *command.node == NodeId::kMapUpdate ||
            *command.node == NodeId::kPoseSave ||
            *command.node == NodeId::kFallbackMapSave;
      }
      break;
    }
    case ExecutionCommandKind::kOptimizeThrough:
      if (!command.agent) {
        return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
            "optimizer replay command requires a target agent"));
      }
      result = coordinator_->ExecuteOptimizeThrough(base, *command.agent,
                                                    context);
      if (result) {
        presentation_phase = VisualizationPhase::kOptimization;
      }
      break;
    case ExecutionCommandKind::kReconfigure:
      if (!command.config_domain || command.config_revision == 0) {
        return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
            "reconfigure command requires a domain and revision"));
      }
      result = coordinator_->ExecuteReconfigure(
          base, *command.config_domain, command.config_revision, context);
      if (result) {
        presentation_phase = InferVisualizationPhase(*CommittedState());
      }
      break;
  }
  if (!result) return Result<ExecutionReceipt>::Failure(result.GetError());

  const auto after = Snapshot();
  if (after.revision <= base->revision) {
    return Result<ExecutionReceipt>::Failure(Error::InvalidArgument(
        "successful command did not advance the committed revision"));
  }
  const CommittedRuntimeSnapshot before{
      base->revision, base->config->revision, base->ordered_agents,
      base->artifacts};
  const auto excluded = command.kind == ExecutionCommandKind::kStage &&
                                command.stage == StageId::kAlignment
                            ? ExcludedAlignmentAgents(after)
                            : std::vector<AgentId>{};
  ExecutionReceipt receipt{
      base->revision, after.revision,
      ArtifactRevisionAffectedAgents(before, after), excluded,
      after.recovery_required};
  if (presentation_phase) {
    PublishVisualizationBestEffort(*presentation_phase,
                                   presentation_include_maps,
                                   base->revision);
  }
  return Result<ExecutionReceipt>::Ok(std::move(receipt));
}

Result<ConfigCommandReceipt> StageExecutor::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected,
    const ExecutionContext& context) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<ConfigCommandReceipt>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = EnsureMutationAllowed();
  if (!ready) {
    return Result<ConfigCommandReceipt>::Failure(ready.GetError());
  }
  if (!context.cancellation) {
    return Result<ConfigCommandReceipt>::Failure(
        Error::InvalidArgument("execution cancellation token is required"));
  }
  const auto base = CommittedState();
  if (context.base_revision != base->revision ||
      expected.runtime_revision != base->revision ||
      !base->config || expected.config_revision != base->config->revision) {
    return Result<ConfigCommandReceipt>::Failure(Error::InvalidArgument(
        "config transaction revision does not match committed runtime"));
  }
  auto applied = coordinator_->ApplyConfig(base, candidate, expected, context);
  if (!applied)
    return Result<ConfigCommandReceipt>::Failure(applied.GetError());
  PublishVisualizationBestEffort(
      InferVisualizationPhase(*CommittedState()), false, base->revision);
  return Result<ConfigCommandReceipt>::Ok(
      {std::move(applied).Value(), Snapshot().recovery_required});
}

Result<void> StageExecutor::InitializeRuntimeRevisions(
    uint64_t runtime_revision, uint64_t config_revision) {
  const auto committed = CommittedState();
  if (!committed || !committed->config) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime state is unavailable for rebase"));
  }
  if (runtime_revision == 0 || config_revision == 0) {
    return Result<void>::Failure(
        Error::InvalidArgument("runtime revisions must be non-zero"));
  }
  auto rebased = std::make_shared<RuntimeState>(*committed);
  auto config = std::make_shared<RuntimeConfig>(*committed->config);
  rebased->revision = runtime_revision;
  config->revision = config_revision;
  rebased->config = std::move(config);
  const auto recovery = runtime_state_store_.AuthoritySnapshot().recovery_required;
  runtime_state_store_.Initialize(std::move(rebased), recovery);
  PublishEmptyVisualization();
  return Result<void>::Ok();
}

void StageExecutor::RecordRecoveryRequired(
    std::shared_ptr<const Error> recovery_required) noexcept {
  runtime_state_store_.LatchRecoveryRequired(std::move(recovery_required));
}

}  // namespace open_lmm
