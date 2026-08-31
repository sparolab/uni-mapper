#include "runtime_reconfigurer.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

#include <runtime/state/runtime_payload_builder.hpp>
#include <config/document/config.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include <foundation/logging/logging.hpp>

namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

struct LoadedDocument {
  RuntimeConfigDocument document;
  ValidatedConfigDocument validated;
};

Result<Config> LoadCandidateSnapshot(const RuntimeConfigDocument& source) {
  return LoadConfigFileBounded(source.path,
                               SchemaLimits{}.maximum_document_bytes);
}

const char* SelectorFor(ConfigDomain domain) {
  switch (domain) {
    case ConfigDomain::kLoopDetector: return "config_loop_detector";
    case ConfigDomain::kOptimizer: return "config_backend_optimizer";
    case ConfigDomain::kDynamicRemover: return "config_dynamic_remover";
    case ConfigDomain::kMapSave: return "config_map_server";
    case ConfigDomain::kGlobal:
    case ConfigDomain::kDataLoader: return nullptr;
  }
  return nullptr;
}

RuntimeConfigDocument* DocumentFor(RuntimeConfigDocuments& documents,
                                   ConfigDomain domain) {
  switch (domain) {
    case ConfigDomain::kLoopDetector: return &documents.loop_detector;
    case ConfigDomain::kOptimizer: return &documents.optimizer;
    case ConfigDomain::kDynamicRemover: return &documents.dynamic_remover;
    case ConfigDomain::kMapSave: return &documents.map_server;
    case ConfigDomain::kGlobal:
    case ConfigDomain::kDataLoader: return nullptr;
  }
  return nullptr;
}

Result<Config> CandidateSnapshot(const ConfigCandidate* candidate,
                                 const RuntimeConfigDocument& source) {
  if (!candidate) return LoadCandidateSnapshot(source);
  if (candidate->document_json.empty() ||
      candidate->document_json.size() >
          SchemaLimits{}.maximum_document_bytes) {
    return Result<Config>::Failure(
        Error::InvalidArgument("candidate config is empty or exceeds byte limit")
            .WithConfig(source.path.string()));
  }
  Config parsed = Config::FromJson(candidate->document_json,
                                   source.path.string() + ":candidate");
  if (!parsed.is_valid()) {
    return Result<Config>::Failure(
        Error::ParseError(parsed.error_message()).WithConfig(
            source.path.string()));
  }
  return Result<Config>::Ok(std::move(parsed));
}

Result<void> SelectCandidateDocument(
    RuntimeConfigDocuments& documents, const ConfigCandidate& candidate,
    const SchemaRegistry& registry) {
  auto* document = DocumentFor(documents, candidate.domain);
  const char* selector = SelectorFor(candidate.domain);
  if (!document || !selector) {
    return Result<void>::Failure(Error::InvalidArgument(
        "data/global config requires a new runtime replacement"));
  }
  if (!candidate.selected_document) return Result<void>::Ok();
  if (candidate.selected_document->empty()) {
    return Result<void>::Failure(
        Error::InvalidArgument("selected config document must be non-empty"));
  }

  const fs::path config_directory = documents.root.path.parent_path();
  fs::path selected = *candidate.selected_document;
  if (selected.is_relative()) selected = config_directory / selected;
  selected = selected.lexically_normal();
  std::error_code error;
  selected = fs::weakly_canonical(selected, error);
  if (error) {
    return Result<void>::Failure(
        Error::IoError("failed to resolve selected config document: " +
                       error.message()));
  }
  if (selected == documents.root.path) {
    return Result<void>::Failure(Error::InvalidArgument(
        "domain config document must not replace the root config"));
  }
  for (const RuntimeConfigDocument* other :
       {&documents.map_server, &documents.data_loader,
        &documents.loop_detector, &documents.optimizer,
        &documents.dynamic_remover}) {
    if (other != document && selected == other->path) {
      return Result<void>::Failure(Error::InvalidArgument(
          "domain config document must not alias another module document"));
    }
  }
  const bool exists = fs::exists(selected, error);
  if (error || (exists && !fs::is_regular_file(selected, error))) {
    return Result<void>::Failure(Error::InvalidArgument(
        "selected config document must be a regular file or new file"));
  }
  if (!fs::is_directory(selected.parent_path(), error) || error) {
    return Result<void>::Failure(Error::InvalidArgument(
        "selected config document parent must be an existing directory"));
  }

  const fs::path canonical_config_directory =
      fs::weakly_canonical(config_directory, error);
  if (error) {
    return Result<void>::Failure(Error::IoError(
        "failed to resolve config directory for path policy: " +
        error.message()));
  }
  const fs::path relative = selected.lexically_relative(canonical_config_directory);
  const bool external = relative.empty() ||
      (!relative.empty() && *relative.begin() == "..");
  if (external) {
    LogWarning(
        "[config/path_policy] external mutable module config is deprecated "
        "and remains enabled for compatibility: " + selected.string());
  }

  auto root_json = nlohmann::json::parse(documents.root.canonical_json);
  root_json.at("global").at(selector) = candidate.selected_document->string();
  auto validated = registry.Validate(ConfigDocumentKind::kRoot, root_json,
                                     documents.root.path.string());
  if (!validated) return Result<void>::Failure(validated.GetError());
  documents.root.canonical_json = validated.Value().CanonicalJson();
  document->path = std::move(selected);
  return Result<void>::Ok();
}

Result<void> ValidateCandidateDocuments(const RuntimeConfigDocuments& documents,
                                        const SchemaRegistry& registry) {
  const std::pair<ConfigDocumentKind, const RuntimeConfigDocument*> inputs[] = {
      {ConfigDocumentKind::kRoot, &documents.root},
      {ConfigDocumentKind::kMapServer, &documents.map_server},
      {ConfigDocumentKind::kDataLoader, &documents.data_loader},
      {ConfigDocumentKind::kLoopDetector, &documents.loop_detector},
      {ConfigDocumentKind::kBackendOptimizer, &documents.optimizer},
      {ConfigDocumentKind::kDynamicRemover, &documents.dynamic_remover},
  };
  std::optional<ValidatedConfigDocument> root;
  std::optional<ValidatedConfigDocument> map;
  for (const auto& [kind, document] : inputs) {
    auto validated = registry.ParseAndValidate(
        kind, document->canonical_json, document->path.string());
    if (!validated) return Result<void>::Failure(validated.GetError());
    if (kind == ConfigDocumentKind::kRoot)
      root = std::move(validated).Value();
    else if (kind == ConfigDocumentKind::kMapServer)
      map = std::move(validated).Value();
  }
  auto runtime = ValidateRuntimeConfigDocuments(*root, *map);
  if (!runtime) return Result<void>::Failure(runtime.GetError());
  return Result<void>::Ok();
}

Result<LoadedDocument> LoadCandidate(ConfigDocumentKind kind,
                                     const RuntimeConfigDocument& source,
                                     const Config& snapshot,
                                     const SchemaRegistry& registry) {
  auto validated = registry.ParseAndValidate(
      kind, snapshot.ToJson(), source.path.string());
  if (!validated) {
    return Result<LoadedDocument>::Failure(validated.GetError());
  }
  auto validated_document = std::move(validated).Value();
  const std::string canonical = validated_document.CanonicalJson();
  return Result<LoadedDocument>::Ok(
      {{source.path, canonical}, std::move(validated_document)});
}

void HashText(uint64_t& hash, const std::string& text) {
  for (unsigned char value : text) {
    hash ^= value;
    hash *= kFnvPrime;
  }
}

std::string Hex(uint64_t hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

void RefreshAlignmentIdentity(RuntimeConfig& config) {
  uint64_t config_hash = kFnvOffset;
  HashText(config_hash, config.documents->data_loader.canonical_json);
  HashText(config_hash, config.documents->loop_detector.canonical_json);
  HashText(config_hash, config.documents->optimizer.canonical_json);
  HashText(config_hash, std::to_string(config.root.anchor_agent_index));
  config.fingerprint = Hex(config_hash);

  auto alignment =
      std::make_shared<AlignmentArtifactMetadata>(*config.alignment_artifacts);
  uint64_t runtime_hash = kFnvOffset;
  HashText(runtime_hash, config.fingerprint);
  for (const auto& [agent, fingerprint] : alignment->input_fingerprints) {
    HashText(runtime_hash, agent.Value());
    HashText(runtime_hash, fingerprint);
  }
  alignment->runtime_fingerprint = Hex(runtime_hash);
  config.alignment_artifacts = std::move(alignment);
}

Result<std::shared_ptr<const RuntimePayload>> ResetAlignmentPayload(
    const std::shared_ptr<const RuntimeState>& base,
    std::shared_ptr<BackendOptimizerBase> optimizer,
  bool reset_loops) {
  auto contexts = base->payload->contexts;
  auto database = reset_loops
                      ? std::make_shared<SharedDatabase>()
                      : std::make_shared<SharedDatabase>(
                            *base->payload->database);
  database->raw_data = base->payload->database->raw_data;
  database->optimized_data.clear();
  database->stored_alignments.clear();
  database->alignment_feedback.reset();
  if (reset_loops) {
    for (auto& context : contexts) context.loop_output.reset();
  }
  RuntimePayloadBuilder builder(base->payload);
  return builder.SetContexts(std::move(contexts))
      .SetDatabase(std::move(database))
      .SetOptimizer(std::move(optimizer))
      .Build();
}

}  // namespace

RuntimeReconfigurer::RuntimeReconfigurer(
    std::shared_ptr<const AlgorithmProvider> algorithms)
    : algorithms_(std::move(algorithms)) {}

Result<RuntimeReconfigureCandidate> RuntimeReconfigurer::Prepare(
    const std::shared_ptr<const RuntimeState>& base, ConfigDomain domain,
    uint64_t revision) const {
  return PrepareImpl(base, domain, revision, nullptr);
}

Result<RuntimeReconfigureCandidate> RuntimeReconfigurer::Prepare(
    const std::shared_ptr<const RuntimeState>& base,
    const ConfigCandidate& candidate, uint64_t revision) const {
  return PrepareImpl(base, candidate.domain, revision, &candidate);
}

Result<RuntimeReconfigureCandidate> RuntimeReconfigurer::PrepareImpl(
    const std::shared_ptr<const RuntimeState>& base, ConfigDomain domain,
    uint64_t revision, const ConfigCandidate* candidate) const {
  if (!base || !base->config || !base->config->documents ||
      !base->config->alignment_artifacts || !base->payload) {
    return Result<RuntimeReconfigureCandidate>::Failure(
        Error::InvalidArgument("reconfigure requires a complete config snapshot"));
  }
  if (revision <= base->config->revision) {
    return Result<RuntimeReconfigureCandidate>::Failure(
        Error::InvalidArgument("config revision must increase"));
  }
  auto next = std::make_shared<RuntimeConfig>(*base->config);
  auto documents =
      std::make_shared<RuntimeConfigDocuments>(*base->config->documents);
  next->documents = documents;
  next->revision = revision;
  std::shared_ptr<const RuntimePayload> payload = base->payload;
  const SchemaRegistry& registry = base->config->schema_registry
      ? *base->config->schema_registry
      : BuiltinConfigSchemaRegistry();
  if (candidate) {
    auto selected =
        SelectCandidateDocument(*documents, *candidate, registry);
    if (!selected)
      return Result<RuntimeReconfigureCandidate>::Failure(
          selected.GetError());
  }

  if (domain == ConfigDomain::kLoopDetector) {
    auto raw = CandidateSnapshot(candidate, documents->loop_detector);
    if (!raw)
      return Result<RuntimeReconfigureCandidate>::Failure(raw.GetError());
    auto loaded = LoadCandidate(ConfigDocumentKind::kLoopDetector,
                                documents->loop_detector,
                                raw.Value(),
                                registry);
    if (!loaded) {
      return Result<RuntimeReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = DecodeLoopDetectorConfig(loaded.Value().validated);
    if (!typed) {
      return Result<RuntimeReconfigureCandidate>::Failure(typed.GetError());
    }
    auto preflight = algorithms_->PreflightDescriptor(typed.Value());
    if (!preflight) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          preflight.GetError());
    }
    auto optimizer = algorithms_->CreateOptimizer(*base->config->optimizer);
    if (!optimizer) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          optimizer.GetError());
    }
    next->loop_detector = std::make_shared<const LoopDetectorConfig>(
        std::move(typed).Value());
    documents->loop_detector = std::move(loaded).Value().document;
    auto rebuilt = ResetAlignmentPayload(base, std::move(optimizer).Value(),
                                         true);
    if (!rebuilt) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          rebuilt.GetError());
    }
    payload = std::move(rebuilt).Value();
    RefreshAlignmentIdentity(*next);
  } else if (domain == ConfigDomain::kOptimizer) {
    auto raw = CandidateSnapshot(candidate, documents->optimizer);
    if (!raw)
      return Result<RuntimeReconfigureCandidate>::Failure(raw.GetError());
    auto loaded = LoadCandidate(ConfigDocumentKind::kBackendOptimizer,
                                documents->optimizer, raw.Value(), registry);
    if (!loaded) {
      return Result<RuntimeReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = DecodeOptimizerConfig(loaded.Value().validated);
    if (!typed) {
      return Result<RuntimeReconfigureCandidate>::Failure(typed.GetError());
    }
    auto optimizer = algorithms_->CreateOptimizer(typed.Value());
    if (!optimizer) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          optimizer.GetError());
    }
    next->optimizer = std::make_shared<const OptimizerConfig>(
        std::move(typed).Value());
    documents->optimizer = std::move(loaded).Value().document;
    auto rebuilt = ResetAlignmentPayload(base, std::move(optimizer).Value(),
                                         false);
    if (!rebuilt) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          rebuilt.GetError());
    }
    payload = std::move(rebuilt).Value();
    RefreshAlignmentIdentity(*next);
  } else if (domain == ConfigDomain::kDynamicRemover) {
    auto raw = CandidateSnapshot(candidate, documents->dynamic_remover);
    if (!raw)
      return Result<RuntimeReconfigureCandidate>::Failure(raw.GetError());
    auto loaded = LoadCandidate(ConfigDocumentKind::kDynamicRemover,
                                documents->dynamic_remover,
                                raw.Value(),
                                registry);
    if (!loaded) {
      return Result<RuntimeReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = DecodeDynamicRemoverConfig(loaded.Value().validated);
    if (!typed) {
      return Result<RuntimeReconfigureCandidate>::Failure(typed.GetError());
    }
    auto preflight = algorithms_->PreflightRemover(typed.Value());
    if (!preflight) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          preflight.GetError());
    }
    next->dynamic_remover = std::make_shared<const DynamicRemoverConfig>(
        std::move(typed).Value());
    documents->dynamic_remover = std::move(loaded).Value().document;
  } else if (domain == ConfigDomain::kMapSave) {
    auto raw = CandidateSnapshot(candidate, documents->map_server);
    if (!raw)
      return Result<RuntimeReconfigureCandidate>::Failure(raw.GetError());
    auto loaded = LoadCandidate(ConfigDocumentKind::kMapServer,
                                documents->map_server, raw.Value(), registry);
    if (!loaded) {
      return Result<RuntimeReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto root_schema = registry.ParseAndValidate(
        ConfigDocumentKind::kRoot, documents->root.canonical_json,
        documents->root.path.string());
    auto map_schema = registry.ParseAndValidate(
        ConfigDocumentKind::kMapServer,
        loaded.Value().document.canonical_json,
        loaded.Value().document.path.string());
    if (!root_schema) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          root_schema.GetError());
    }
    if (!map_schema) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          map_schema.GetError());
    }
    auto runtime_schema = ValidateRuntimeConfigDocuments(root_schema.Value(),
                                                          map_schema.Value());
    if (!runtime_schema) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          runtime_schema.GetError());
    }
    auto typed = DecodeMapSaveConfig(loaded.Value().validated);
    if (!typed) {
      return Result<RuntimeReconfigureCandidate>::Failure(typed.GetError());
    }
    const int anchor = loaded.Value().validated.Document()
                           .at("map_server")
                           .at("anchor_agent_index")
                           .get<int>();
    if (anchor != base->config->root.anchor_agent_index) {
      return Result<RuntimeReconfigureCandidate>::Failure(
          Error::InvalidArgument(
              "map_server/anchor_agent_index requires a new runtime replacement"));
    }
    next->map_save = std::make_shared<const MapSaveConfig>(typed.Value());
    next->root.enable_map_updater = typed.Value().enable_map_updater;
    next->root.save_voxel_size = typed.Value().save_voxel_size;
    next->root.parallel_data_load = typed.Value().parallel_data_load;
    next->root.parallel_map_update = typed.Value().parallel_map_update;
    next->root.max_parallel_agents = typed.Value().max_parallel_agents;
    documents->map_server = std::move(loaded).Value().document;
  } else {
    return Result<RuntimeReconfigureCandidate>::Failure(
        Error::InvalidArgument(
            "data/global config requires a new runtime replacement"));
  }

  if (domain == ConfigDomain::kLoopDetector ||
      domain == ConfigDomain::kDynamicRemover) {
    auto all_valid = ValidateCandidateDocuments(*documents,
                                                *next->schema_registry);
    if (!all_valid)
      return Result<RuntimeReconfigureCandidate>::Failure(
          all_valid.GetError());
  }

  return Result<RuntimeReconfigureCandidate>::Ok(
      {std::move(next), std::move(payload)});
}

}  // namespace open_lmm
