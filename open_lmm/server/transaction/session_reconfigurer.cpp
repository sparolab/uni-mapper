#include "session_reconfigurer.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

#include <open_lmm/server/session_payload_builder.hpp>
#include <open_lmm/common/plugin_host_v2.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include <open_lmm/utils/plugin_schema_registry.hpp>

#include <dlfcn.h>

namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

struct LoadedDocument {
  Config config;
  SessionConfigDocument document;
};

Result<Config> LoadRawCandidate(const SessionConfigDocument& source) {
  std::error_code error;
  const auto bytes = std::filesystem::file_size(source.path, error);
  if (error || bytes > SchemaLimits{}.maximum_document_bytes) {
    return Result<Config>::Failure(Error::InvalidArgument(
        "candidate plugin config is missing or exceeds byte limit"));
  }
  Config disk(source.path.string());
  if (!disk.is_valid()) {
    return Result<Config>::Failure(
        Error::ParseError(disk.error_message()).WithConfig(source.path.string()));
  }
  return Result<Config>::Ok(std::move(disk));
}

bool HasCompleteV2Symbols(const std::string& library) {
  void* handle = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!handle) return false;
  const bool complete = dlsym(handle, OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2) &&
                        dlsym(handle, OPEN_LMM_PLUGIN_OPEN_SYMBOL_V2) &&
                        dlsym(handle, OPEN_LMM_PLUGIN_CALL_SYMBOL_V2) &&
                        dlsym(handle, OPEN_LMM_PLUGIN_CLOSE_SYMBOL_V2);
  dlclose(handle);
  return complete;
}

Result<bool> AdvertisesSchema(const std::string& library) {
  void* handle = dlopen(library.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!handle)
    return Result<bool>::Failure(
        Error::PluginLoadFailed("failed to inspect plugin: " + library));
  auto query = reinterpret_cast<open_lmm_plugin_query_fn_v2>(
      dlsym(handle, OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2));
  if (!query) {
    dlclose(handle);
    return Result<bool>::Failure(
        Error::PluginLoadFailed("plugin query symbol disappeared: " + library));
  }
  open_lmm_plugin_descriptor_v2 descriptor{};
  descriptor.struct_size = sizeof(descriptor);
  descriptor.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  descriptor.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  open_lmm_status_v2 status{};
  try {
    status = query(&descriptor);
  } catch (...) {
    dlclose(handle);
    return Result<bool>::Failure(
        Error::PluginLoadFailed("plugin query threw: " + library));
  }
  dlclose(handle);
  const uint32_t minimum = descriptor.abi_minor == 0
      ? OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_0_SIZE
      : OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_1_SIZE;
  if (status.struct_size < sizeof(status) ||
      status.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      status.code > OPEN_LMM_STATUS_HOST_ERROR_V2 ||
      status.message.struct_size < sizeof(status.message) ||
      status.message.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      (status.message.size != 0 && !status.message.data) ||
      status.message.size > 64U * 1024U ||
      status.code != OPEN_LMM_STATUS_OK_V2 ||
      descriptor.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      descriptor.struct_size < minimum ||
      descriptor.minimum_host_minor > OPEN_LMM_PLUGIN_ABI_V2_MINOR)
    return Result<bool>::Failure(
        Error::PluginLoadFailed("malformed plugin metadata: " + library));
  return Result<bool>::Ok(
      (descriptor.capability_bits &
       OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0);
}

Result<std::optional<PluginV2Metadata>> Discover(
    const Config& source, std::string_view section, std::string_view kind) {
  const std::string model = source.param<std::string>(
      std::string(section), "model", "");
  const std::string abi = source.param<std::string>(
      std::string(section), "plugin_abi", "auto");
  if (model.empty() || abi == "v1")
    return Result<std::optional<PluginV2Metadata>>::Ok(std::nullopt);
  const std::string library = "libcreate_" + model + ".so";
  if (!HasCompleteV2Symbols(library)) {
    if (abi == "v2")
      return Result<std::optional<PluginV2Metadata>>::Failure(
          Error::PluginLoadFailed("explicit ABI-v2 plugin unavailable: " +
                                  library));
    return Result<std::optional<PluginV2Metadata>>::Ok(std::nullopt);
  }
  auto schema = AdvertisesSchema(library);
  if (!schema)
    return Result<std::optional<PluginV2Metadata>>::Failure(schema.GetError());
  if (!schema.Value())
    return Result<std::optional<PluginV2Metadata>>::Ok(std::nullopt);
  auto loaded = LoadPluginV2(library, kind, source.ToJson());
  if (!loaded)
    return Result<std::optional<PluginV2Metadata>>::Failure(loaded.GetError());
  auto metadata = loaded.Value().Metadata();
  metadata.selected_model = model;
  return Result<std::optional<PluginV2Metadata>>::Ok(std::move(metadata));
}

Result<std::shared_ptr<const SchemaRegistry>> CandidateRegistry(
    const std::shared_ptr<const PluginV2Metadata>& descriptor,
    const std::shared_ptr<const PluginV2Metadata>& dynamic) {
  std::vector<PluginV2Metadata> plugins;
  if (descriptor) plugins.push_back(*descriptor);
  if (dynamic) plugins.push_back(*dynamic);
  auto built = BuildSessionSchemaRegistry(plugins);
  if (!built)
    return Result<std::shared_ptr<const SchemaRegistry>>::Failure(
        built.GetError());
  return Result<std::shared_ptr<const SchemaRegistry>>::Ok(
      std::make_shared<const SchemaRegistry>(std::move(built).Value()));
}

Result<void> ValidateCandidateDocuments(const SessionConfigDocuments& documents,
                                        const SchemaRegistry& registry) {
  const std::pair<ConfigDocumentKind, const SessionConfigDocument*> inputs[] = {
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
  auto session = ValidateSessionConfigDocuments(*root, *map);
  if (!session) return Result<void>::Failure(session.GetError());
  return Result<void>::Ok();
}

Result<LoadedDocument> LoadCandidate(ConfigDocumentKind kind,
                                     const SessionConfigDocument& source,
                                     const SchemaRegistry& registry) {
  Config disk(source.path.string());
  if (!disk.is_valid()) {
    return Result<LoadedDocument>::Failure(
        Error::ParseError(disk.error_message()).WithConfig(
            source.path.string()));
  }
  auto validated = registry.ParseAndValidate(
      kind, disk.ToJson(), source.path.string());
  if (!validated) {
    return Result<LoadedDocument>::Failure(validated.GetError());
  }
  const std::string canonical = validated.Value().CanonicalJson();
  return Result<LoadedDocument>::Ok(
      {Config::FromJson(canonical, source.path.string()),
       {source.path, canonical}});
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

void RefreshAlignmentIdentity(SessionConfig& config) {
  uint64_t config_hash = kFnvOffset;
  HashText(config_hash, config.documents->data_loader.canonical_json);
  HashText(config_hash, config.documents->loop_detector.canonical_json);
  HashText(config_hash, config.documents->optimizer.canonical_json);
  HashText(config_hash, std::to_string(config.root.anchor_agent_index));
  config.fingerprint = Hex(config_hash);

  auto alignment =
      std::make_shared<AlignmentArtifactMetadata>(*config.alignment_artifacts);
  uint64_t session_hash = kFnvOffset;
  HashText(session_hash, config.fingerprint);
  for (const auto& [agent, fingerprint] : alignment->input_fingerprints) {
    HashText(session_hash, agent.Value());
    HashText(session_hash, fingerprint);
  }
  alignment->session_fingerprint = Hex(session_hash);
  config.alignment_artifacts = std::move(alignment);
}

Result<std::shared_ptr<const SessionPayload>> ResetAlignmentPayload(
    const std::shared_ptr<const SessionState>& base,
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
  SessionPayloadBuilder builder(base->payload);
  return builder.SetContexts(std::move(contexts))
      .SetDatabase(std::move(database))
      .SetOptimizer(std::move(optimizer))
      .Build();
}

}  // namespace

SessionReconfigurer::SessionReconfigurer(
    std::shared_ptr<const AlgorithmFactory> algorithms)
    : algorithms_(algorithms ? std::move(algorithms)
                             : std::make_shared<AlgorithmFactory>()) {}

Result<SessionReconfigureCandidate> SessionReconfigurer::Prepare(
    const std::shared_ptr<const SessionState>& base, ConfigDomain domain,
    uint64_t revision) const {
  if (!base || !base->config || !base->config->documents ||
      !base->config->alignment_artifacts || !base->payload) {
    return Result<SessionReconfigureCandidate>::Failure(
        Error::InvalidArgument("reconfigure requires a complete config snapshot"));
  }
  if (revision <= base->config->revision) {
    return Result<SessionReconfigureCandidate>::Failure(
        Error::InvalidArgument("config revision must increase"));
  }
  auto next = std::make_shared<SessionConfig>(*base->config);
  auto documents =
      std::make_shared<SessionConfigDocuments>(*base->config->documents);
  next->documents = documents;
  next->revision = revision;
  std::shared_ptr<const SessionPayload> payload = base->payload;
  const SchemaRegistry& registry = base->config->schema_registry
      ? *base->config->schema_registry
      : BuiltinConfigSchemaRegistry();

  if (domain == ConfigDomain::kLoopDetector) {
    auto raw = LoadRawCandidate(documents->loop_detector);
    if (!raw)
      return Result<SessionReconfigureCandidate>::Failure(raw.GetError());
    auto discovered = Discover(raw.Value(), "loop_detector", "descriptor");
    if (!discovered)
      return Result<SessionReconfigureCandidate>::Failure(
          discovered.GetError());
    std::shared_ptr<const PluginV2Metadata> descriptor;
    if (discovered.Value()) {
      descriptor = std::make_shared<const PluginV2Metadata>(
          std::move(*discovered.Value()));
    }
    auto candidate_registry = CandidateRegistry(
        descriptor, base->config->remover_plugin_schema);
    if (!candidate_registry)
      return Result<SessionReconfigureCandidate>::Failure(
          candidate_registry.GetError());
    next->schema_registry = candidate_registry.Value();
    next->descriptor_plugin_schema = std::move(descriptor);
    auto loaded = LoadCandidate(ConfigDocumentKind::kLoopDetector,
                                documents->loop_detector,
                                *next->schema_registry);
    if (!loaded) {
      return Result<SessionReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = ParseLoopDetectorConfig(loaded.Value().config);
    if (!typed) {
      return Result<SessionReconfigureCandidate>::Failure(typed.GetError());
    }
    auto preflight = algorithms_->PreflightDescriptor(typed.Value());
    if (!preflight) {
      return Result<SessionReconfigureCandidate>::Failure(
          preflight.GetError());
    }
    auto optimizer = algorithms_->CreateOptimizer(*base->config->optimizer);
    if (!optimizer) {
      return Result<SessionReconfigureCandidate>::Failure(
          optimizer.GetError());
    }
    next->loop_detector = std::make_shared<const LoopDetectorConfig>(
        std::move(typed).Value());
    documents->loop_detector = std::move(loaded).Value().document;
    auto rebuilt = ResetAlignmentPayload(base, std::move(optimizer).Value(),
                                         true);
    if (!rebuilt) {
      return Result<SessionReconfigureCandidate>::Failure(
          rebuilt.GetError());
    }
    payload = std::move(rebuilt).Value();
    RefreshAlignmentIdentity(*next);
  } else if (domain == ConfigDomain::kOptimizer) {
    auto loaded = LoadCandidate(ConfigDocumentKind::kBackendOptimizer,
                                documents->optimizer, registry);
    if (!loaded) {
      return Result<SessionReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = ParseOptimizerConfig(loaded.Value().config);
    if (!typed) {
      return Result<SessionReconfigureCandidate>::Failure(typed.GetError());
    }
    auto optimizer = algorithms_->CreateOptimizer(typed.Value());
    if (!optimizer) {
      return Result<SessionReconfigureCandidate>::Failure(
          optimizer.GetError());
    }
    next->optimizer = std::make_shared<const OptimizerConfig>(
        std::move(typed).Value());
    documents->optimizer = std::move(loaded).Value().document;
    auto rebuilt = ResetAlignmentPayload(base, std::move(optimizer).Value(),
                                         false);
    if (!rebuilt) {
      return Result<SessionReconfigureCandidate>::Failure(
          rebuilt.GetError());
    }
    payload = std::move(rebuilt).Value();
    RefreshAlignmentIdentity(*next);
  } else if (domain == ConfigDomain::kDynamicRemover) {
    auto raw = LoadRawCandidate(documents->dynamic_remover);
    if (!raw)
      return Result<SessionReconfigureCandidate>::Failure(raw.GetError());
    auto discovered = Discover(raw.Value(), "dynamic_remover",
                               "dynamic_remover");
    if (!discovered)
      return Result<SessionReconfigureCandidate>::Failure(
          discovered.GetError());
    std::shared_ptr<const PluginV2Metadata> remover;
    if (discovered.Value()) {
      remover = std::make_shared<const PluginV2Metadata>(
          std::move(*discovered.Value()));
    }
    auto candidate_registry = CandidateRegistry(
        base->config->descriptor_plugin_schema, remover);
    if (!candidate_registry)
      return Result<SessionReconfigureCandidate>::Failure(
          candidate_registry.GetError());
    next->schema_registry = candidate_registry.Value();
    next->remover_plugin_schema = std::move(remover);
    auto loaded = LoadCandidate(ConfigDocumentKind::kDynamicRemover,
                                documents->dynamic_remover,
                                *next->schema_registry);
    if (!loaded) {
      return Result<SessionReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto typed = ParseDynamicRemoverConfig(loaded.Value().config);
    if (!typed) {
      return Result<SessionReconfigureCandidate>::Failure(typed.GetError());
    }
    auto preflight = algorithms_->PreflightRemover(typed.Value());
    if (!preflight) {
      return Result<SessionReconfigureCandidate>::Failure(
          preflight.GetError());
    }
    next->dynamic_remover = std::make_shared<const DynamicRemoverConfig>(
        std::move(typed).Value());
    documents->dynamic_remover = std::move(loaded).Value().document;
  } else if (domain == ConfigDomain::kMapSave) {
    auto loaded = LoadCandidate(ConfigDocumentKind::kMapServer,
                                documents->map_server, registry);
    if (!loaded) {
      return Result<SessionReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto root_schema = registry.ParseAndValidate(
        ConfigDocumentKind::kRoot, documents->root.canonical_json,
        documents->root.path.string());
    auto map_schema = registry.ParseAndValidate(
        ConfigDocumentKind::kMapServer,
        loaded.Value().document.canonical_json,
        loaded.Value().document.path.string());
    if (!root_schema) {
      return Result<SessionReconfigureCandidate>::Failure(
          root_schema.GetError());
    }
    if (!map_schema) {
      return Result<SessionReconfigureCandidate>::Failure(
          map_schema.GetError());
    }
    auto session_schema = ValidateSessionConfigDocuments(root_schema.Value(),
                                                          map_schema.Value());
    if (!session_schema) {
      return Result<SessionReconfigureCandidate>::Failure(
          session_schema.GetError());
    }
    auto typed = ParseMapSaveConfig(loaded.Value().config);
    if (!typed) {
      return Result<SessionReconfigureCandidate>::Failure(typed.GetError());
    }
    const int anchor = loaded.Value().config.param<int>(
        "map_server", "anchor_agent_index", 0);
    if (anchor != base->config->root.anchor_agent_index) {
      return Result<SessionReconfigureCandidate>::Failure(
          Error::InvalidArgument(
              "map_server/anchor_agent_index requires a new pipeline session"));
    }
    next->map_save = std::make_shared<const MapSaveConfig>(typed.Value());
    next->root.enable_map_updater = typed.Value().enable_map_updater;
    next->root.save_voxel_size = typed.Value().save_voxel_size;
    next->root.parallel_data_load = typed.Value().parallel_data_load;
    next->root.parallel_map_update = typed.Value().parallel_map_update;
    next->root.max_parallel_agents = typed.Value().max_parallel_agents;
    documents->map_server = std::move(loaded).Value().document;
  } else {
    return Result<SessionReconfigureCandidate>::Failure(
        Error::InvalidArgument(
            "data/global config requires a new pipeline session"));
  }

  if (domain == ConfigDomain::kLoopDetector ||
      domain == ConfigDomain::kDynamicRemover) {
    auto all_valid = ValidateCandidateDocuments(*documents,
                                                *next->schema_registry);
    if (!all_valid)
      return Result<SessionReconfigureCandidate>::Failure(
          all_valid.GetError());
  }

  return Result<SessionReconfigureCandidate>::Ok(
      {std::move(next), std::move(payload)});
}

}  // namespace open_lmm
