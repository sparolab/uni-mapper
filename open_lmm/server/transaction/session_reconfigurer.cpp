#include "session_reconfigurer.hpp"

#include <iomanip>
#include <sstream>
#include <utility>

#include <open_lmm/server/session_payload_builder.hpp>
#include <open_lmm/utils/config_schema.hpp>

namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

struct LoadedDocument {
  Config config;
  SessionConfigDocument document;
};

Result<LoadedDocument> LoadCandidate(ConfigDocumentKind kind,
                                     const SessionConfigDocument& source) {
  Config disk(source.path.string());
  if (!disk.is_valid()) {
    return Result<LoadedDocument>::Failure(
        Error::ParseError(disk.error_message()).WithConfig(
            source.path.string()));
  }
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
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

  if (domain == ConfigDomain::kLoopDetector) {
    auto loaded = LoadCandidate(ConfigDocumentKind::kLoopDetector,
                                documents->loop_detector);
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
                                documents->optimizer);
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
    auto loaded = LoadCandidate(ConfigDocumentKind::kDynamicRemover,
                                documents->dynamic_remover);
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
                                documents->map_server);
    if (!loaded) {
      return Result<SessionReconfigureCandidate>::Failure(loaded.GetError());
    }
    auto root_schema = BuiltinConfigSchemaRegistry().ParseAndValidate(
        ConfigDocumentKind::kRoot, documents->root.canonical_json,
        documents->root.path.string());
    auto map_schema = BuiltinConfigSchemaRegistry().ParseAndValidate(
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

  return Result<SessionReconfigureCandidate>::Ok(
      {std::move(next), std::move(payload)});
}

}  // namespace open_lmm
