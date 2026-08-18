#include "alignment_artifact_store.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <sstream>

#include <nlohmann/json.hpp>

#include <open_lmm/common/alignment_types.hpp>

namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;

void HashBytes(uint64_t& hash, const char* data, std::size_t size) {
  constexpr uint64_t kFnvPrime = 1099511628211ULL;
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= static_cast<unsigned char>(data[index]);
    hash *= kFnvPrime;
  }
}

void HashText(uint64_t& hash, const std::string& text) {
  HashBytes(hash, text.data(), text.size());
}

void HashFile(uint64_t& hash, const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  char buffer[8192];
  while (input && (input.read(buffer, sizeof(buffer)) || input.gcount() > 0)) {
    HashBytes(hash, buffer, static_cast<std::size_t>(input.gcount()));
  }
}

std::string Hex(uint64_t hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

std::optional<Eigen::Isometry3d> MatrixFromJson(const nlohmann::json& value) {
  if (!value.is_array() || value.size() != 16) return std::nullopt;
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      const auto& scalar = value[row * 4 + col];
      if (!scalar.is_number()) return std::nullopt;
      matrix(row, col) = scalar.get<double>();
    }
  }
  Eigen::Isometry3d transform(matrix);
  return ValidateRigidTransform(transform, "alignment artifact transform")
             ? std::optional<Eigen::Isometry3d>(transform)
             : std::nullopt;
}

nlohmann::json MatrixJson(const Eigen::Isometry3d& transform) {
  nlohmann::json values = nlohmann::json::array();
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      values.push_back(transform.matrix()(row, col));
    }
  }
  return values;
}

const char* MethodName(AlignmentMethod method) {
  switch (method) {
    case AlignmentMethod::kPending: return "pending";
    case AlignmentMethod::kKissMatcher: return "kiss_matcher";
    case AlignmentMethod::kDescriptor: return "descriptor";
    case AlignmentMethod::kManual: return "manual";
  }
  return "unknown";
}

std::optional<AlignmentMethod> ParseMethod(const std::string& method) {
  if (method == "manual") return AlignmentMethod::kManual;
  if (method == "descriptor") return AlignmentMethod::kDescriptor;
  if (method == "kiss_matcher") return AlignmentMethod::kKissMatcher;
  return std::nullopt;
}

}  // namespace

Result<AlignmentArtifactStore> AlignmentArtifactStore::Open(
    const AlignmentArtifactIdentityInput& input,
    AgentSymbolCatalogHandle catalog) {
  if (!catalog || input.agents.size() != input.data_directories.size()) {
    return Result<AlignmentArtifactStore>::Failure(Error::InvalidArgument(
        "alignment artifact identity requires matching agents and inputs"));
  }
  AlignmentArtifactStore store;
  uint64_t config_hash = kFnvOffset;
  HashText(config_hash, input.data_loader_config);
  HashText(config_hash, input.loop_detector_config);
  HashText(config_hash, input.optimizer_config);
  HashText(config_hash, std::to_string(input.anchor_agent_index));
  store.identity_.config_fingerprint = Hex(config_hash);

  for (std::size_t index = 0; index < input.agents.size(); ++index) {
    uint64_t hash = kFnvOffset;
    const auto& directory = input.data_directories[index];
    HashText(hash, directory.string());
    HashFile(hash, directory / input.pose_file_name);
    std::vector<std::filesystem::path> scans;
    std::error_code error;
    std::filesystem::directory_iterator iterator(
        directory / input.scan_directory_name, error);
    for (const std::filesystem::directory_iterator end;
         !error && iterator != end; iterator.increment(error)) {
      if (iterator->is_regular_file(error) &&
          iterator->path().extension() == "." + input.scan_extension) {
        scans.push_back(iterator->path());
      }
    }
    std::sort(scans.begin(), scans.end());
    for (const auto& scan : scans) {
      HashText(hash, scan.filename().string());
      error.clear();
      const auto size = std::filesystem::file_size(scan, error);
      if (!error) HashText(hash, std::to_string(size));
      error.clear();
      const auto modified = std::filesystem::last_write_time(scan, error);
      if (!error) {
        HashText(hash,
                 std::to_string(modified.time_since_epoch().count()));
      }
    }
    store.identity_.input_fingerprints[input.agents[index]] = Hex(hash);
  }
  uint64_t runtime_hash = kFnvOffset;
  HashText(runtime_hash, store.identity_.config_fingerprint);
  for (const auto& [agent, fingerprint] :
       store.identity_.input_fingerprints) {
    HashText(runtime_hash, agent.Value());
    HashText(runtime_hash, fingerprint);
  }
  store.identity_.runtime_fingerprint = Hex(runtime_hash);
  store.identity_.cache_path = input.cache_root / "map_alignment_cache.json";

  std::ifstream cache(store.identity_.cache_path);
  if (!cache) return Result<AlignmentArtifactStore>::Ok(std::move(store));
  try {
    nlohmann::json root;
    cache >> root;
    if (root.value("version", 0) != 3 ||
        root.value("runtime_fingerprint", std::string()) !=
            store.identity_.runtime_fingerprint) {
      return Result<AlignmentArtifactStore>::Ok(std::move(store));
    }
    for (const auto& item : root.at("alignments")) {
      if (item.value("approval", std::string()) != "user") continue;
      auto source = AgentId::Parse(item.value("source_agent", std::string()));
      auto target = AgentId::Parse(item.value("target_agent", std::string()));
      auto method = ParseMethod(item.value("method", std::string()));
      auto transform = MatrixFromJson(item["accepted_global_T_agent"]);
      if (!source || !target || !method || !transform ||
          !catalog->SymbolFor(source.Value()) ||
          !catalog->SymbolFor(target.Value())) {
        continue;
      }
      StoredAlignment stored;
      stored.proposal.source_agent = source.Value();
      stored.proposal.target_agent = target.Value();
      stored.proposal.method = *method;
      stored.proposal.target_T_source = *transform;
      if (item.contains("metrics")) {
        const auto& metrics = item["metrics"];
        stored.proposal.metrics.correspondence_count =
            metrics.value("correspondence_count", 0UL);
        stored.proposal.metrics.rotation_inliers =
            metrics.value("rotation_inliers", 0UL);
        stored.proposal.metrics.final_inliers =
            metrics.value("final_inliers", 0UL);
        stored.proposal.metrics.consensus_size =
            metrics.value("consensus_size", 0UL);
        if (metrics.contains("fitness")) {
          stored.proposal.metrics.fitness = metrics["fitness"].get<double>();
        }
        if (metrics.contains("overlap_ratio")) {
          stored.proposal.metrics.overlap_ratio =
              metrics["overlap_ratio"].get<double>();
        }
      }
      stored.approval = AlignmentApproval::kUser;
      stored.accepted_at_unix_ms = item.value("accepted_at_unix_ms", 0ULL);
      store.cached_[source.Value()] = std::move(stored);
    }
  } catch (const std::exception&) {
    store.cached_.clear();
  }
  return Result<AlignmentArtifactStore>::Ok(std::move(store));
}

Result<AlignmentArtifactStore> AlignmentArtifactStore::FromCommitted(
    const RuntimeState& committed) {
  if (!committed.config || !committed.config->alignment_artifacts ||
      !committed.payload || !committed.payload->database) {
    return Result<AlignmentArtifactStore>::Failure(Error::InvalidArgument(
        "committed runtime has no alignment artifact metadata"));
  }
  AlignmentArtifactStore store;
  store.identity_.config_fingerprint = committed.config->fingerprint;
  store.identity_.cache_path =
      committed.config->alignment_artifacts->cache_path;
  store.identity_.input_fingerprints =
      committed.config->alignment_artifacts->input_fingerprints;
  store.identity_.runtime_fingerprint =
      committed.config->alignment_artifacts->runtime_fingerprint;
  store.cached_ = committed.payload->database->stored_alignments;
  return Result<AlignmentArtifactStore>::Ok(std::move(store));
}

const AlignmentArtifactIdentity& AlignmentArtifactStore::Identity() const {
  return identity_;
}

const std::map<AgentId, StoredAlignment>& AlignmentArtifactStore::Cached()
    const {
  return cached_;
}

void AlignmentArtifactStore::InstallInto(SharedDatabase& database) const {
  database.stored_alignments = cached_;
}

Result<void> AlignmentArtifactStore::Prepare(
    const RuntimeState& state,
    const std::filesystem::path& output_directory,
    PendingOutputSet& pending, ArtifactRepository& artifacts) const {
  if (!state.payload || !state.payload->database) {
    return Result<void>::Failure(
        Error::InvalidArgument("alignment artifact has no runtime payload"));
  }
  nlohmann::json root;
  root["version"] = 3;
  root["transform_convention"] = "global_T_agent";
  root["generated_at_unix_ms"] = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  root["config_fingerprint"] = identity_.config_fingerprint;
  root["runtime_fingerprint"] = identity_.runtime_fingerprint;
  root["input_fingerprints"] = nlohmann::json::object();
  for (const auto& [agent, fingerprint] : identity_.input_fingerprints) {
    root["input_fingerprints"][agent.Value()] = fingerprint;
  }
  root["alignments"] = nlohmann::json::array();
  bool has_user_approval = false;
  for (const auto& context : state.payload->contexts) {
    if (!context.loop_output ||
        !context.loop_output->accepted_global_T_agent ||
        !context.loop_output->accepted_alignment_method ||
        !context.loop_output->accepted_alignment_approval) {
      return Result<void>::Failure(Error::IoError(
          "cannot save incomplete alignment artifact for agent " +
          context.agent.id.Value()));
    }
    const auto& output = *context.loop_output;
    nlohmann::json item;
    item["agent"] = context.agent.id.Value();
    item["source_agent"] = context.agent.id.Value();
    item["target_agent"] = output.accepted_target_agent.Value();
    item["method"] = MethodName(*output.accepted_alignment_method);
    item["approval"] = *output.accepted_alignment_approval ==
                               AlignmentApproval::kUser
                           ? "user"
                           : "automatic";
    item["accepted_at_unix_ms"] = output.accepted_at_unix_ms;
    item["accepted_global_T_agent"] =
        MatrixJson(*output.accepted_global_T_agent);
    const auto optimized = state.payload->database->descriptor_store
                               .aligned_maps.find(context.agent.id);
    if (optimized != state.payload->database->descriptor_store
                         .aligned_maps.end()) {
      item["optimized_global_T_agent"] =
          MatrixJson(optimized->second.global_T_agent);
      item["map_revision"] = optimized->second.revision;
    }
    const auto& metrics = output.accepted_alignment_metrics;
    item["metrics"] = {{"correspondence_count", metrics.correspondence_count},
                       {"rotation_inliers", metrics.rotation_inliers},
                       {"final_inliers", metrics.final_inliers},
                       {"consensus_size", metrics.consensus_size}};
    if (metrics.fitness) item["metrics"]["fitness"] = *metrics.fitness;
    if (metrics.overlap_ratio) {
      item["metrics"]["overlap_ratio"] = *metrics.overlap_ratio;
    }
    has_user_approval |= *output.accepted_alignment_approval ==
                         AlignmentApproval::kUser;
    root["alignments"].push_back(std::move(item));
  }

  const auto destination = output_directory / "map_alignments.json";
  const auto temporary = std::filesystem::path(destination.string() + ".tmp");
  pending.Add(temporary, destination);
  std::ofstream output(temporary);
  if (!output || !(output << root.dump(2) << '\n')) {
    return Result<void>::Failure(Error::IoError(
        "failed to write alignment artifact " + temporary.string()));
  }
  output.close();
  uint64_t artifact_hash = kFnvOffset;
  HashText(artifact_hash, root.dump());
  for (const AgentId& agent : state.ordered_agents) {
    artifacts.RecordExternalFile(ArtifactType::kMapAlignment, agent,
                                 destination.string(), Hex(artifact_hash));
  }
  if (has_user_approval) {
    const auto cache_temporary =
        std::filesystem::path(identity_.cache_path.string() + ".tmp");
    pending.Add(cache_temporary, identity_.cache_path);
    std::ofstream cache(cache_temporary);
    if (!cache || !(cache << root.dump(2) << '\n')) {
      return Result<void>::Failure(Error::IoError(
          "failed to write alignment cache " + cache_temporary.string()));
    }
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
