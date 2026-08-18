#pragma once

#include <open_lmm/common/agent_id.hpp>

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

enum class ConfigDomain : uint8_t {
  kGlobal,
  kDataLoader,
  kLoopDetector,
  kOptimizer,
  kDynamicRemover,
  kMapSave,
};

struct ExpectedRevision {
  uint64_t runtime_revision = 0;
  uint64_t config_revision = 0;
};

// One domain document candidate. selected_document is the root-config selector
// value for this domain; when absent, the committed document path is retained.
// The host derives and patches the root document itself, so callers cannot
// smuggle unrelated root changes into a domain-scoped transaction.
struct ConfigCandidate {
  ConfigDomain domain = ConfigDomain::kGlobal;
  std::string document_json;
  std::optional<std::filesystem::path> selected_document;
};

struct ConfigApplyReceipt {
  uint64_t previous_config_revision = 0;
  uint64_t config_revision = 0;
  uint64_t base_runtime_revision = 0;
  uint64_t runtime_revision = 0;
  std::vector<AgentId> affected_agents;
};

struct RuntimeReplaceReceipt {
  uint64_t previous_runtime_revision = 0;
  uint64_t previous_config_revision = 0;
  uint64_t runtime_revision = 0;
  uint64_t config_revision = 0;
};

}  // namespace open_lmm
