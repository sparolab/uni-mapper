#pragma once

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>

#include <cstdint>
#include <map>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

enum class MapPresentationPhase : uint8_t { kEmpty, kVisible, kPending };

struct VisibleMapPresentation {
  std::string drawable;
  VisualizationPointKind point_kind =
      VisualizationPointKind::kFilteredScanPreview;
};

struct MapPresentationCommit {
  bool accepted = false;
  std::optional<std::string> replaced_drawable;
};

// GUI-only presentation state. It owns no CPU point payload and no GPU
// resource; it linearizes asynchronous request generations and drawable names
// so computation updates cannot clear the last usable presentation early.
class MapPresentationState {
 public:
  void Begin(const AgentId& agent, uint64_t generation);
  [[nodiscard]] bool IsCurrent(const AgentId& agent,
                               uint64_t generation) const;
  void FinishWithoutReplacement(const AgentId& agent, uint64_t generation);
  [[nodiscard]] MapPresentationCommit Commit(
      const AgentId& agent, uint64_t generation, std::string drawable,
      VisualizationPointKind point_kind);

  [[nodiscard]] std::optional<std::string> SetVisible(const AgentId& agent,
                                                       bool visible);
  [[nodiscard]] std::optional<std::string> DiscardVisible(
      const AgentId& agent);
  // Retires asynchronous work from the previous stage while preserving every
  // drawable that is currently visible.
  void CancelPending() noexcept;
  // Called only after a replacement presentation has been installed. The
  // returned drawable names remain owned by the viewer.
  [[nodiscard]] std::vector<std::string> DiscardAllVisible();
  [[nodiscard]] bool IsVisible(const AgentId& agent) const;
  [[nodiscard]] std::optional<VisibleMapPresentation> Visible(
      const AgentId& agent) const;
  [[nodiscard]] MapPresentationPhase Phase(const AgentId& agent) const;
  // Starts a new dataset epoch. Returned drawable names remain owned by the
  // viewer and must be removed by the caller.
  [[nodiscard]] std::vector<std::string> ResetEpoch();
 private:
  struct AgentState {
    bool visible = true;
    std::optional<uint64_t> pending_generation;
    std::optional<VisibleMapPresentation> presented;
  };
  std::map<AgentId, AgentState> agents_;
};

}  // namespace open_lmm
