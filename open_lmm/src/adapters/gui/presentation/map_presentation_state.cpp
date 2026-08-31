#include <adapters/gui/presentation/map_presentation_state.hpp>

#include <utility>

namespace open_lmm {

void MapPresentationState::Begin(const AgentId& agent, uint64_t generation) {
  if (!agent.IsValid() || generation == 0) return;
  agents_[agent].pending_generation = generation;
}

bool MapPresentationState::IsCurrent(const AgentId& agent,
                                     uint64_t generation) const {
  const auto found = agents_.find(agent);
  return found != agents_.end() && found->second.pending_generation &&
         *found->second.pending_generation == generation;
}

void MapPresentationState::FinishWithoutReplacement(const AgentId& agent,
                                                     uint64_t generation) {
  const auto found = agents_.find(agent);
  if (found == agents_.end() || !found->second.pending_generation ||
      *found->second.pending_generation != generation) {
    return;
  }
  found->second.pending_generation.reset();
}

MapPresentationCommit MapPresentationState::Commit(
    const AgentId& agent, uint64_t generation, std::string drawable,
    VisualizationPointKind point_kind) {
  auto found = agents_.find(agent);
  if (found == agents_.end() || !found->second.pending_generation ||
      *found->second.pending_generation != generation || drawable.empty()) {
    return {};
  }
  auto& state = found->second;
  state.pending_generation.reset();
  MapPresentationCommit result{true, std::nullopt};
  if (!state.visible) return result;
  if (state.presented && state.presented->drawable != drawable) {
    result.replaced_drawable = state.presented->drawable;
  }
  state.presented = VisibleMapPresentation{std::move(drawable), point_kind};
  return result;
}

std::optional<std::string> MapPresentationState::SetVisible(
    const AgentId& agent, bool visible) {
  auto& state = agents_[agent];
  state.visible = visible;
  if (visible || !state.presented) return std::nullopt;
  auto removed = std::move(state.presented->drawable);
  state.presented.reset();
  return removed;
}

std::optional<std::string> MapPresentationState::DiscardVisible(
    const AgentId& agent) {
  const auto found = agents_.find(agent);
  if (found == agents_.end() || !found->second.presented) return std::nullopt;
  auto removed = std::move(found->second.presented->drawable);
  found->second.presented.reset();
  return removed;
}

void MapPresentationState::CancelPending() noexcept {
  for (auto& [agent, state] : agents_) {
    (void)agent;
    state.pending_generation.reset();
  }
}

std::vector<std::string> MapPresentationState::DiscardAllVisible() {
  std::vector<std::string> drawables;
  for (auto& [agent, state] : agents_) {
    (void)agent;
    if (!state.presented) continue;
    drawables.push_back(std::move(state.presented->drawable));
    state.presented.reset();
  }
  return drawables;
}

bool MapPresentationState::IsVisible(const AgentId& agent) const {
  const auto found = agents_.find(agent);
  return found == agents_.end() || found->second.visible;
}

std::optional<VisibleMapPresentation> MapPresentationState::Visible(
    const AgentId& agent) const {
  const auto found = agents_.find(agent);
  return found == agents_.end() ? std::nullopt : found->second.presented;
}

MapPresentationPhase MapPresentationState::Phase(const AgentId& agent) const {
  const auto found = agents_.find(agent);
  if (found == agents_.end()) return MapPresentationPhase::kEmpty;
  if (found->second.pending_generation) return MapPresentationPhase::kPending;
  return found->second.presented ? MapPresentationPhase::kVisible
                                 : MapPresentationPhase::kEmpty;
}

std::vector<std::string> MapPresentationState::ResetEpoch() {
  std::vector<std::string> drawables;
  for (auto& [agent, state] : agents_) {
    (void)agent;
    if (state.presented) {
      drawables.push_back(std::move(state.presented->drawable));
    }
  }
  agents_.clear();
  return drawables;
}

}  // namespace open_lmm
