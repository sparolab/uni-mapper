#pragma once

#include <memory>
#include <mutex>

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <open_lmm/server/runtime_state.hpp>

namespace open_lmm {

// Read model derived exclusively from one immutable committed runtime state.
class VisualizationProjector {
 public:
  void Clear(uint64_t runtime_revision);
  void Publish(std::shared_ptr<const RuntimeState> runtime,
               bool include_maps);
  [[nodiscard]] Result<VisualizationSnapshot> Project(
      const AgentId& agent) const;

 private:
  struct State {
    uint64_t revision = 0;
    std::map<AgentId, VisualizationSnapshot> agents;
    std::map<AgentId, std::filesystem::path> map_paths;
  };

  mutable std::mutex mutex_;
  std::shared_ptr<const State> state_ = std::make_shared<State>();
};

}  // namespace open_lmm
