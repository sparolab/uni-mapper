#pragma once

#include <open_lmm/common/visualization_snapshot.hpp>

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace open_lmm {

struct VisualizationUpdate {
  bool changed = false;
  std::vector<std::string> remove_drawables;
  std::vector<std::string> add_drawables;
};

class VisualizationRepository {
 public:
  [[nodiscard]] VisualizationUpdate Commit(
      std::shared_ptr<const VisualizationSnapshot> snapshot);
  [[nodiscard]] std::shared_ptr<const VisualizationSnapshot> Latest(
      const AgentId& agent) const;
  [[nodiscard]] std::vector<std::shared_ptr<const VisualizationSnapshot>>
  Snapshots() const;
  [[nodiscard]] std::size_t ApproximateBytes() const;

  [[nodiscard]] static std::string MapName(const AgentId& agent, uint64_t revision);
  [[nodiscard]] static std::string TrajectoryName(const AgentId& agent,
                                                  uint64_t revision);
  [[nodiscard]] static std::string PoseName(const AgentId& agent, std::size_t pose_index,
                                            uint64_t revision);
  [[nodiscard]] static std::string IntraLoopName(const AgentId& agent,
                                                 uint64_t revision);
  [[nodiscard]] static std::string InterLoopName(const AgentId& agent,
                                                 uint64_t revision);

 private:
  std::map<AgentId, std::shared_ptr<const VisualizationSnapshot>> snapshots_;
};

}  // namespace open_lmm
