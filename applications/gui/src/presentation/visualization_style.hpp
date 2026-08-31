#pragma once

#include <open_lmm/common/visualization_snapshot.hpp>

namespace open_lmm {

inline constexpr float kVisualizationTrajectoryLineWidth = 1.0F;
inline constexpr float kVisualizationInterLoopLineWidth = 5.0F;
inline constexpr float kVisualizationIntraLoopLineWidth =
    kVisualizationInterLoopLineWidth;
inline constexpr int kDefaultVisualizationColorMode = 2;  // AGENT
struct VisualizationLayerPreferences {
  bool trajectory = true;
  bool pose_axes = true;
  bool points = false;
  bool intra_loops = false;
  bool inter_loops = false;
};

constexpr VisualizationLayerPreferences DefaultVisualizationPreferences(
    VisualizationPhase phase) {
  switch (phase) {
    case VisualizationPhase::kLoopDetection:
    case VisualizationPhase::kOptimization:
      return {true, true, true, true, true};
    case VisualizationPhase::kDataLoad:
    case VisualizationPhase::kMapUpdate:
    case VisualizationPhase::kSave:
      return {true, true, true, false, false};
  }
  return {};
}

}  // namespace open_lmm
