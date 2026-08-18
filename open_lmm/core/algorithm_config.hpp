#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>

#include <open_lmm/common/result.hpp>
#include <open_lmm/utils/config_schema.hpp>

namespace open_lmm {

struct DataLoaderConfig {
  std::string type;
  std::string pose_file_name;
  std::string pose_format;
  std::string scan_type;
  std::string scan_dir_name;
  Eigen::Isometry3d extrinsic = Eigen::Isometry3d::Identity();
  float voxel_size = 0.1F;
  float min_range = 0.0F;
  float max_range = 100.0F;
  std::string delimiter = " ";
  bool show_progress = true;
};

struct LoopDetectorConfig {
  std::string type;
  std::size_t num_candidates = 5;
  double distance_threshold = 0.13;
  std::size_t kdtree_rebuild_threshold = 50;
  std::string model;
  double pcm_translation_threshold = 10.0;
  double pcm_rotation_threshold_deg = 20.0;
  std::string pcm_solver = "heuristic";
  int pcm_threads = 1;
  std::size_t pcm_max_candidates = 0;
  float kiss_voxel_size = 2.0F;
  bool kiss_use_quatro = false;
  float pose_nn_distance_threshold = 10.0F;
  float inter_loop_keyframe_spacing_m = 10.0F;
  std::string feedback_mode = "adaptive";
  std::string headless_policy = "kiss_then_descriptor";
  int feedback_timeout_sec = 0;
  std::string plugin_config_json;
};

struct OptimizerConfig {
  std::string type;
  double relinearize_threshold = 0.1;
  int relinearize_skip = 1;
  int isam_extra_updates = 5;
  int min_loop_frame_gap = 30;
  int icp_search_num = 3;
};

enum class PluginThreadSafety : uint8_t {
  kSingleThreadOnly,
  kInstanceIsolatedParallel,
  kReentrantSharedInstance,
};

struct DynamicRemoverConfig {
  std::string type;
  std::string model;
  std::string plugin_config_json;
  PluginThreadSafety thread_safety = PluginThreadSafety::kSingleThreadOnly;
  std::size_t internal_cpu_threads = 1;
};

struct MapSaveConfig {
  bool enable_map_updater = false;
  double save_voxel_size = 0.2;
  bool parallel_data_load = false;
  bool parallel_map_update = false;
  std::size_t max_parallel_agents = 1;
};

// Typed decoding accepts only schema-validated, defaulted and migrated input.
// These functions convert representation; schema and preflight own policy.
Result<DataLoaderConfig> DecodeDataLoaderConfig(
    const ValidatedConfigDocument& document);
Result<LoopDetectorConfig> DecodeLoopDetectorConfig(
    const ValidatedConfigDocument& document);
Result<OptimizerConfig> DecodeOptimizerConfig(
    const ValidatedConfigDocument& document);
Result<DynamicRemoverConfig> DecodeDynamicRemoverConfig(
    const ValidatedConfigDocument& document);
Result<MapSaveConfig> DecodeMapSaveConfig(
    const ValidatedConfigDocument& document);

// Known built-ins are validated against the target-selection options used for
// this host build. Unknown names remain eligible for external ABI-v1 loading.
Result<void> ValidateDescriptorPluginSelection(std::string_view model);
Result<void> ValidateDynamicRemoverPluginSelection(
    std::string_view type, std::string_view model);

}  // namespace open_lmm
