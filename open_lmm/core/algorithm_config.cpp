#include "algorithm_config.hpp"

#include <exception>
#include <stdexcept>

namespace open_lmm {
namespace {

#ifndef OPEN_LMM_HAVE_DESCRIPTOR_SCAN_CONTEXT
#define OPEN_LMM_HAVE_DESCRIPTOR_SCAN_CONTEXT 1
#endif
#ifndef OPEN_LMM_HAVE_DESCRIPTOR_SOLID
#define OPEN_LMM_HAVE_DESCRIPTOR_SOLID 1
#endif
#ifndef OPEN_LMM_HAVE_DYNAMIC_REMOVER_HMM_MOS
#define OPEN_LMM_HAVE_DYNAMIC_REMOVER_HMM_MOS 1
#endif
#ifndef OPEN_LMM_HAVE_DYNAMIC_REMOVER_DUFOMAP
#define OPEN_LMM_HAVE_DYNAMIC_REMOVER_DUFOMAP 1
#endif
#ifndef OPEN_LMM_HAVE_DYNAMIC_REMOVER_OTD
#define OPEN_LMM_HAVE_DYNAMIC_REMOVER_OTD 1
#endif
#ifndef OPEN_LMM_HAVE_DYNAMIC_REMOVER_FREE_DOM
#define OPEN_LMM_HAVE_DYNAMIC_REMOVER_FREE_DOM 1
#endif
#ifndef OPEN_LMM_HAVE_DYNAMIC_REMOVER_ERASOR
#define OPEN_LMM_HAVE_DYNAMIC_REMOVER_ERASOR 1
#endif

Result<void> DisabledBuiltIn(std::string_view kind, std::string_view model) {
  return Result<void>::Failure(Error::InvalidArgument(
      std::string(kind) + " plugin '" + std::string(model) +
      "' is not included in this OpenLMM build"));
}

template <typename T, typename Parser>
Result<T> ParseTyped(const char* name, Parser&& parser) {
  try {
    return Result<T>::Ok(parser());
  } catch (const std::exception& error) {
    return Result<T>::Failure(
        Error::ParseError(std::string(name) + " config: " + error.what())
            .WithConfig(name));
  }
}

}  // namespace

Result<void> ValidateDescriptorPluginSelection(std::string_view model) {
  if (model == "scan_context" && !OPEN_LMM_HAVE_DESCRIPTOR_SCAN_CONTEXT) {
    return DisabledBuiltIn("descriptor", model);
  }
  if (model == "solid" && !OPEN_LMM_HAVE_DESCRIPTOR_SOLID) {
    return DisabledBuiltIn("descriptor", model);
  }
  return Result<void>::Ok();
}

Result<void> ValidateDynamicRemoverPluginSelection(
    std::string_view type, std::string_view model) {
  const bool known_online = model == "hmm_mos" || model == "dufomap" ||
                            model == "otd";
  const bool known_offline = model == "free_dom" || model == "erasor";
  if ((known_online && type != "online") ||
      (known_offline && type != "offline")) {
    return Result<void>::Failure(Error::InvalidArgument(
        "dynamic remover plugin '" + std::string(model) +
        "' is incompatible with type '" + std::string(type) + "'"));
  }
  if ((model == "hmm_mos" && !OPEN_LMM_HAVE_DYNAMIC_REMOVER_HMM_MOS) ||
      (model == "dufomap" && !OPEN_LMM_HAVE_DYNAMIC_REMOVER_DUFOMAP) ||
      (model == "otd" && !OPEN_LMM_HAVE_DYNAMIC_REMOVER_OTD) ||
      (model == "free_dom" && !OPEN_LMM_HAVE_DYNAMIC_REMOVER_FREE_DOM) ||
      (model == "erasor" && !OPEN_LMM_HAVE_DYNAMIC_REMOVER_ERASOR)) {
    return DisabledBuiltIn("dynamic remover", model);
  }
  return Result<void>::Ok();
}

Result<DataLoaderConfig> ParseDataLoaderConfig(const Config& source) {
  return ParseTyped<DataLoaderConfig>("data_loader", [&] {
    DataLoaderConfig config;
    config.type = source.param<std::string>(
        "data_loader", "data_loader_type", "");
    config.pose_format = source.param<std::string>(
        "data_loader", "pose_format", "");
    config.scan_type = source.param<std::string>(
        "data_loader", "scan_type", "");
    config.scan_dir_name = source.param<std::string>(
        "data_loader", "scan_dir_name", "");
    config.pose_file_name = source.param<std::string>(
        "data_loader", "pose_file_name", "");
    config.extrinsic = source.param<Eigen::Isometry3d>(
        "data_loader", "extrinsic", Eigen::Isometry3d::Identity());
    config.voxel_size = source.param<float>(
        "data_loader", "voxel_size", 0.1F);
    config.min_range = source.param<float>(
        "data_loader", "min_range", 0.0F);
    config.max_range = source.param<float>(
        "data_loader", "max_range", 100.0F);
    config.delimiter = source.param<std::string>(
        "data_loader", "delimiter", " ");
    if (config.type != "file_based") {
      throw std::invalid_argument(
          "data_loader_type must be file_based");
    }
    if (config.pose_file_name.empty() || config.scan_dir_name.empty() ||
        config.voxel_size <= 0.0F || config.min_range < 0.0F ||
        config.max_range <= config.min_range || config.delimiter.size() != 1) {
      throw std::invalid_argument(
          "file names, positive voxel size, valid range, and one-character "
          "delimiter are required");
    }
    if (config.pose_format == "custom") {
      throw std::invalid_argument(
          "pose_format 'custom' is declared but not implemented");
    }
    if (config.pose_format != "kitti" && config.pose_format != "tum") {
      throw std::invalid_argument("unsupported pose_format; expected kitti or tum");
    }
    if (config.scan_type != "pcd" && config.scan_type != "bin") {
      throw std::invalid_argument("unsupported scan_type");
    }
    return config;
  });
}

Result<LoopDetectorConfig> ParseLoopDetectorConfig(const Config& source) {
  return ParseTyped<LoopDetectorConfig>("loop_detector", [&] {
    LoopDetectorConfig config;
    config.type = source.param<std::string>(
        "loop_detector", "loop_detector_type", "");
    config.plugin_abi = source.param<std::string>(
        "loop_detector", "plugin_abi", "auto");
    const int num_candidates = source.param<int>(
        "database", "num_candidates", 5);
    const int rebuild_threshold = source.param<int>(
        "database", "rebuild_threshold", 50);
    const int max_candidates = source.param<int>(
        "alignment", "pcm_max_candidates", 0);
    config.num_candidates = num_candidates > 0
                                ? static_cast<std::size_t>(num_candidates)
                                : 0;
    config.distance_threshold = source.param<double>(
        "database", "distance_threshold", 0.13);
    config.kdtree_rebuild_threshold = rebuild_threshold > 0
                                          ? static_cast<std::size_t>(rebuild_threshold)
                                          : 0;
    config.model = source.param<std::string>(
        "loop_detector", "model", "");
    config.pcm_translation_threshold = source.param<double>(
        "alignment", "pcm_translation_threshold", 10.0);
    config.pcm_rotation_threshold_deg = source.param<double>(
        "alignment", "pcm_rotation_threshold_deg", 20.0);
    config.pcm_solver = source.param<std::string>(
        "alignment", "pcm_solver", "heuristic");
    config.pcm_threads = source.param<int>("alignment", "pcm_threads", 1);
    config.pcm_max_candidates = max_candidates >= 0
                                    ? static_cast<std::size_t>(max_candidates)
                                    : 0;
    config.kiss_voxel_size = source.param<float>(
        "alignment", "kiss_voxel_size", 2.0F);
    config.kiss_use_quatro = source.param<bool>(
        "alignment", "kiss_use_quatro", false);
    config.pose_nn_distance_threshold = source.param<float>(
        "alignment", "pose_nn_distance_threshold", 10.0F);
    config.feedback_mode = source.param<std::string>(
        "alignment", "feedback_mode", "adaptive");
    config.headless_policy = source.param<std::string>(
        "alignment", "headless_policy", "legacy_combined");
    config.feedback_timeout_sec = source.param<int>(
        "alignment", "feedback_timeout_sec", 0);
    config.plugin_config_json = source.ToJson();
    if (config.type != "kdtree") {
      throw std::invalid_argument("loop_detector_type must be kdtree");
    }
    if ((config.plugin_abi != "auto" && config.plugin_abi != "v1" &&
         config.plugin_abi != "v2") ||
        num_candidates <= 0 || rebuild_threshold <= 0 ||
        max_candidates < 0 || config.distance_threshold < 0.0 ||
        config.model.empty() || config.pcm_translation_threshold <= 0.0 ||
        config.pcm_rotation_threshold_deg <= 0.0 ||
        config.pcm_rotation_threshold_deg > 180.0 ||
        (config.pcm_solver != "heuristic" && config.pcm_solver != "exact") ||
        config.pcm_threads <= 0 || config.kiss_voxel_size <= 0.0F ||
        config.pose_nn_distance_threshold <= 0.0F ||
        config.feedback_timeout_sec < 0 ||
        (config.feedback_mode != "adaptive" &&
         config.feedback_mode != "automatic" &&
         config.feedback_mode != "interactive" &&
         config.feedback_mode != "always_manual") ||
        (config.headless_policy != "legacy_combined" &&
         config.headless_policy != "kiss_only" &&
         config.headless_policy != "kiss_then_descriptor" &&
         config.headless_policy != "fail")) {
      throw std::invalid_argument("invalid KD-tree alignment parameters");
    }
    auto plugin = ValidateDescriptorPluginSelection(config.model);
    if (!plugin) throw std::invalid_argument(plugin.GetError().Message());
    return config;
  });
}

Result<OptimizerConfig> ParseOptimizerConfig(const Config& source) {
  return ParseTyped<OptimizerConfig>("backend_optimizer", [&] {
    OptimizerConfig config;
    config.type = source.param<std::string>(
        "backend_optimizer", "backend_optimizer_type", "");
    config.relinearize_threshold = source.param<double>(
        "backend_optimizer", "relinearizeThreshold", 0.1);
    config.relinearize_skip = source.param<int>(
        "backend_optimizer", "relinearizeSkip", 1);
    config.isam_extra_updates = source.param<int>(
        "backend_optimizer", "isam_extra_updates", 5);
    config.min_loop_frame_gap = source.param<int>(
        "backend_optimizer", "min_loop_frame_gap", 30);
    config.icp_search_num = source.param<int>(
        "backend_optimizer", "icp_search_num", 3);
    if (config.type != "incremental" ||
        config.relinearize_threshold <= 0.0 || config.relinearize_skip <= 0 ||
        config.isam_extra_updates < 0 || config.min_loop_frame_gap < 0 ||
        config.icp_search_num < 0) {
      throw std::invalid_argument("invalid incremental optimizer parameters");
    }
    return config;
  });
}

Result<DynamicRemoverConfig> ParseDynamicRemoverConfig(const Config& source) {
  return ParseTyped<DynamicRemoverConfig>("dynamic_remover", [&] {
    DynamicRemoverConfig config;
    config.type = source.param<std::string>(
        "dynamic_remover", "dynamic_remover_type", "");
    config.model = source.param<std::string>("dynamic_remover", "model", "");
    const int internal_cpu_threads = config.model == "erasor"
        ? source.param<int>("dynamic_remover", "internal_cpu_threads", 1)
        : 1;
    config.plugin_config_json = source.ToJson();
    if ((config.type != "offline" && config.type != "online") ||
        config.model.empty() || internal_cpu_threads <= 0) {
      throw std::invalid_argument(
          "type/model must be valid and internal_cpu_threads must be positive");
    }
    config.internal_cpu_threads =
        static_cast<std::size_t>(internal_cpu_threads);
    auto plugin = ValidateDynamicRemoverPluginSelection(
        config.type, config.model);
    if (!plugin) throw std::invalid_argument(plugin.GetError().Message());
    if (config.model == "erasor") {
      config.thread_safety =
          PluginThreadSafety::kInstanceIsolatedParallel;
    }
    return config;
  });
}

Result<MapSaveConfig> ParseMapSaveConfig(const Config& source) {
  return ParseTyped<MapSaveConfig>("map_server", [&] {
    MapSaveConfig config;
    config.enable_map_updater = source.param<bool>(
        "map_server", "enable_map_updater", true);
    config.save_voxel_size = source.param<double>(
        "map_server", "save_voxel_size", 0.2);
    config.parallel_data_load = source.param<bool>(
        "map_server", "parallel_data_load", false);
    config.parallel_map_update = source.param<bool>(
        "map_server", "parallel_map_update", false);
    const int max_parallel_agents = source.param<int>(
        "map_server", "max_parallel_agents", 1);
    if (config.save_voxel_size <= 0.0 || max_parallel_agents <= 0) {
      throw std::invalid_argument(
          "save_voxel_size and max_parallel_agents must be positive");
    }
    config.max_parallel_agents =
        static_cast<std::size_t>(max_parallel_agents);
    return config;
  });
}

}  // namespace open_lmm
