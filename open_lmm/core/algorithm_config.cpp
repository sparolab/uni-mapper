#include "algorithm_config.hpp"

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
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

Result<const nlohmann::json*> TypedDocument(
    const ValidatedConfigDocument& source, ConfigDocumentKind expected,
    const char* name) {
  if (source.Kind() != expected) {
    return Result<const nlohmann::json*>::Failure(Error::InvalidArgument(
        std::string(name) + " decoder received the wrong document kind"));
  }
  return Result<const nlohmann::json*>::Ok(&source.Document());
}

int CheckedInt(const nlohmann::json& value, std::string_view pointer) {
  if (!value.is_number_unsigned() && !value.is_number_integer()) {
    throw std::invalid_argument(std::string(pointer) +
                                " must be an integer");
  }
  const auto integer = value.get<int64_t>();
  if (integer < std::numeric_limits<int>::min() ||
      integer > std::numeric_limits<int>::max()) {
    throw std::out_of_range(std::string(pointer) +
                            " does not fit the typed int representation");
  }
  return static_cast<int>(integer);
}

float CheckedFloat(const nlohmann::json& value, std::string_view pointer) {
  if (!value.is_number()) {
    throw std::invalid_argument(std::string(pointer) + " must be a number");
  }
  const double number = value.get<double>();
  if (!std::isfinite(number) ||
      number < -static_cast<double>(std::numeric_limits<float>::max()) ||
      number > static_cast<double>(std::numeric_limits<float>::max())) {
    throw std::out_of_range(std::string(pointer) +
                            " does not fit the finite float representation");
  }
  return static_cast<float>(number);
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

Result<DataLoaderConfig> DecodeDataLoaderConfig(
    const ValidatedConfigDocument& source) {
  return ParseTyped<DataLoaderConfig>("data_loader", [&] {
    auto typed = TypedDocument(source, ConfigDocumentKind::kDataLoader,
                               "data_loader");
    if (!typed) throw std::invalid_argument(typed.GetError().Message());
    const auto& root = *typed.Value();
    const auto& data = root.at("data_loader");
    DataLoaderConfig config;
    config.type = data.at("data_loader_type").get<std::string>();
    config.pose_format = data.at("pose_format").get<std::string>();
    config.scan_type = data.at("scan_type").get<std::string>();
    config.scan_dir_name = data.at("scan_dir_name").get<std::string>();
    config.pose_file_name = data.at("pose_file_name").get<std::string>();
    const auto extrinsic = data.at("extrinsic").get<std::vector<double>>();
    config.extrinsic = Eigen::Isometry3d::Identity();
    config.extrinsic.translation() =
        Eigen::Vector3d(extrinsic[0], extrinsic[1], extrinsic[2]);
    const double quaternion_scale =
        std::max({std::abs(extrinsic[3]), std::abs(extrinsic[4]),
                  std::abs(extrinsic[5]), std::abs(extrinsic[6])});
    if (!std::isfinite(quaternion_scale) || quaternion_scale == 0.0) {
      throw std::invalid_argument(
          "/data_loader/extrinsic quaternion must have finite non-zero norm");
    }
    Eigen::Vector4d scaled(extrinsic[3] / quaternion_scale,
                           extrinsic[4] / quaternion_scale,
                           extrinsic[5] / quaternion_scale,
                           extrinsic[6] / quaternion_scale);
    const double scaled_norm = scaled.norm();
    if (!std::isfinite(scaled_norm) || scaled_norm <= 0.0) {
      throw std::invalid_argument(
          "/data_loader/extrinsic quaternion normalization failed");
    }
    scaled /= scaled_norm;
    config.extrinsic.linear() =
        Eigen::Quaterniond(scaled[3], scaled[0], scaled[1], scaled[2])
            .toRotationMatrix();
    const auto& matrix = config.extrinsic.matrix();
    const auto& rotation = config.extrinsic.linear();
    if (!matrix.allFinite() ||
        !rotation.transpose().isApprox(rotation.inverse(), 1.0e-12) ||
        !std::isfinite(rotation.determinant()) ||
        std::abs(rotation.determinant() - 1.0) > 1.0e-12) {
      throw std::invalid_argument(
          "/data_loader/extrinsic must decode to a finite rigid transform");
    }
    config.voxel_size =
        CheckedFloat(data.at("voxel_size"), "/data_loader/voxel_size");
    config.min_range =
        CheckedFloat(data.at("min_range"), "/data_loader/min_range");
    config.max_range =
        CheckedFloat(data.at("max_range"), "/data_loader/max_range");
    config.delimiter = data.at("delimiter").get<std::string>();
    return config;
  });
}

Result<LoopDetectorConfig> DecodeLoopDetectorConfig(
    const ValidatedConfigDocument& source) {
  return ParseTyped<LoopDetectorConfig>("loop_detector", [&] {
    auto typed = TypedDocument(source, ConfigDocumentKind::kLoopDetector,
                               "loop_detector");
    if (!typed) throw std::invalid_argument(typed.GetError().Message());
    const auto& root = *typed.Value();
    const auto& loop = root.at("loop_detector");
    const auto& database = root.at("database");
    const auto& alignment = root.at("alignment");
    LoopDetectorConfig config;
    config.type = loop.at("loop_detector_type").get<std::string>();
    config.plugin_abi = loop.at("plugin_abi").get<std::string>();
    config.num_candidates = database.at("num_candidates").get<std::size_t>();
    config.distance_threshold = database.at("distance_threshold").get<double>();
    config.kdtree_rebuild_threshold =
        database.at("rebuild_threshold").get<std::size_t>();
    config.model = loop.at("model").get<std::string>();
    config.pcm_translation_threshold =
        alignment.at("pcm_translation_threshold").get<double>();
    config.pcm_rotation_threshold_deg =
        alignment.at("pcm_rotation_threshold_deg").get<double>();
    config.pcm_solver = alignment.at("pcm_solver").get<std::string>();
    config.pcm_threads =
        CheckedInt(alignment.at("pcm_threads"), "/alignment/pcm_threads");
    config.pcm_max_candidates =
        alignment.at("pcm_max_candidates").get<std::size_t>();
    config.kiss_voxel_size = CheckedFloat(
        alignment.at("kiss_voxel_size"), "/alignment/kiss_voxel_size");
    config.kiss_use_quatro = alignment.at("kiss_use_quatro").get<bool>();
    config.pose_nn_distance_threshold = CheckedFloat(
        alignment.at("pose_nn_distance_threshold"),
        "/alignment/pose_nn_distance_threshold");
    config.inter_loop_keyframe_spacing_m = CheckedFloat(
        alignment.at("inter_loop_keyframe_spacing_m"),
        "/alignment/inter_loop_keyframe_spacing_m");
    config.feedback_mode = alignment.at("feedback_mode").get<std::string>();
    config.headless_policy = alignment.at("headless_policy").get<std::string>();
    config.feedback_timeout_sec = CheckedInt(
        alignment.at("feedback_timeout_sec"),
        "/alignment/feedback_timeout_sec");
    config.plugin_config_json = source.CanonicalJson();
    return config;
  });
}

Result<OptimizerConfig> DecodeOptimizerConfig(
    const ValidatedConfigDocument& source) {
  return ParseTyped<OptimizerConfig>("backend_optimizer", [&] {
    auto typed = TypedDocument(source, ConfigDocumentKind::kBackendOptimizer,
                               "backend_optimizer");
    if (!typed) throw std::invalid_argument(typed.GetError().Message());
    const auto& data = typed.Value()->at("backend_optimizer");
    OptimizerConfig config;
    config.type = data.at("backend_optimizer_type").get<std::string>();
    config.relinearize_threshold =
        data.at("relinearizeThreshold").get<double>();
    config.relinearize_skip = CheckedInt(
        data.at("relinearizeSkip"),
        "/backend_optimizer/relinearizeSkip");
    config.isam_extra_updates = CheckedInt(
        data.at("isam_extra_updates"),
        "/backend_optimizer/isam_extra_updates");
    config.min_loop_frame_gap = CheckedInt(
        data.at("min_loop_frame_gap"),
        "/backend_optimizer/min_loop_frame_gap");
    config.icp_search_num = CheckedInt(
        data.at("icp_search_num"), "/backend_optimizer/icp_search_num");
    return config;
  });
}

Result<DynamicRemoverConfig> DecodeDynamicRemoverConfig(
    const ValidatedConfigDocument& source) {
  return ParseTyped<DynamicRemoverConfig>("dynamic_remover", [&] {
    auto typed = TypedDocument(source, ConfigDocumentKind::kDynamicRemover,
                               "dynamic_remover");
    if (!typed) throw std::invalid_argument(typed.GetError().Message());
    const auto& data = typed.Value()->at("dynamic_remover");
    DynamicRemoverConfig config;
    config.type = data.at("dynamic_remover_type").get<std::string>();
    config.model = data.at("model").get<std::string>();
    config.plugin_abi = data.at("plugin_abi").get<std::string>();
    const int internal_cpu_threads = config.model == "erasor"
        ? CheckedInt(data.at("internal_cpu_threads"),
                     "/dynamic_remover/internal_cpu_threads")
        : 1;
    config.plugin_config_json = source.CanonicalJson();
    config.internal_cpu_threads =
        static_cast<std::size_t>(internal_cpu_threads);
    if (config.model == "erasor") {
      config.thread_safety =
          PluginThreadSafety::kInstanceIsolatedParallel;
    }
    return config;
  });
}

Result<MapSaveConfig> DecodeMapSaveConfig(
    const ValidatedConfigDocument& source) {
  return ParseTyped<MapSaveConfig>("map_server", [&] {
    auto typed = TypedDocument(source, ConfigDocumentKind::kMapServer,
                               "map_server");
    if (!typed) throw std::invalid_argument(typed.GetError().Message());
    const auto& data = typed.Value()->at("map_server");
    MapSaveConfig config;
    config.enable_map_updater = data.at("enable_map_updater").get<bool>();
    config.save_voxel_size = data.at("save_voxel_size").get<double>();
    config.parallel_data_load = data.at("parallel_data_load").get<bool>();
    config.parallel_map_update = data.at("parallel_map_update").get<bool>();
    const int max_parallel_agents = CheckedInt(
        data.at("max_parallel_agents"), "/map_server/max_parallel_agents");
    config.max_parallel_agents =
        static_cast<std::size_t>(max_parallel_agents);
    return config;
  });
}

}  // namespace open_lmm
