#include "config_schema.hpp"

#include <limits>
#include <stdexcept>
#include <utility>

namespace open_lmm {
namespace {

constexpr uint32_t kVersion = 1;

ConstraintSet Enum(std::initializer_list<nlohmann::json> values) {
  ConstraintSet constraints;
  constraints.allowed_values.assign(values);
  return constraints;
}

ConstraintSet Range(std::optional<double> minimum = std::nullopt,
                    std::optional<double> maximum = std::nullopt,
                    bool exclusive_minimum = false,
                    bool exclusive_maximum = false) {
  ConstraintSet constraints;
  constraints.minimum = minimum;
  constraints.maximum = maximum;
  constraints.exclusive_minimum = exclusive_minimum;
  constraints.exclusive_maximum = exclusive_maximum;
  return constraints;
}

FieldSpec Field(std::string pointer, SchemaValueType type, bool required,
                std::optional<nlohmann::json> default_value = std::nullopt,
                ConstraintSet constraints = {}) {
  FieldSpec field;
  field.pointer = std::move(pointer);
  field.type = type;
  field.required = required;
  field.default_value = std::move(default_value);
  field.constraints = std::move(constraints);
  return field;
}

FieldSpec Object(std::string pointer, bool required = true) {
  return Field(std::move(pointer), SchemaValueType::kObject, required);
}

FieldSpec ExtensionObject() {
  auto field = Object("/extensions", false);
  field.allow_unknown_children = true;
  return field;
}

FieldSpec Text(std::string pointer, bool required,
               std::optional<nlohmann::json> default_value = std::nullopt,
               bool non_empty = false) {
  ConstraintSet constraints;
  if (non_empty) constraints.minimum_length = 1;
  return Field(std::move(pointer), SchemaValueType::kString, required,
               std::move(default_value), std::move(constraints));
}

FieldSpec Path(std::string pointer, bool required,
               std::optional<nlohmann::json> default_value = std::nullopt,
               bool non_empty = false) {
  auto field = Text(std::move(pointer), required, std::move(default_value),
                    non_empty);
  field.type = SchemaValueType::kPath;
  return field;
}

FieldSpec Bool(std::string pointer, bool value) {
  return Field(std::move(pointer), SchemaValueType::kBoolean, false, value);
}

FieldSpec UInt(std::string pointer, uint64_t value,
               std::optional<double> minimum = std::nullopt,
               std::optional<double> maximum = std::nullopt) {
  if (!maximum)
    maximum = static_cast<double>(std::numeric_limits<int>::max());
  return Field(std::move(pointer), SchemaValueType::kUnsignedInteger, false,
               value, Range(minimum, maximum));
}

FieldSpec Number(std::string pointer, double value,
                 std::optional<double> minimum = std::nullopt,
                 std::optional<double> maximum = std::nullopt,
                 bool exclusive_minimum = false,
                 bool exclusive_maximum = false) {
  return Field(std::move(pointer), SchemaValueType::kNumber, false, value,
               Range(minimum, maximum, exclusive_minimum, exclusive_maximum));
}

FieldSpec FloatNumber(std::string pointer, double value,
                      std::optional<double> minimum = std::nullopt,
                      bool exclusive_minimum = false) {
  return Number(std::move(pointer), value, minimum,
                static_cast<double>(std::numeric_limits<float>::max()),
                exclusive_minimum);
}

FieldSpec Choice(std::string pointer, std::string value,
                 std::initializer_list<nlohmann::json> allowed,
                 bool required = false) {
  return Field(std::move(pointer), SchemaValueType::kString, required,
               required ? std::nullopt
                        : std::optional<nlohmann::json>(std::move(value)),
               Enum(allowed));
}

FragmentSelector Plugin(std::string pointer, std::string expected,
                        std::string kind) {
  return {std::move(pointer), expected, std::move(kind), expected};
}

CrossFieldRule Less(std::string left, std::string right,
                    std::string message) {
  return {CrossFieldRuleKind::kLessThan,
          {std::move(left), std::move(right)}, std::nullopt,
          std::move(message)};
}

CrossFieldRule Equals(std::string pointer, nlohmann::json expected,
                      std::string message) {
  return {CrossFieldRuleKind::kEqualsConstant, {std::move(pointer)},
          std::move(expected), std::move(message)};
}

SchemaFragment RootSchema() {
  SchemaFragment fragment{"root.v1", ConfigDocumentKind::kRoot, kVersion};
  fragment.fields = {
      Object("/global"),
      Path("/global/config_path", false, ""),
      Path("/global/config_map_server", true, std::nullopt, true),
      Path("/global/config_data_loader", true, std::nullopt, true),
      Path("/global/config_loop_detector", true, std::nullopt, true),
      Path("/global/config_backend_optimizer", true, std::nullopt, true),
      Path("/global/config_dynamic_remover", true, std::nullopt, true),
      Object("/directory"),
      Path("/directory/root_dir_path", true, std::nullopt, true),
      Path("/directory/root_save_dir", true, std::nullopt, true),
      Field("/directory/sub_dir_list", SchemaValueType::kArray, true,
            std::nullopt,
            ConstraintSet{.minimum_items = 1,
                          .maximum_items = 255,
                          .item_type = SchemaValueType::kString,
                          .items_non_empty = true,
                          .unique_items = true}),
      ExtensionObject(),
  };
  return fragment;
}

SchemaFragment MapSchema() {
  SchemaFragment fragment{"map_server.v1", ConfigDocumentKind::kMapServer,
                          kVersion};
  fragment.fields = {
      Object("/map_server"),
      Bool("/map_server/enable_map_updater", true),
      UInt("/map_server/anchor_agent_index", 0),
      Number("/map_server/save_voxel_size", 0.2, 0.0, std::nullopt, true),
      Bool("/map_server/parallel_data_load", false),
      Bool("/map_server/parallel_map_update", false),
      UInt("/map_server/max_parallel_agents", 1, 1, 255),
      ExtensionObject(),
  };
  return fragment;
}

SchemaFragment DataLoaderSchema() {
  SchemaFragment fragment{"data_loader.file_based.v1",
                          ConfigDocumentKind::kDataLoader, kVersion};
  ConstraintSet extrinsic;
  extrinsic.minimum_items = 7;
  extrinsic.maximum_items = 7;
  extrinsic.item_type = SchemaValueType::kNumber;
  extrinsic.array_slice_norm =
      ConstraintSet::ArraySliceNorm{3, 4, 1.0e-12, true};
  fragment.fields = {
      Object("/data_loader"),
      Choice("/data_loader/data_loader_type", "file_based", {"file_based"},
             true),
      Choice("/data_loader/pose_format", "kitti",
             {"kitti", "tum"}),
      Path("/data_loader/pose_file_name", false, "optimized_poses.txt", true),
      Field("/data_loader/extrinsic", SchemaValueType::kArray, false,
            nlohmann::json::array({0, 0, 0, 0, 0, 0, 1}), extrinsic),
      Choice("/data_loader/scan_type", "pcd", {"pcd", "bin"}),
      Path("/data_loader/scan_dir_name", false, "Scans", true),
      FloatNumber("/data_loader/voxel_size", 0.1, 0.0, true),
      FloatNumber("/data_loader/min_range", 0.0, 0.0),
      FloatNumber("/data_loader/max_range", 100.0, 0.0, true),
      Field("/data_loader/delimiter", SchemaValueType::kString, false, " ",
            ConstraintSet{.minimum_length = 1, .maximum_length = 1}),
      ExtensionObject(),
  };
  fragment.rules = {Less("/data_loader/min_range", "/data_loader/max_range",
                         "data loader min_range must be below max_range")};
  return fragment;
}

SchemaFragment LoopBaseSchema() {
  SchemaFragment fragment{"loop_detector.base.v1",
                          ConfigDocumentKind::kLoopDetector, kVersion};
  fragment.fields = {
      Object("/loop_detector"),
      Choice("/loop_detector/loop_detector_type", "kdtree", {"kdtree"},
             true),
      Choice("/loop_detector/plugin_abi", "auto", {"auto", "v1", "v2"}),
      Text("/loop_detector/model", true, std::nullopt, true),
      Field("/database", SchemaValueType::kObject, false,
            nlohmann::json::object()),
      UInt("/database/descriptor_vector_dim", 1, 1),
      Number("/database/distance_threshold", 0.13, 0.0),
      UInt("/database/num_candidates", 5, 1),
      UInt("/database/rebuild_threshold", 50, 1),
      Field("/alignment", SchemaValueType::kObject, false,
            nlohmann::json::object()),
      Number("/alignment/pcm_translation_threshold", 10.0, 0.0,
             std::nullopt, true),
      Number("/alignment/pcm_rotation_threshold_deg", 20.0, 0.0, 180.0,
             true),
      Choice("/alignment/pcm_solver", "heuristic", {"heuristic", "exact"}),
      UInt("/alignment/pcm_threads", 1, 1),
      UInt("/alignment/pcm_max_candidates", 0, 0),
      FloatNumber("/alignment/kiss_voxel_size", 2.0, 0.0, true),
      Bool("/alignment/kiss_use_quatro", false),
      FloatNumber("/alignment/pose_nn_distance_threshold", 10.0, 0.0, true),
      FloatNumber("/alignment/inter_loop_keyframe_spacing_m", 10.0, 0.0,
                  true),
      Choice("/alignment/feedback_mode", "adaptive",
             {"adaptive", "automatic", "interactive", "always_manual"}),
      Choice("/alignment/headless_policy", "kiss_then_descriptor",
             {"kiss_only", "kiss_then_descriptor", "fail"}),
      UInt("/alignment/feedback_timeout_sec", 0, 0),
      ExtensionObject(),
  };
  fragment.fields[20].value_migrations.push_back(
      {"legacy_combined", "kiss_then_descriptor", 1});
  return fragment;
}

SchemaFragment ScanContextSchema() {
  SchemaFragment fragment{"loop_detector.scan_context.v1",
                          ConfigDocumentKind::kLoopDetector, kVersion,
                          Plugin("/loop_detector/model", "scan_context",
                                 "descriptor")};
  fragment.fields = {
      UInt("/loop_detector/num_ring", 20, 1),
      UInt("/loop_detector/num_sector", 60, 1),
      Number("/loop_detector/max_range", 80.0, 0.0, std::nullopt, true),
  };
  return fragment;
}

SchemaFragment SolidSchema() {
  SchemaFragment fragment{"loop_detector.solid.v1",
                          ConfigDocumentKind::kLoopDetector, kVersion,
                          Plugin("/loop_detector/model", "solid",
                                 "descriptor")};
  fragment.fields = {
      UInt("/loop_detector/num_range", 40, 1),
      UInt("/loop_detector/num_angle", 60, 1),
      UInt("/loop_detector/num_height", 32, 1),
      Number("/loop_detector/fov_u", 2.0),
      Number("/loop_detector/fov_d", -24.8),
      UInt("/loop_detector/min_distance", 3, 0),
      UInt("/loop_detector/max_distance", 80, 1),
      Number("/loop_detector/voxel_size", 0.4, 0.0, std::nullopt, true),
  };
  fragment.rules = {
      Less("/loop_detector/fov_d", "/loop_detector/fov_u",
           "SOLiD lower FOV must be below upper FOV"),
      Less("/loop_detector/min_distance", "/loop_detector/max_distance",
           "SOLiD min_distance must be below max_distance")};
  return fragment;
}

SchemaFragment OptimizerSchema() {
  SchemaFragment fragment{"backend_optimizer.incremental.v1",
                          ConfigDocumentKind::kBackendOptimizer, kVersion};
  fragment.fields = {
      Object("/backend_optimizer"),
      Choice("/backend_optimizer/backend_optimizer_type", "incremental",
             {"incremental"}, true),
      Number("/backend_optimizer/relinearizeThreshold", 0.1, 0.0,
             std::nullopt, true),
      UInt("/backend_optimizer/relinearizeSkip", 1, 1),
      UInt("/backend_optimizer/isam_extra_updates", 5, 0),
      UInt("/backend_optimizer/min_loop_frame_gap", 30, 0),
      UInt("/backend_optimizer/icp_search_num", 3, 0),
      ExtensionObject(),
  };
  return fragment;
}

SchemaFragment RemoverBaseSchema() {
  SchemaFragment fragment{"dynamic_remover.base.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion};
  fragment.fields = {
      Object("/dynamic_remover"),
      Field("/dynamic_remover/dynamic_remover_type", SchemaValueType::kString,
            true, std::nullopt, Enum({"online", "offline"})),
      Text("/dynamic_remover/model", true, std::nullopt, true),
      Choice("/dynamic_remover/plugin_abi", "auto", {"auto", "v1", "v2"}),
      ExtensionObject(),
  };
  return fragment;
}

SchemaFragment HmmMosSchema() {
  SchemaFragment fragment{"dynamic_remover.hmm_mos.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion,
                          Plugin("/dynamic_remover/model", "hmm_mos",
                                 "dynamic_remover")};
  fragment.fields = {
      Bool("/dynamic_remover/replace_intensity", true),
      Number("/dynamic_remover/voxel_size", 0.2, 0.0, std::nullopt, true),
      Number("/dynamic_remover/occupancy_sigma", 0.2, 0.0, std::nullopt, true),
      Number("/dynamic_remover/free_sigma", 0.2, 0.0, std::nullopt, true),
      Number("/dynamic_remover/belief_threshold", 0.99, 0.0, 1.0, true),
      UInt("/dynamic_remover/conv_size", 5, 1),
      UInt("/dynamic_remover/local_window_size", 3, 1),
      UInt("/dynamic_remover/global_window_size", 300, 1),
      UInt("/dynamic_remover/min_otsu", 3, 0),
      Number("/dynamic_remover/max_range", 50.0, 0.0, std::nullopt, true),
      Number("/dynamic_remover/min_range", 0.5, 0.0),
  };
  fragment.rules = {
      Equals("/dynamic_remover/dynamic_remover_type", "online",
             "hmm_mos requires online dynamic remover type"),
      Less("/dynamic_remover/min_range", "/dynamic_remover/max_range",
           "hmm_mos min_range must be below max_range")};
  return fragment;
}

SchemaFragment DufoMapSchema() {
  SchemaFragment fragment{"dynamic_remover.dufomap.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion,
                          Plugin("/dynamic_remover/model", "dufomap",
                                 "dynamic_remover")};
  fragment.fields = {
      Bool("/dynamic_remover/replace_intensity", false),
      Number("/dynamic_remover/resolution", 0.2, 0.0, std::nullopt, true),
      UInt("/dynamic_remover/levels", 20, 1),
      Choice("/dynamic_remover/down_sampling_method", "center",
             {"center", "centroid", "uniform"}),
      UInt("/dynamic_remover/hit_depth", 0, 0),
      UInt("/dynamic_remover/miss_depth", 0, 0),
      Number("/dynamic_remover/max_range", 80.0),
      Number("/dynamic_remover/inflate_unknown", 1.0, 0.0),
      Bool("/dynamic_remover/inflate_unknown_compensation", true),
      Bool("/dynamic_remover/ray_passthrough_hits", false),
      Number("/dynamic_remover/inflate_hits_dist", 0.2, 0.0),
      Bool("/dynamic_remover/simple_ray_casting", false),
      Number("/dynamic_remover/simple_ray_casting_factor", 1.0, 0.0,
             std::nullopt, true),
      Bool("/dynamic_remover/parallel", true),
      UInt("/dynamic_remover/num_threads", 16, 1),
      Bool("/dynamic_remover/only_valid", false),
  };
  fragment.rules = {
      Equals("/dynamic_remover/dynamic_remover_type", "online",
             "dufomap requires online dynamic remover type")};
  return fragment;
}

SchemaFragment OtdSchema() {
  SchemaFragment fragment{"dynamic_remover.otd.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion,
                          Plugin("/dynamic_remover/model", "otd",
                                 "dynamic_remover")};
  fragment.fields = {
      Bool("/dynamic_remover/replace_intensity", true),
      Number("/dynamic_remover/sensor_height", 1.73, 0.0, std::nullopt, true),
      Number("/dynamic_remover/tau_seeds", 0.6, 0.0),
      Number("/dynamic_remover/tau_dis", 0.14, 0.0),
      Number("/dynamic_remover/tau_ratio", 0.8, 0.0, 1.0),
      Number("/dynamic_remover/voxel_size", 0.5, 0.0, std::nullopt, true),
  };
  fragment.rules = {
      Equals("/dynamic_remover/dynamic_remover_type", "online",
             "otd requires online dynamic remover type")};
  return fragment;
}

SchemaFragment ErasorSchema() {
  SchemaFragment fragment{"dynamic_remover.erasor.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion,
                          Plugin("/dynamic_remover/model", "erasor",
                                 "dynamic_remover")};
  fragment.fields = {
      Bool("/dynamic_remover/replace_intensity", false),
      Number("/dynamic_remover/max_range", 80.0, 0.0, std::nullopt, true),
      UInt("/dynamic_remover/num_rings", 20, 1),
      UInt("/dynamic_remover/num_sectors", 108, 1),
      Number("/dynamic_remover/min_h", -1.7),
      Number("/dynamic_remover/max_h", 3.1),
      Number("/dynamic_remover/scan_ratio_threshold", 0.2, 0.0, 1.0),
      UInt("/dynamic_remover/minimum_num_pts", 6, 1),
      Number("/dynamic_remover/gf_dist_thr", 0.125, 0.0, std::nullopt, true),
      UInt("/dynamic_remover/gf_iter", 3, 1),
      UInt("/dynamic_remover/gf_num_lpr", 10, 1),
      Number("/dynamic_remover/gf_th_seeds_height", 0.5),
      UInt("/dynamic_remover/num_lowest_pts", 1, 1),
      Number("/dynamic_remover/query_voxel_size", 0.1, 0.0, std::nullopt,
             true),
      Number("/dynamic_remover/map_voxel_size", 0.2, 0.0, std::nullopt, true),
      UInt("/dynamic_remover/voxelization_interval", 10, 1),
      UInt("/dynamic_remover/removal_interval", 5, 1),
      UInt("/dynamic_remover/internal_cpu_threads", 1, 1, 255),
      Number("/dynamic_remover/tf_z", 0.7),
  };
  fragment.rules = {
      Equals("/dynamic_remover/dynamic_remover_type", "offline",
             "erasor requires offline dynamic remover type"),
      Less("/dynamic_remover/min_h", "/dynamic_remover/max_h",
           "erasor min_h must be below max_h")};
  return fragment;
}

SchemaFragment FreeDomSchema() {
  SchemaFragment fragment{"dynamic_remover.free_dom.v1",
                          ConfigDocumentKind::kDynamicRemover, kVersion,
                          Plugin("/dynamic_remover/model", "free_dom",
                                 "dynamic_remover")};
  fragment.fields = {
      Bool("/dynamic_remover/replace_intensity", true),
      Choice("/dynamic_remover/dynamic_removal_level", "aggressive",
             {"conservative", "moderate", "aggressive"}),
      Number("/dynamic_remover/min_range", 2.7, 0.0),
      Number("/dynamic_remover/max_range", 80.0, 0.0, std::nullopt, true),
      Number("/dynamic_remover/min_z", -20.0),
      Number("/dynamic_remover/max_z", 20.0),
      Number("/dynamic_remover/sub_voxel_size", 0.1, 0.0, std::nullopt, true),
      UInt("/dynamic_remover/voxel_depth", 2, 1),
      UInt("/dynamic_remover/block_depth", 5, 1),
      Bool("/dynamic_remover/enable_local_map", false),
      Number("/dynamic_remover/local_map_range", 50.0, 0.0, std::nullopt,
             true),
      Number("/dynamic_remover/local_map_min_z", -20.0),
      Number("/dynamic_remover/local_map_max_z", 20.0),
      Number("/dynamic_remover/raycast_max_range", 80.0, 0.0,
             std::nullopt, true),
      Number("/dynamic_remover/raycast_min_z", -10.0),
      Number("/dynamic_remover/raycast_max_z", 10.0),
      UInt("/dynamic_remover/counts_to_free", 6, 1),
      UInt("/dynamic_remover/counts_to_revert", 20, 1),
      UInt("/dynamic_remover/conservative_connectivity", 26, 1),
      UInt("/dynamic_remover/aggressive_connectivity", 124, 1),
      Bool("/dynamic_remover/enable_raycast_enhancement", true),
      Number("/dynamic_remover/lidar_horizon_fov_degree", 360.0, 0.0, 360.0,
             true),
      Number("/dynamic_remover/lidar_vertical_fov_upper_degree", 2.0),
      Number("/dynamic_remover/lidar_vertical_fov_lower_degree", -24.8),
      UInt("/dynamic_remover/depth_image_vertical_lines", 64, 2),
      Number("/dynamic_remover/depth_image_min_range", 0.2, 0.0),
      Number("/dynamic_remover/max_raycast_enhancement_range", 80.0, 0.0),
      Number("/dynamic_remover/raycast_enhancement_depth_margin", 0.2, 0.0),
      UInt("/dynamic_remover/inpaint_size", 3, 0),
      UInt("/dynamic_remover/erosion_size", 0, 0),
      Number("/dynamic_remover/min_raycast_enhancement_area", 0.0, 0.0),
      UInt("/dynamic_remover/depth_image_top_margin", 0, 0),
      Bool("/dynamic_remover/learn_fov", false),
      Bool("/dynamic_remover/enable_fov_mask", false),
      Path("/dynamic_remover/fov_mask_path", false, ""),
      Path("/dynamic_remover/fov_mask_name", false),
      UInt("/dynamic_remover/num_threads", 8, 1),
  };
  fragment.fields[35].deprecation =
      {true, "/dynamic_remover/fov_mask_path", 1};
  fragment.rules = {
      Equals("/dynamic_remover/dynamic_remover_type", "offline",
             "free_dom requires offline dynamic remover type"),
      Less("/dynamic_remover/min_range", "/dynamic_remover/max_range",
           "free_dom min_range must be below max_range"),
      Less("/dynamic_remover/min_z", "/dynamic_remover/max_z",
           "free_dom min_z must be below max_z"),
      Less("/dynamic_remover/local_map_min_z",
           "/dynamic_remover/local_map_max_z",
           "free_dom local map min z must be below max z"),
      Less("/dynamic_remover/raycast_min_z", "/dynamic_remover/raycast_max_z",
           "free_dom raycast min z must be below max z"),
      Less("/dynamic_remover/lidar_vertical_fov_lower_degree",
           "/dynamic_remover/lidar_vertical_fov_upper_degree",
           "free_dom lower vertical FOV must be below upper vertical FOV")};
  return fragment;
}

std::vector<SchemaFragment> BuiltinFragments() {
  std::vector<SchemaFragment> fragments;
  fragments.push_back(RootSchema());
  fragments.push_back(MapSchema());
  fragments.push_back(DataLoaderSchema());
  fragments.push_back(LoopBaseSchema());
  fragments.push_back(ScanContextSchema());
  fragments.push_back(SolidSchema());
  fragments.push_back(OptimizerSchema());
  fragments.push_back(RemoverBaseSchema());
  fragments.push_back(HmmMosSchema());
  fragments.push_back(DufoMapSchema());
  fragments.push_back(OtdSchema());
  fragments.push_back(ErasorSchema());
  fragments.push_back(FreeDomSchema());
  return fragments;
}

}  // namespace

const SchemaRegistry& BuiltinConfigSchemaRegistry() {
  static const SchemaRegistry registry = [] {
    auto created = SchemaRegistry::Create(BuiltinFragments());
    if (!created) {
      throw std::logic_error("invalid built-in config schema: " +
                             created.GetError().Message());
    }
    return std::move(created).Value();
  }();
  return registry;
}

Result<void> ValidateSessionConfigDocuments(
    const ValidatedConfigDocument& root,
    const ValidatedConfigDocument& map_server) {
  if (root.Kind() != ConfigDocumentKind::kRoot ||
      map_server.Kind() != ConfigDocumentKind::kMapServer) {
    return Result<void>::Failure(
        Error::InvalidArgument("session validation requires root and map schema documents"));
  }
  const auto agent_count = root.Document().at("directory").at("sub_dir_list").size();
  const auto anchor = map_server.Document()
                          .at("map_server")
                          .at("anchor_agent_index")
                          .get<std::size_t>();
  if (anchor >= agent_count) {
    return Result<void>::Failure(
        Error::InvalidArgument("map anchor_agent_index exceeds configured agent count")
            .WithConfig("session")
            .WithValidation("/map_server/anchor_agent_index",
                            "index < /directory/sub_dir_list size",
                            std::to_string(anchor), kVersion));
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
