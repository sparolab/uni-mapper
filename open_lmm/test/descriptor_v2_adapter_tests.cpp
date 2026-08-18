#include <open_lmm/common/plugin_host_v2.hpp>
#include <open_lmm/core/descriptor/built_in_descriptor_engine.hpp>
#include <open_lmm/core/descriptor/generic_descriptor_v2_adapter.hpp>
#include <open_lmm/utils/load_module.hpp>

#include <cmath>
#include <algorithm>
#include <array>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <span>
#include <vector>

namespace {
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

AlgorithmExecutionContext Context() {
  AlgorithmExecutionContext context;
  context.base_revision = 9;
  context.cancellation = std::make_shared<CancellationToken>();
  return context;
}

std::vector<pcl::PointXYZI> Scan(double yaw) {
  std::vector<pcl::PointXYZI> points;
  for (int ring = 0; ring < 8; ++ring) {
    for (int sector = 0; sector < 60; ++sector) {
      const double angle = yaw + sector * 2.0 * M_PI / 60.0;
      const double radius = 8.0 + ring * 2.0 + (sector % 7) * 0.1;
      pcl::PointXYZI point;
      point.x = static_cast<float>(radius * std::cos(angle));
      point.y = static_cast<float>(radius * std::sin(angle));
      point.z = static_cast<float>((sector % 11) * 0.2 + ring);
      point.intensity = static_cast<float>(sector);
      points.push_back(point);
    }
  }
  return points;
}

void TestGenericAndV1Parity(const char* scan_library,
                            const char* solid_library) {
  const std::string scan_config = R"({"loop_detector":{"num_sector":60,"num_ring":20,"max_range":80.0}})";
  const std::string solid_config = R"({"loop_detector":{}})";
  auto scan_engine =
      LoadGenericDescriptorV2Adapter(scan_library, scan_config).Value();
  auto second_scan_engine =
      LoadGenericDescriptorV2Adapter(scan_library, scan_config).Value();
  auto solid_engine =
      LoadGenericDescriptorV2Adapter(solid_library, solid_config).Value();
  const std::string twelve_ring_config =
      R"({"loop_detector":{"num_sector":60,"num_ring":12,"max_range":80.0}})";
  auto twelve_ring_engine =
      LoadGenericDescriptorV2Adapter(scan_library, twelve_ring_config).Value();
  Check(scan_engine->IndexMetadata().index_dimension == 20 &&
            solid_engine->IndexMetadata().index_dimension == 40 &&
            twelve_ring_engine->IndexMetadata().index_dimension == 12,
        "both external descriptor plugins use generic metadata");

  auto first_points = Scan(0.0);
  auto second_points = Scan(2.0 * M_PI / 60.0 * 7.0);
  auto first = scan_engine->Make(
      Context(), DescriptorPointView{std::span<const pcl::PointXYZI>(first_points)});
  auto second = scan_engine->Make(
      Context(), DescriptorPointView{std::span<const pcl::PointXYZI>(second_points)});
  Check(first && second, "generic Scan Context make succeeds");
  auto v2_match = scan_engine->Compare(Context(), first.Value(), second.Value());
  Check(v2_match && std::isfinite(v2_match.Value().score),
        "generic adapter delegates compare to plugin");
  auto cross_handle = second_scan_engine->Compare(
      Context(), first.Value(), second.Value());
  Check(cross_handle &&
            std::abs(cross_handle.Value().score - v2_match.Value().score) <
                1e-12 &&
            cross_handle.Value().relative_pose.matrix().isApprox(
                v2_match.Value().relative_pose.matrix(), 1e-12),
        "artifacts are portable across compatible plugin handles");
  Check(!twelve_ring_engine->Compare(Context(), first.Value(), second.Value()),
        "artifacts from a differently configured handle were accepted");

  auto solid = solid_engine->Make(
      Context(), DescriptorPointView{std::span<const pcl::PointXYZI>(first_points)});
  Check(static_cast<bool>(solid), "generic SOLiD make succeeds");
  Check(!scan_engine->Compare(Context(), first.Value(), solid.Value()),
        "cross-format artifacts are rejected before plugin call");

  auto legacy = load_plugin_v1<IDescriptorKdtree>(
      scan_library, "descriptor", scan_config).Value();
  auto v1_engine = BuiltInDescriptorEngine::Create(
      "scan_context.v1", "scan_context", 1, legacy).Value();
  auto v1_first = v1_engine->Make(
      Context(), DescriptorPointView{std::span<const pcl::PointXYZI>(first_points)}).Value();
  auto v1_second = v1_engine->Make(
      Context(), DescriptorPointView{std::span<const pcl::PointXYZI>(second_points)}).Value();
  auto v1_match = v1_engine->Compare(Context(), v1_first, v1_second).Value();
  Check(std::abs(v1_match.score - v2_match.Value().score) < 1e-12 &&
            v1_match.relative_pose.matrix().isApprox(
                v2_match.Value().relative_pose.matrix(), 1e-12),
        "Scan Context v1/v2 score and yaw are identical");
}

void TestIndexKeyOperation(const char* scan_library) {
  const std::string config = R"({"loop_detector":{"num_sector":60,"num_ring":20,"max_range":80.0}})";
  constexpr uint64_t capabilities =
      OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2;
  auto plugin = LoadPluginV2(scan_library, "descriptor", config, capabilities)
                    .Value();
  auto points = Scan(0.0);
  PluginV2Call make{OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2, nullptr, 0};
  make.points = PluginPointView{points.data(), points.size(),
                                sizeof(pcl::PointXYZI),
                                OPEN_LMM_ELEMENT_F32_V2,
                                OPEN_LMM_ENDIAN_LITTLE_V2};
  auto artifact = plugin.Call(make).Value();
  open_lmm_descriptor_artifact_header_v2 header{};
  std::memcpy(&header, artifact.data(), sizeof(header));
  const std::size_t payload_offset =
      sizeof(header) + header.format_id_size +
      header.key_count * sizeof(double);
  Check(payload_offset + header.payload_size == artifact.size(),
        "descriptor artifact envelope is bounded");
  PluginV2Call index{OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2,
                     artifact.data() + payload_offset, header.payload_size};
  auto key = plugin.Call(index);
  Check(key && key.Value().size() ==
                   sizeof(open_lmm_descriptor_index_key_header_v2) +
                       header.key_count * sizeof(double),
        "declared index-key operation is callable");

  std::vector<std::array<uint8_t, 16>> big_endian_points(points.size());
  for (std::size_t i = 0; i < points.size(); ++i) {
    const float values[] = {points[i].x, points[i].y, points[i].z, 0.0F};
    for (std::size_t component = 0; component < 4; ++component) {
      std::memcpy(big_endian_points[i].data() + component * sizeof(float),
                  &values[component], sizeof(float));
      std::reverse(big_endian_points[i].begin() + component * sizeof(float),
                   big_endian_points[i].begin() +
                       (component + 1) * sizeof(float));
    }
  }
  PluginV2Call big_endian_make{OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2,
                                nullptr, 0};
  big_endian_make.points = PluginPointView{
      big_endian_points.data(), big_endian_points.size(),
      sizeof(big_endian_points.front()), OPEN_LMM_ELEMENT_F32_V2,
      OPEN_LMM_ENDIAN_BIG_V2};
  Check(static_cast<bool>(plugin.Call(big_endian_make)),
        "big-endian descriptor point input was rejected");
}

}  // namespace

int main(int argc, char** argv) {
  Check(argc == 7, "descriptor and probe fixture paths are required");
  TestGenericAndV1Parity(argv[1], argv[2]);
  TestIndexKeyOperation(argv[1]);
  auto missing_symbols = ProbeGenericDescriptorV2Plugin(argv[3]);
  Check(missing_symbols && missing_symbols.Value() ==
                               DescriptorV2Availability::kUnavailable,
        "missing complete ABI-v2 symbols are fallback-eligible");
  Check(!ProbeGenericDescriptorV2Plugin(argv[4]),
        "malformed ABI-v2 query is a hard error, not fallback-eligible");
  auto newer_minor = ProbeGenericDescriptorV2Plugin(argv[5]);
  Check(newer_minor &&
            newer_minor.Value() == DescriptorV2Availability::kAvailable,
        "newer additive minor is accepted through the known descriptor prefix");
  auto missing_operations = LoadGenericDescriptorV2Adapter(argv[5], "{}");
  Check(!missing_operations &&
            missing_operations.GetError().Message().find(
                "canonical operation metadata") != std::string::npos,
        "descriptor capabilities without canonical operations were accepted");
  Check(!ProbeGenericDescriptorV2Plugin(argv[6]),
        "malformed metadata was hidden by a missing descriptor capability");
  std::cout << "Descriptor ABI-v2 adapter tests passed\n";
  return EXIT_SUCCESS;
}
