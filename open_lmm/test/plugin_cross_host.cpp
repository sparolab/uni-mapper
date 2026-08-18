#include <open_lmm/common/plugin_host_v2.hpp>

#include <string>

namespace {
int32_t OPEN_LMM_PLUGIN_CALL_V2 Cancelled(void* value) {
  return *static_cast<bool*>(value) ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 2) return 2;
  bool cancelled = false;
  auto result = open_lmm::LoadPluginV2(argv[1], "fixture", "{}", 1,
                                       &cancelled, &Cancelled);
  if (!result) return 3;
  auto plugin = std::move(result).Value();
  const std::string request = "cross-toolchain";
  auto response = plugin.Call({"echo", request.data(), request.size()});
  if (!response ||
      std::string(response.Value().begin(), response.Value().end()) != request) {
    return 4;
  }
  const float points[2][4] = {{1.0F, 2.0F, 3.0F, 4.0F},
                              {5.0F, 6.0F, 7.0F, 8.0F}};
  double poses[1][16]{};
  poses[0][0] = poses[0][5] = poses[0][10] = poses[0][15] = 1.0;
  open_lmm::PluginV2Call views{"views", nullptr, 0};
  views.points = open_lmm::PluginPointView{
      points, 2, sizeof(points[0]), OPEN_LMM_ELEMENT_F32_V2,
      OPEN_LMM_ENDIAN_LITTLE_V2};
  views.poses = open_lmm::PluginPoseView{
      poses, 1, sizeof(poses[0]), OPEN_LMM_ELEMENT_F64_V2,
      OPEN_LMM_ENDIAN_LITTLE_V2};
  auto view_response = plugin.Call(views);
  if (!view_response ||
      std::string(view_response.Value().begin(), view_response.Value().end()) !=
          "views-ok") {
    return 5;
  }
  cancelled = true;
  return plugin.Call({"echo", request.data(), request.size()}) ? 6 : 0;
}
