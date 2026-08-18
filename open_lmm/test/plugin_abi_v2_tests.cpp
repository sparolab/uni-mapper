#include <open_lmm/common/plugin_host_v2.hpp>

#include <dlfcn.h>

#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <utility>

namespace {
int Fail(const char* message) {
  std::cerr << message << '\n';
  return EXIT_FAILURE;
}
int32_t OPEN_LMM_PLUGIN_CALL_V2 Cancelled(void* value) {
  return *static_cast<bool*>(value) ? 1 : 0;
}
}  // namespace

int main(int argc, char** argv) {
  if (argc != 11) return Fail("expected ten fixture paths");
  void* valid_probe = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  void* partial_probe = dlopen(argv[4], RTLD_NOW | RTLD_LOCAL);
  if (!valid_probe || !partial_probe) return Fail("fixture probe dlopen failed");
  using CloseCount = uint32_t (*)();
  auto valid_close_count = reinterpret_cast<CloseCount>(
      dlsym(valid_probe, "open_lmm_fixture_close_count_v2"));
  auto partial_close_count = reinterpret_cast<CloseCount>(
      dlsym(partial_probe, "open_lmm_fixture_close_count_v2"));
  auto maximum_active_calls = reinterpret_cast<CloseCount>(
      dlsym(valid_probe, "open_lmm_fixture_maximum_active_calls_v2"));
  if (!valid_close_count || !partial_close_count || !maximum_active_calls)
    return Fail("fixture probe missing");
  bool cancelled = false;
  const std::string input = "host-owned-copy";
  {
    auto plugin = open_lmm::LoadPluginV2(argv[1], "fixture", "{}", 1,
                                         &cancelled, &Cancelled);
    if (!plugin || plugin.Value().Metadata().name != "c_echo_v2")
      return Fail("valid C plugin did not load");
    auto loaded = std::move(plugin).Value();
    if (loaded.Metadata().plugin_id != "org.openlmm.fixture.echo" ||
        loaded.Metadata().plugin_version != "1.0.0" ||
        loaded.Metadata().schema_id !=
            "org.openlmm.fixture.echo.schema" ||
        loaded.Metadata().schema_fragment_json.empty() ||
        loaded.Metadata().operations.size() != 7)
      return Fail("minor-1 self-description metadata was not copied");
    if (loaded.Call({"undeclared", nullptr, 0}))
      return Fail("undeclared operation was accepted");
    auto output = loaded.Call({"echo", input.data(), input.size()});
    if (!output ||
        std::string(output.Value().begin(), output.Value().end()) != input)
      return Fail("echo call failed");
    bool call_cancelled = true;
    open_lmm::PluginV2Call call_scoped_cancel{"echo", input.data(),
                                               input.size()};
    call_scoped_cancel.cancellation_context = &call_cancelled;
    call_scoped_cancel.is_cancelled = &Cancelled;
    auto cancelled_call = loaded.Call(call_scoped_cancel);
    if (cancelled_call || cancelled_call.GetError().code !=
                              open_lmm::Error::Code::kCancelled)
      return Fail("call-scoped cancellation was not propagated");
    if (!loaded.Call({"echo", input.data(), input.size()}))
      return Fail("call-scoped cancellation leaked into the next call");
    const float points[2][4] = {{1.0F, 2.0F, 3.0F, 4.0F},
                                {5.0F, 6.0F, 7.0F, 8.0F}};
    double poses[1][16]{};
    poses[0][0] = poses[0][5] = poses[0][10] = poses[0][15] = 1.0;
    open_lmm::PluginV2Call view_call{"views", nullptr, 0};
    view_call.points = open_lmm::PluginPointView{
        points, 2, sizeof(points[0]), OPEN_LMM_ELEMENT_F32_V2,
        OPEN_LMM_ENDIAN_LITTLE_V2};
    view_call.poses = open_lmm::PluginPoseView{
        poses, 1, sizeof(poses[0]), OPEN_LMM_ELEMENT_F64_V2,
        OPEN_LMM_ENDIAN_LITTLE_V2};
    auto views = loaded.Call(view_call);
    if (!views ||
        std::string(views.Value().begin(), views.Value().end()) != "views-ok")
      return Fail("point/pose view call failed");
    auto point_call = view_call;
    point_call.operation = "point";
    point_call.poses.reset();
    auto point_only = loaded.Call(point_call);
    if (!point_only ||
        std::string(point_only.Value().begin(), point_only.Value().end()) !=
            "point-ok")
      return Fail("point-only view call failed");
    auto pose_call = view_call;
    pose_call.operation = "pose";
    pose_call.points.reset();
    auto pose_only = loaded.Call(pose_call);
    if (!pose_only ||
        std::string(pose_only.Value().begin(), pose_only.Value().end()) !=
            "pose-ok")
      return Fail("pose-only view call failed");
    open_lmm::PluginV2Call byte_limited{"chunks", nullptr, 0};
    byte_limited.result_limits = {8, 8, 8};
    auto bytes_exceeded = loaded.Call(byte_limited);
    if (bytes_exceeded ||
        bytes_exceeded.GetError().context.plugin != "c_echo_v2")
      return Fail("total result byte limit was not enforced");
    open_lmm::PluginV2Call chunk_limited{"chunks", nullptr, 0};
    chunk_limited.result_limits = {64, 3, 8};
    if (loaded.Call(chunk_limited))
      return Fail("individual result chunk limit was not enforced");
    open_lmm::PluginV2Call count_limited{"chunks", nullptr, 0};
    count_limited.result_limits = {64, 8, 2};
    if (loaded.Call(count_limited))
      return Fail("result chunk count limit was not enforced");
    open_lmm::PluginV2Call invalid_limits{"echo", input.data(), input.size()};
    invalid_limits.result_limits = {4, 8, 1};
    auto invalid_limit_result = loaded.Call(invalid_limits);
    if (invalid_limit_result ||
        invalid_limit_result.GetError().code !=
            open_lmm::Error::Code::kInvalidArgument)
      return Fail("invalid host result limits were accepted");
    auto malformed_view = view_call;
    malformed_view.points->stride_bytes = sizeof(float) * 2;
    if (loaded.Call(malformed_view))
      return Fail("malformed point stride accepted");
    malformed_view = view_call;
    malformed_view.poses->data = nullptr;
    if (loaded.Call(malformed_view))
      return Fail("null pose data accepted");
    bool first_ok = false;
    bool second_ok = false;
    std::thread first([&] {
      first_ok = loaded.Call({"slow", input.data(), input.size()}).IsOk();
    });
    std::thread second([&] {
      second_ok = loaded.Call({"slow", input.data(), input.size()}).IsOk();
    });
    first.join();
    second.join();
    if (!first_ok || !second_ok || maximum_active_calls() != 1)
      return Fail("non-concurrent plugin handle calls were not serialized");
    cancelled = true;
    if (loaded.Call({"echo", input.data(), input.size()}))
      return Fail("cancel was not propagated");
  }
  if (valid_close_count() != 1) return Fail("valid handle was not closed once");
  if (open_lmm::LoadPluginV2(argv[2], "fixture", "{}"))
    return Fail("wrong major accepted");
  if (open_lmm::LoadPluginV2(argv[3], "fixture", "{}"))
    return Fail("short descriptor accepted");
  auto newer_minor = open_lmm::LoadPluginV2(argv[6], "fixture", "{}");
  if (!newer_minor || newer_minor.Value().Metadata().abi_minor != 7)
    return Fail("compatible newer minor was rejected");
  if (open_lmm::LoadPluginV2(argv[7], "fixture", "{}"))
    return Fail("missing close callback accepted");
  if (open_lmm::LoadPluginV2(argv[8], "fixture", "{}"))
    return Fail("successful open with null handle accepted");
  if (open_lmm::LoadPluginV2(argv[9], "fixture", "{}"))
    return Fail("unsupported minimum host minor accepted");
  auto older_minor = open_lmm::LoadPluginV2(argv[10], "fixture", "{}");
  if (!older_minor || older_minor.Value().Metadata().abi_minor != 0 ||
      older_minor.Value().Metadata().plugin_id != "c_echo_v2")
    return Fail("minimum readable minor-0 descriptor prefix was rejected");
  if (open_lmm::LoadPluginV2(argv[1], "fixture", "{}", UINT64_C(4)))
    return Fail("unsupported capability accepted");
  if (open_lmm::LoadPluginV2(argv[4], "fixture", "{}"))
    return Fail("failed partial open accepted");
  if (partial_close_count() != 1) return Fail("partial handle was not closed once");
  {
    auto malformed = open_lmm::LoadPluginV2(argv[5], "fixture", "{}");
    if (!malformed) return Fail("malformed-result fixture did not load");
    auto malformed_loaded = std::move(malformed).Value();
    if (malformed_loaded.Call({"echo", input.data(), input.size()}))
      return Fail("malformed result accepted");
  }
  dlclose(partial_probe);
  dlclose(valid_probe);
  return EXIT_SUCCESS;
}
