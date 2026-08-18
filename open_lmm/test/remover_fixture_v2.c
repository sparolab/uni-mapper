#include <open_lmm/common/plugin_api_v2.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#ifdef __cplusplus
#include <stdexcept>
#endif

struct open_lmm_plugin_handle_v2 {
  char mode[32];
  uint64_t expected;
  uint64_t pushed;
  int active;
  float points[64][4];
};

#ifdef __cplusplus
extern "C" {
#endif

static int abort_count;
int open_lmm_remover_fixture_abort_count(void) { return abort_count; }
void open_lmm_remover_fixture_reset(void) { abort_count = 0; }

static open_lmm_string_view_v2 string_view(const char* text) {
  open_lmm_string_view_v2 value = {
      sizeof(value), OPEN_LMM_PLUGIN_ABI_V2_MAJOR,
      OPEN_LMM_PLUGIN_ABI_V2_MINOR, text, strlen(text)};
  return value;
}
static open_lmm_status_v2 status(open_lmm_status_code_v2 code,
                                 const char* text) {
  open_lmm_status_v2 value = {
      sizeof(value), OPEN_LMM_PLUGIN_ABI_V2_MAJOR,
      OPEN_LMM_PLUGIN_ABI_V2_MINOR, code, {0}};
  value.message = string_view(text);
  return value;
}
static int operation(const open_lmm_call_v2* call, const char* name) {
  return call && call->operation.size == strlen(name) &&
         memcmp(call->operation.data, name, call->operation.size) == 0;
}

open_lmm_status_v2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* descriptor) {
  static open_lmm_operation_descriptor_v2 operations[4];
  if (!descriptor) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "null");
  memset(descriptor, 0, sizeof(*descriptor));
  descriptor->struct_size = sizeof(*descriptor);
  descriptor->abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  descriptor->abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  descriptor->plugin_kind = string_view("dynamic_remover");
  descriptor->plugin_name = string_view("fixture");
  descriptor->capability = string_view("dynamic_remover:v2");
  descriptor->capability_bits = OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
      OPEN_LMM_CAPABILITY_POSE_VIEW_V2 |
      OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2 |
      OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2;
  descriptor->minimum_host_minor = 1;
  descriptor->plugin_id = string_view("test.remover.fixture");
  descriptor->plugin_version = string_view("1.0");
  const char* names[4] = {OPEN_LMM_REMOVER_BEGIN_OPERATION_V2,
                          OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2,
                          OPEN_LMM_REMOVER_FINISH_OPERATION_V2,
                          OPEN_LMM_REMOVER_ABORT_OPERATION_V2};
  for (int i = 0; i < 4; ++i) {
    operations[i].struct_size = sizeof(operations[i]);
    operations[i].abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
    operations[i].abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
    operations[i].operation = string_view(names[i]);
  }
  operations[0].required_capability_bits = OPEN_LMM_CAPABILITY_POSE_VIEW_V2;
  operations[1].required_capability_bits = OPEN_LMM_CAPABILITY_POINT_VIEW_V2;
  descriptor->operations = operations;
#ifdef OPEN_LMM_REMOVER_FIXTURE_BAD_OPS
  descriptor->operation_count = 3;
#else
  descriptor->operation_count = 4;
#endif
  descriptor->schema_id = string_view("");
  descriptor->thread_safety = OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2;
  descriptor->cancellation = OPEN_LMM_CANCELLATION_COOPERATIVE_V2;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}

open_lmm_status_v2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** output) {
  (void)host;
  if (!config || !output) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "open");
  open_lmm_plugin_handle_v2* handle =
      (open_lmm_plugin_handle_v2*)calloc(1, sizeof(*handle));
  if (!handle) return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "alloc");
  const size_t size = config->json.size < sizeof(handle->mode) - 1
      ? (size_t)config->json.size : sizeof(handle->mode) - 1;
  if (size) memcpy(handle->mode, config->json.data, size);
  *output = handle;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}

open_lmm_status_v2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* sink) {
  if (!handle || !call) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "call");
  if (operation(call, OPEN_LMM_REMOVER_ABORT_OPERATION_V2)) {
    ++abort_count; handle->active = 0;
    return status(OPEN_LMM_STATUS_OK_V2, "");
  }
  if (operation(call, OPEN_LMM_REMOVER_BEGIN_OPERATION_V2)) {
    handle->active = 1;
    handle->expected = call->indexed_poses.frame_count;
    handle->pushed = 0;
    if (strstr(handle->mode, "slow_begin")) usleep(50000);
    if (strstr(handle->mode, "fail_begin"))
      return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "begin failed");
    return status(OPEN_LMM_STATUS_OK_V2, "");
  }
  if (operation(call, OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2)) {
#ifdef __cplusplus
    if (strstr(handle->mode, "throw_push"))
      throw std::runtime_error("fixture push throw");
#endif
    if (strstr(handle->mode, "slow_push")) usleep(50000);
    if (strstr(handle->mode, "fail_push"))
      return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "push failed");
    if (!handle->active || handle->pushed >= 64 ||
        !call->frame_points.points.data || call->frame_points.points.count == 0)
      return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "push input");
    memcpy(handle->points[handle->pushed], call->frame_points.points.data,
           sizeof(handle->points[0]));
    ++handle->pushed;
    return status(OPEN_LMM_STATUS_OK_V2, "");
  }
  if (operation(call, OPEN_LMM_REMOVER_FINISH_OPERATION_V2)) {
    if (strstr(handle->mode, "slow_finish")) usleep(50000);
    if (strstr(handle->mode, "fail_finish"))
      return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "finish failed");
    if (strstr(handle->mode, "null_finish")) {
      handle->active = 0;
      return status(OPEN_LMM_STATUS_OK_V2, "");
    }
    if (!sink || !sink->write || handle->pushed != handle->expected)
      return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "finish input");
    open_lmm_point_cloud_header_v2 header = {
        sizeof(header), OPEN_LMM_PLUGIN_ABI_V2_MAJOR,
        OPEN_LMM_PLUGIN_ABI_V2_MINOR, handle->pushed, 16, 4,
        OPEN_LMM_ELEMENT_F32_V2, OPEN_LMM_ENDIAN_LITTLE_V2, 0};
    if (strstr(handle->mode, "malformed")) header.stride_bytes = 12;
    const size_t bytes = sizeof(header) + handle->pushed * 16;
    unsigned char* result = (unsigned char*)malloc(bytes);
    memcpy(result, &header, sizeof(header));
    memcpy(result + sizeof(header), handle->points, handle->pushed * 16);
    open_lmm_blob_view_v2 chunk = {
        sizeof(chunk), OPEN_LMM_PLUGIN_ABI_V2_MAJOR,
        OPEN_LMM_PLUGIN_ABI_V2_MINOR, result, bytes,
                                   OPEN_LMM_MEMORY_HOST_V2};
    const open_lmm_status_code_v2 written = sink->write(sink->host_context, &chunk);
    free(result);
    handle->active = 0;
    return status(written, written ? "sink" : "");
  }
  return status(OPEN_LMM_STATUS_UNSUPPORTED_CAPABILITY_V2, "operation");
}

void open_lmm_plugin_close_v2(open_lmm_plugin_handle_v2* handle) {
  free(handle);
}
#ifdef __cplusplus
}
#endif
