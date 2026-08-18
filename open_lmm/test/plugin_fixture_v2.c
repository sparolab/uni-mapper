#include <open_lmm/common/plugin_api_v2.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#ifndef OPEN_LMM_PLUGIN_FIXTURE_V2_MODE
#define OPEN_LMM_PLUGIN_FIXTURE_V2_MODE 0
#endif

struct open_lmm_plugin_handle_v2 {
  const open_lmm_host_api_v2* host;
};

static uint32_t close_count = 0;
static uint32_t active_calls = 0;
static uint32_t maximum_active_calls = 0;

uint32_t open_lmm_fixture_close_count_v2(void) { return close_count; }
uint32_t open_lmm_fixture_maximum_active_calls_v2(void) {
  return maximum_active_calls;
}

#define V2_INIT(type) \
  { sizeof(type), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR }

static open_lmm_string_view_v2 text(const char* value) {
  open_lmm_string_view_v2 result = V2_INIT(open_lmm_string_view_v2);
  result.data = value;
  result.size = value ? (uint64_t)strlen(value) : 0;
  return result;
}

static open_lmm_status_v2 status(open_lmm_status_code_v2 code,
                                 const char* message) {
  open_lmm_status_v2 result = V2_INIT(open_lmm_status_v2);
  result.code = code;
  result.message = text(message);
  return result;
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* descriptor) {
  if (!descriptor) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "descriptor");
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 1
  descriptor->abi_major = 99;
#elif OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 2
  descriptor->struct_size = 4;
#else
  descriptor->struct_size = sizeof(*descriptor);
  descriptor->abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  descriptor->abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
#endif
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 5
  descriptor->abi_minor = 7;
#endif
  descriptor->plugin_kind = text("fixture");
  descriptor->plugin_name = text("c_echo_v2");
  descriptor->capability = text("echo,cancel");
  descriptor->capability_bits = OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
                                OPEN_LMM_CAPABILITY_POSE_VIEW_V2;
  descriptor->minimum_host_minor = 0;
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 8
  descriptor->minimum_host_minor = 1;
#endif
  return status(OPEN_LMM_STATUS_OK_V2, "");
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** out) {
  (void)config;
  if (!host || !out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "open args");
  *out = (open_lmm_plugin_handle_v2*)calloc(1, sizeof(**out));
  if (!*out) return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "allocation");
  (*out)->host = host;
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 7
  free(*out);
  *out = NULL;
  return status(OPEN_LMM_STATUS_OK_V2, "");
#endif
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 3
  return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "partial open");
#else
  return status(OPEN_LMM_STATUS_OK_V2, "");
#endif
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* sink) {
  if (!handle || !call || !sink || !sink->write)
    return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "call args");
  if (handle->host->is_cancelled &&
      handle->host->is_cancelled(handle->host->host_context))
    return status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
  ++active_calls;
  if (active_calls > maximum_active_calls) maximum_active_calls = active_calls;
  if (call->operation.size == 4 && call->operation.data &&
      memcmp(call->operation.data, "slow", 4) == 0) {
    usleep(20000);
  }
  open_lmm_blob_view_v2 chunk = V2_INIT(open_lmm_blob_view_v2);
#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE == 4
  chunk.data = NULL;
  chunk.size = 1;
#else
  if (call->operation.size == 6 && call->operation.data &&
      memcmp(call->operation.data, "chunks", 6) == 0) {
    static const char repeated[] = "four";
    chunk.data = repeated;
    chunk.size = sizeof(repeated) - 1;
    chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
    for (int i = 0; i < 4; ++i) {
      if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2) {
        --active_calls;
        return status(OPEN_LMM_STATUS_HOST_ERROR_V2, "sink rejected");
      }
    }
    --active_calls;
    return status(OPEN_LMM_STATUS_OK_V2, "");
  } else if (call->operation.size == 5 && call->operation.data &&
      memcmp(call->operation.data, "views", 5) == 0) {
    static const char view_result[] = "views-ok";
    if (!call->points.data || call->points.count != 2 ||
        call->points.stride_bytes != 4 * sizeof(float) ||
        call->points.element_type != OPEN_LMM_ELEMENT_F32_V2 ||
        !call->poses.data || call->poses.count != 1 ||
        call->poses.stride_bytes != 16 * sizeof(double) ||
        call->poses.element_type != OPEN_LMM_ELEMENT_F64_V2) {
      return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid views");
    }
    chunk.data = view_result;
    chunk.size = sizeof(view_result) - 1;
  } else if (call->operation.size == 5 && call->operation.data &&
             memcmp(call->operation.data, "point", 5) == 0) {
    static const char point_result[] = "point-ok";
    if (!call->points.data || call->points.count != 2 ||
        call->points.stride_bytes != 4 * sizeof(float) ||
        call->points.element_type != OPEN_LMM_ELEMENT_F32_V2 ||
        call->poses.count != 0 || call->poses.data != NULL) {
      return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid point view");
    }
    chunk.data = point_result;
    chunk.size = sizeof(point_result) - 1;
  } else if (call->operation.size == 4 && call->operation.data &&
             memcmp(call->operation.data, "pose", 4) == 0) {
    static const char pose_result[] = "pose-ok";
    if (!call->poses.data || call->poses.count != 1 ||
        call->poses.stride_bytes != 16 * sizeof(double) ||
        call->poses.element_type != OPEN_LMM_ELEMENT_F64_V2 ||
        call->points.count != 0 || call->points.data != NULL) {
      return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid pose view");
    }
    chunk.data = pose_result;
    chunk.size = sizeof(pose_result) - 1;
  } else {
    chunk.data = call->request.data;
    chunk.size = call->request.size;
  }
#endif
  chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2) {
    --active_calls;
    return status(OPEN_LMM_STATUS_HOST_ERROR_V2, "sink rejected");
  }
  --active_calls;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}

#if OPEN_LMM_PLUGIN_FIXTURE_V2_MODE != 6
void OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_close_v2(
    open_lmm_plugin_handle_v2* handle) {
  ++close_count;
  free(handle);
}
#endif
