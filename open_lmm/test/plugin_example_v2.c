#include <open_lmm/common/plugin_api_v2.h>

#include <stdlib.h>
#include <string.h>

struct open_lmm_plugin_handle_v2 { const open_lmm_host_api_v2* host; };
static open_lmm_string_view_v2 text(const char* value) {
  open_lmm_string_view_v2 view = {0};
  view.struct_size = sizeof(view);
  view.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  view.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  view.data = value;
  view.size = value ? (uint64_t)strlen(value) : 0;
  return view;
}
static open_lmm_status_v2 status(open_lmm_status_code_v2 code,
                                 const char* message) {
  open_lmm_status_v2 value = {0};
  value.struct_size = sizeof(value);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  value.code = code;
  value.message = text(message);
  return value;
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* out) {
  if (!out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "descriptor");
  out->struct_size = sizeof(*out);
  out->abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  out->abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  out->plugin_kind = text("example");
  out->plugin_name = text("echo");
  out->capability = text("echo,cancel");
  out->capability_bits = UINT64_C(3);
  out->minimum_host_minor = 0;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}
open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** out) {
  (void)config;
  if (!host || !out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "open");
  *out = (open_lmm_plugin_handle_v2*)malloc(sizeof(**out));
  if (!*out) return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "allocation");
  (*out)->host = host;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}
open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* sink) {
  if (!handle || !call || !sink || !sink->write)
    return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "call");
  if (handle->host->is_cancelled &&
      handle->host->is_cancelled(handle->host->host_context))
    return status(OPEN_LMM_STATUS_CANCELLED_V2, "cancelled");
  open_lmm_blob_view_v2 output = {0};
  output.struct_size = sizeof(output);
  output.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  output.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  output.data = call->request.data;
  output.size = call->request.size;
  output.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (sink->write(sink->host_context, &output) != OPEN_LMM_STATUS_OK_V2)
    return status(OPEN_LMM_STATUS_HOST_ERROR_V2, "sink");
  return status(OPEN_LMM_STATUS_OK_V2, "");
}
void OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_close_v2(
    open_lmm_plugin_handle_v2* handle) { free(handle); }
