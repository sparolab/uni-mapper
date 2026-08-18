#include <open_lmm/common/plugin_api_v2.h>

#include <stdlib.h>
#include <string.h>

#ifndef OPEN_LMM_DESCRIPTOR_V2_FIXTURE_MALFORMED
#define OPEN_LMM_DESCRIPTOR_V2_FIXTURE_MALFORMED 0
#endif

struct open_lmm_plugin_handle_v2 {
  int unused;
};

#define INIT_V2(type) \
  { sizeof(type), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR }

static open_lmm_string_view_v2 text(const char* value) {
  open_lmm_string_view_v2 result = INIT_V2(open_lmm_string_view_v2);
  result.data = value;
  result.size = value ? (uint64_t)strlen(value) : 0;
  return result;
}

static open_lmm_status_v2 status(open_lmm_status_code_v2 code,
                                 const char* message) {
  open_lmm_status_v2 result = INIT_V2(open_lmm_status_v2);
  result.code = code;
  result.message = text(message);
  return result;
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* descriptor) {
  if (!descriptor) {
    return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "descriptor");
  }
  descriptor->struct_size = sizeof(*descriptor);
  descriptor->abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  descriptor->abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR + 6;
  descriptor->plugin_kind = text("descriptor");
  descriptor->plugin_name = text("newer_minor_descriptor_fixture");
  descriptor->capability = text("descriptor");
  descriptor->capability_bits =
      OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2 |
      OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2;
#if OPEN_LMM_DESCRIPTOR_V2_FIXTURE_MALFORMED
  descriptor->capability_bits &= ~OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2;
#endif
  descriptor->minimum_host_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  descriptor->plugin_id = text("org.openlmm.fixture.descriptor.newer");
#if OPEN_LMM_DESCRIPTOR_V2_FIXTURE_MALFORMED
  descriptor->plugin_id.data = NULL;
#endif
  descriptor->plugin_version = text("1.0.0");
  descriptor->schema_id = text("");
  descriptor->thread_safety = OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2;
  descriptor->cancellation = OPEN_LMM_CANCELLATION_COOPERATIVE_V2;
  static open_lmm_artifact_format_v2 format;
  memset(&format, 0, sizeof(format));
  format.struct_size = sizeof(format);
  format.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  format.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  format.format_id = text("fixture_descriptor");
  format.format_version = 1;
  format.index_dimension = 4;
  descriptor->artifact_formats = &format;
  descriptor->artifact_format_count = 1;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** out) {
  (void)host;
  (void)config;
  if (!out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "out");
  *out = (open_lmm_plugin_handle_v2*)calloc(1, sizeof(**out));
  return *out ? status(OPEN_LMM_STATUS_OK_V2, "")
              : status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "allocation");
}

open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle,
    const open_lmm_call_v2* request,
    open_lmm_result_sink_v2* sink) {
  (void)handle;
  (void)request;
  (void)sink;
  return status(OPEN_LMM_STATUS_UNSUPPORTED_CAPABILITY_V2, "fixture");
}

void OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_close_v2(
    open_lmm_plugin_handle_v2* handle) {
  free(handle);
}
