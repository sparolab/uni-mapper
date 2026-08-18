#include <open_lmm/common/plugin_api_v2.h>
#include <stdlib.h>
#include <string.h>

struct open_lmm_plugin_handle_v2 { int unused; };
#define INIT(type) { sizeof(type), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR }

static open_lmm_string_view_v2 text(const char* value) {
  open_lmm_string_view_v2 result = INIT(open_lmm_string_view_v2);
  result.data = value; result.size = value ? (uint64_t)strlen(value) : 0;
  return result;
}
static open_lmm_status_v2 status(open_lmm_status_code_v2 code,
                                 const char* message) {
  open_lmm_status_v2 result = INIT(open_lmm_status_v2);
  result.code = code; result.message = text(message); return result;
}
open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* out) {
  if (!out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "query");
  if (getenv("OPEN_LMM_SCHEMA_FIXTURE_QUERY_UNAVAILABLE"))
    return status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "temporarily unavailable");
  static open_lmm_operation_descriptor_v2 operation;
  operation = (open_lmm_operation_descriptor_v2)INIT(open_lmm_operation_descriptor_v2);
  operation.operation = text(OPEN_LMM_PLUGIN_SCHEMA_OPERATION_V2);
  operation.required_capability_bits = OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2;
  *out = (open_lmm_plugin_descriptor_v2)INIT(open_lmm_plugin_descriptor_v2);
  out->plugin_kind = text("descriptor");
  out->plugin_name = text("external_fixture");
  out->capability = text("schema");
  out->capability_bits = OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2;
  out->minimum_host_minor = 1;
  out->plugin_id = text("external_fixture");
  out->plugin_version = text("1.0.0");
  out->operations = &operation; out->operation_count = 1;
  out->schema_id = text("descriptor.external_fixture.schema");
  out->schema_version = 1;
  out->thread_safety = OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2;
  out->cancellation = OPEN_LMM_CANCELLATION_NONE_V2;
  return status(OPEN_LMM_STATUS_OK_V2, "");
}
open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** out) {
  (void)host; (void)config;
  if (!out) return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "open");
  *out = (open_lmm_plugin_handle_v2*)calloc(1, sizeof(**out));
  return *out ? status(OPEN_LMM_STATUS_OK_V2, "")
              : status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "allocation");
}
open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* sink) {
  if (!handle || !call || !sink || !sink->write)
    return status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "call");
  static const char schema[] =
      "{\"id\":\"descriptor.external_fixture.schema\",\"version\":1,"
      "\"document_kind\":\"loop_detector\","
      "\"selector\":{\"pointer\":\"/loop_detector/model\","
      "\"equals\":\"external_fixture\"},"
      "\"fields\":[{\"pointer\":\"/loop_detector/external_fixture/tuning\","
      "\"type\":\"number\",\"required\":true}]}";
  static const char mutated_schema[] =
      "{\"id\":\"descriptor.external_fixture.schema\",\"version\":1,"
      "\"document_kind\":\"dynamic_remover\",\"fields\":[]}";
  const char* selected = getenv("OPEN_LMM_SCHEMA_FIXTURE_MUTATED")
                             ? mutated_schema
                             : schema;
  open_lmm_blob_view_v2 chunk = INIT(open_lmm_blob_view_v2);
  chunk.data = selected; chunk.size = (uint64_t)strlen(selected);
  chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2)
    return status(OPEN_LMM_STATUS_HOST_ERROR_V2, "sink");
  return status(OPEN_LMM_STATUS_OK_V2, "");
}
void OPEN_LMM_PLUGIN_CALL_V2 open_lmm_plugin_close_v2(
    open_lmm_plugin_handle_v2* handle) { free(handle); }
