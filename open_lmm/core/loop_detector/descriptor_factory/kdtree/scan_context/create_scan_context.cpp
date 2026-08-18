#include "scan_context.h"

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/common/plugin_api_v2.h>
#include <open_lmm/utils/descriptor_plugin_v2.hpp>
#include <open_lmm/utils/plugin_support.hpp>

#include <limits>
#include <memory>
#include <string>

using ScanContextV2 = open_lmm::DescriptorPluginV2Dispatcher<ScanContext>;

struct open_lmm_plugin_handle_v2 {
  ScanContextV2::State state;
};

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IDescriptorKdtree, ScanContext, ScanContextParams>(config,
                                                        "scan_context");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IDescriptorKdtree>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "descriptor", "scan_context",
    &Create, &Destroy, "descriptor:kdtree", 1, "open-lmm-1.0"};

constexpr uint64_t kCapabilities =
    OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2;
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_query_v2(open_lmm_plugin_descriptor_v2* descriptor) {
  if (!descriptor) {
    return open_lmm::DescriptorV2Status(
        OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "descriptor is null");
  }
  static const open_lmm_operation_descriptor_v2 operations[] = {
      {sizeof(open_lmm_operation_descriptor_v2),
       OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       open_lmm::DescriptorV2Text(OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2),
       OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2},
      {sizeof(open_lmm_operation_descriptor_v2),
       OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       open_lmm::DescriptorV2Text(OPEN_LMM_DESCRIPTOR_COMPARE_OPERATION_V2),
       OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2},
      {sizeof(open_lmm_operation_descriptor_v2),
       OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       open_lmm::DescriptorV2Text(OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2),
       OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2},
  };
  static const open_lmm_artifact_format_v2 formats[] = {
      {sizeof(open_lmm_artifact_format_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR,
       OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       open_lmm::DescriptorV2Text("scan_context"), 1, 0, 20}};
  *descriptor = open_lmm::DescriptorV2Header<open_lmm_plugin_descriptor_v2>();
  descriptor->plugin_kind = open_lmm::DescriptorV2Text("descriptor");
  descriptor->plugin_name = open_lmm::DescriptorV2Text("scan_context");
  descriptor->capability =
      open_lmm::DescriptorV2Text("descriptor.make,compare,index_key");
  descriptor->capability_bits = kCapabilities;
  descriptor->minimum_host_minor = 1;
  descriptor->plugin_id =
      open_lmm::DescriptorV2Text("open_lmm.descriptor.scan_context");
  descriptor->plugin_version = open_lmm::DescriptorV2Text("1.0.0");
  descriptor->operations = operations;
  descriptor->operation_count = std::size(operations);
  descriptor->schema_id = open_lmm::DescriptorV2Text("descriptor.scan_context");
  descriptor->schema_version = 1;
  descriptor->thread_safety = OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2;
  descriptor->cancellation = OPEN_LMM_CANCELLATION_COOPERATIVE_V2;
  descriptor->artifact_formats = formats;
  descriptor->artifact_format_count = std::size(formats);
  return open_lmm::DescriptorV2Status(OPEN_LMM_STATUS_OK_V2, "");
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_open_v2(const open_lmm_host_api_v2* host,
                        const open_lmm_config_view_v2* config,
                        open_lmm_plugin_handle_v2** out) {
  if (!host || !config || !out ||
      !open_lmm::DescriptorV2ValidHeader(host->struct_size, host->abi_major,
                                         sizeof(*host)) ||
      !open_lmm::DescriptorV2ValidHeader(config->struct_size,
                                         config->abi_major, sizeof(*config)) ||
      !open_lmm::DescriptorV2ValidHeader(config->json.struct_size,
                                         config->json.abi_major,
                                         sizeof(config->json)) ||
      (config->json.size != 0 && !config->json.data) ||
      config->json.size > std::numeric_limits<std::size_t>::max()) {
    return open_lmm::DescriptorV2Status(
        OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid descriptor open");
  }
  *out = nullptr;
  try {
    const std::string json(config->json.data ? config->json.data : "",
                           static_cast<std::size_t>(config->json.size));
    auto parsed = open_lmm::Config::FromJson(json, "scan_context ABI-v2");
    if (!parsed.is_valid()) {
      return open_lmm::DescriptorV2Status(
          OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid Scan Context JSON");
    }
    auto handle = std::make_unique<open_lmm_plugin_handle_v2>();
    handle->state.host = *host;
    handle->state.model =
        std::make_unique<ScanContext>(ScanContextParams(parsed));
    *out = handle.release();
    return open_lmm::DescriptorV2Status(OPEN_LMM_STATUS_OK_V2, "");
  } catch (...) {
    return open_lmm::DescriptorV2Status(
        OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
        "failed to create Scan Context ABI-v2 plugin");
  }
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_call_v2(open_lmm_plugin_handle_v2* handle,
                        const open_lmm_call_v2* call,
                        open_lmm_result_sink_v2* sink) {
  return ScanContextV2::Call(handle ? &handle->state : nullptr,
                             "scan_context", call, sink);
}

extern "C" void OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_close_v2(open_lmm_plugin_handle_v2* handle) {
  delete handle;
}
