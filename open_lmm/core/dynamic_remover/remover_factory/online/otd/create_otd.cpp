#include "otd.hpp"
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/utils/plugin_support.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/remover_plugin_v2_support.hpp>

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IOnlineRemoverPlugin, OTD, OTDParams>(config, "otd");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IOnlineRemoverPlugin>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "dynamic_remover_online", "otd",
    &Create, &Destroy, "dynamic_remover:online", 1, "open-lmm-1.0"};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}

extern "C" open_lmm_status_v2 open_lmm_plugin_query_v2(
    open_lmm_plugin_descriptor_v2* descriptor) {
  if (!descriptor) {
    return open_lmm::remover_v2_support::Status(
        OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "null descriptor");
  }
  open_lmm::remover_v2_support::FillDescriptor(
      descriptor, "otd", OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2);
  return open_lmm::remover_v2_support::Status(OPEN_LMM_STATUS_OK_V2);
}
extern "C" open_lmm_status_v2 open_lmm_plugin_open_v2(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** output) {
  return open_lmm::remover_v2_support::Open<
      IOnlineRemoverPlugin, OTD, OTDParams, true>(host, config, output);
}
extern "C" open_lmm_status_v2 open_lmm_plugin_call_v2(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* sink) {
  return open_lmm::remover_v2_support::Call<
      IOnlineRemoverPlugin, OTD, OTDParams, true>(handle, call, sink);
}
extern "C" void open_lmm_plugin_close_v2(
    open_lmm_plugin_handle_v2* handle) {
  open_lmm::remover_v2_support::Close<
      IOnlineRemoverPlugin, OTD, OTDParams, true>(handle);
}
