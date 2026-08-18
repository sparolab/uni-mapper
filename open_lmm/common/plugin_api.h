#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define OPEN_LMM_PLUGIN_ABI_VERSION_V1 1u
#define OPEN_LMM_PLUGIN_ENTRY_SYMBOL "open_lmm_plugin_entry"

// Immutable configuration snapshot supplied by the host for one create call.
// json_data is UTF-8 JSON and is only valid for the duration of create().
typedef struct OpenLmmPluginConfigV1 {
  uint32_t struct_size;
  const char* json_data;
  size_t json_size;
  void* host_context;
} OpenLmmPluginConfigV1;

typedef struct OpenLmmPluginApiV1 {
  uint32_t abi_version;
  const char* plugin_kind;
  const char* plugin_name;
  void* (*create)(const OpenLmmPluginConfigV1* config);
  void (*destroy)(void* instance);

  // Capability metadata is descriptive in ABI v1. Compatibility is decided
  // by abi_version and plugin_kind before create() is called.
  const char* capability;
  uint32_t config_schema_version;
  const char* build_version;
} OpenLmmPluginApiV1;

typedef const OpenLmmPluginApiV1* (*OpenLmmPluginEntryV1)(void);

#ifdef __cplusplus
}
#endif
