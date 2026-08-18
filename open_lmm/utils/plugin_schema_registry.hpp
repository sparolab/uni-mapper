#pragma once

#include <open_lmm/common/plugin_host_v2.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <span>

namespace open_lmm {

// Decodes one bounded, self-described plugin fragment. The returned value is
// independent of the borrowed query/call buffers owned by the plugin.
Result<SchemaFragment> DecodePluginSchemaFragment(
    const PluginV2Metadata& metadata, SchemaLimits limits = {});

// Creates an immutable registry for one session. The process-global builtin
// registry is copied, never mutated.
Result<SchemaRegistry> BuildSessionSchemaRegistry(
    std::span<const PluginV2Metadata> plugins, SchemaLimits limits = {});

}  // namespace open_lmm
