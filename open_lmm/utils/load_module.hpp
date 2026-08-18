#pragma once

#include <dlfcn.h>

#include <cstdint>
#include <memory>
#include <string>

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/common/result.hpp>
#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

void open_so(const std::string& so_name);
void* load_symbol(const std::string& so_name, const std::string& symbol_name);

struct PluginMetadata {
  uint32_t abi_version = 0;
  std::string kind;
  std::string name;
  std::string capability;
  uint32_t config_schema_version = 0;
  std::string build_version;
};

namespace detail {

struct SharedLibraryHandle {
  explicit SharedLibraryHandle(void* value) : value(value) {}
  ~SharedLibraryHandle() {
    if (value) dlclose(value);
  }
  SharedLibraryHandle(const SharedLibraryHandle&) = delete;
  SharedLibraryHandle& operator=(const SharedLibraryHandle&) = delete;
  void* value = nullptr;
};

inline std::string PluginString(const char* value) {
  return value ? value : "";
}

}  // namespace detail

// Validates a runtime plugin without constructing an algorithm instance. This
// is used during session creation so an unavailable/incorrect plugin fails
// before any stage starts.
inline Result<PluginMetadata> inspect_plugin_v1(
    const std::string& so_name, const std::string& expected_kind) {
  LogWarning("[plugin ABI v1] same-toolchain compatibility mode: " + so_name);
  void* raw = dlopen(so_name.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!raw) {
    const char* error = dlerror();
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed("dlopen failed for " + so_name + ": " +
                                (error ? error : "unknown error"))
            .WithPlugin(so_name));
  }
  auto handle = std::make_shared<detail::SharedLibraryHandle>(raw);
  dlerror();
  void* symbol = dlsym(raw, OPEN_LMM_PLUGIN_ENTRY_SYMBOL);
  const char* symbol_error = dlerror();
  if (symbol_error || !symbol) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed(
            "missing '" OPEN_LMM_PLUGIN_ENTRY_SYMBOL "' in " + so_name +
            ": " + (symbol_error ? symbol_error : "null symbol"))
            .WithPlugin(so_name));
  }
  const OpenLmmPluginApiV1* api = nullptr;
  try {
    api = reinterpret_cast<OpenLmmPluginEntryV1>(symbol)();
  } catch (...) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed("plugin entry threw an exception in " + so_name)
            .WithPlugin(so_name));
  }
  if (!api || api->abi_version != OPEN_LMM_PLUGIN_ABI_VERSION_V1 ||
      !api->plugin_kind || !api->plugin_name || !api->create ||
      !api->destroy) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed(
            "plugin ABI-v1 entry is incomplete or incompatible in " + so_name)
            .WithPlugin(so_name));
  }
  const std::string actual_kind = detail::PluginString(api->plugin_kind);
  if (actual_kind != expected_kind) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed("plugin kind mismatch in " + so_name +
                                ": expected '" + expected_kind + "', got '" +
                                actual_kind + "'")
            .WithPlugin(so_name));
  }
  return Result<PluginMetadata>::Ok(
      {api->abi_version, actual_kind, detail::PluginString(api->plugin_name),
       detail::PluginString(api->capability), api->config_schema_version,
       detail::PluginString(api->build_version)});
}

// ABI v1 keeps the algorithm interface as a C++ virtual object and therefore
// supports plugins built with the same compiler, standard library, C++
// standard, Eigen and PCL ABI as the host.
template <typename Module>
Result<std::shared_ptr<Module>> load_plugin_v1(
    const std::string& so_name, const std::string& expected_kind,
    const std::string& config_json, PluginMetadata* metadata = nullptr,
    void* host_context = nullptr) {
  LogWarning("[plugin ABI v1] same-toolchain compatibility mode: " + so_name);
  void* raw = dlopen(so_name.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!raw) {
    const char* error = dlerror();
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("dlopen failed for " + so_name + ": " +
                                (error ? error : "unknown error"))
            .WithPlugin(so_name));
  }
  auto handle = std::make_shared<detail::SharedLibraryHandle>(raw);

  dlerror();
  void* symbol = dlsym(raw, OPEN_LMM_PLUGIN_ENTRY_SYMBOL);
  const char* symbol_error = dlerror();
  if (symbol_error || !symbol) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed(
            "missing '" OPEN_LMM_PLUGIN_ENTRY_SYMBOL "' in " + so_name +
            ": " + (symbol_error ? symbol_error : "null symbol"))
            .WithPlugin(so_name));
  }

  const OpenLmmPluginApiV1* api = nullptr;
  try {
    auto entry = reinterpret_cast<OpenLmmPluginEntryV1>(symbol);
    api = entry();
  } catch (...) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("plugin entry threw an exception in " + so_name)
            .WithPlugin(so_name));
  }
  if (!api) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("plugin entry returned nullptr in " + so_name)
            .WithPlugin(so_name));
  }
  if (api->abi_version != OPEN_LMM_PLUGIN_ABI_VERSION_V1) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed(
            "ABI version mismatch in " + so_name + ": expected " +
            std::to_string(OPEN_LMM_PLUGIN_ABI_VERSION_V1) + ", got " +
            std::to_string(api->abi_version))
            .WithPlugin(so_name));
  }
  const std::string actual_kind = detail::PluginString(api->plugin_kind);
  if (actual_kind != expected_kind) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("plugin kind mismatch in " + so_name +
                                ": expected '" + expected_kind + "', got '" +
                                actual_kind + "'")
            .WithPlugin(so_name));
  }
  if (!api->plugin_name || !api->create || !api->destroy) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed(
            "plugin API has missing name/create/destroy in " + so_name)
            .WithPlugin(so_name));
  }

  OpenLmmPluginConfigV1 config{
      static_cast<uint32_t>(sizeof(OpenLmmPluginConfigV1)),
      config_json.data(), config_json.size(), host_context};
  void* instance = nullptr;
  try {
    instance = api->create(&config);
  } catch (...) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("plugin create threw an exception in " + so_name)
            .WithPlugin(so_name));
  }
  if (!instance) {
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed("plugin create returned nullptr in " + so_name)
            .WithPlugin(so_name));
  }

  if (metadata) {
    *metadata = PluginMetadata{
        api->abi_version,
        actual_kind,
        detail::PluginString(api->plugin_name),
        detail::PluginString(api->capability),
        api->config_schema_version,
        detail::PluginString(api->build_version)};
  }

  // destroy runs while the captured handle is alive. dlclose occurs only when
  // this deleter's capture is released after the shared_ptr control block.
  auto deleter = [api, handle = std::move(handle)](Module* value) noexcept {
    if (!value) return;
    try {
      api->destroy(static_cast<void*>(value));
    } catch (...) {
      // A destroy function must not throw across the C boundary.
    }
  };
  return Result<std::shared_ptr<Module>>::Ok(
      std::shared_ptr<Module>(static_cast<Module*>(instance),
                              std::move(deleter)));
}

}  // namespace open_lmm
