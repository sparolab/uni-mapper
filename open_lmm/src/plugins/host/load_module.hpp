#pragma once

#include <dlfcn.h>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <utility>

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/common/result.hpp>
#include <foundation/logging/logging.hpp>

namespace open_lmm {

struct PluginMetadata {
  uint32_t abi_version = 0;
  std::string kind;
  std::string name;
  std::string capability;
  uint32_t config_schema_version = 0;
  std::string build_version;
};

struct PluginContractExpectation {
  std::optional<std::string_view> exact_capability;
  std::optional<std::string_view> exact_plugin_name;
  std::optional<uint32_t> exact_config_schema_version;
  std::optional<std::string_view> exact_build_generation;
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

inline Result<PluginMetadata> validate_plugin_v1(
    const OpenLmmPluginApiV1* api, const std::string& so_name,
    const std::string& expected_kind,
    PluginContractExpectation expectation = {}) {
  if (!api) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed("plugin entry returned nullptr in " + so_name)
            .WithPlugin(so_name));
  }
  if (api->abi_version != OPEN_LMM_PLUGIN_ABI_VERSION_V1) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed(
            "ABI version mismatch in " + so_name + ": expected '" +
            std::to_string(OPEN_LMM_PLUGIN_ABI_VERSION_V1) + "', got '" +
            std::to_string(api->abi_version) + "'")
            .WithPlugin(so_name));
  }
  if (!api->plugin_kind || !api->plugin_name || !api->create ||
      !api->destroy) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed(
            "plugin ABI-v1 entry is incomplete in " + so_name)
            .WithPlugin(so_name));
  }

  PluginMetadata metadata{
      api->abi_version,
      detail::PluginString(api->plugin_kind),
      detail::PluginString(api->plugin_name),
      detail::PluginString(api->capability),
      api->config_schema_version,
      detail::PluginString(api->build_version)};
  const auto mismatch = [&](std::string_view field, std::string expected,
                            std::string actual) {
    return Result<PluginMetadata>::Failure(
        Error::PluginLoadFailed(
            "plugin " + std::string(field) + " mismatch in " + so_name +
            ": expected '" + expected + "', got '" + actual + "'")
            .WithPlugin(so_name)
            .WithValidation("/metadata/" + std::string(field),
                            std::move(expected), std::move(actual),
                            metadata.config_schema_version));
  };
  if (metadata.kind != expected_kind) {
    return mismatch("kind", expected_kind, metadata.kind);
  }
  if (expectation.exact_capability &&
      metadata.capability != *expectation.exact_capability) {
    return mismatch("capability",
                    std::string(*expectation.exact_capability),
                    metadata.capability);
  }
  if (expectation.exact_plugin_name &&
      metadata.name != *expectation.exact_plugin_name) {
    return mismatch("name", std::string(*expectation.exact_plugin_name),
                    metadata.name);
  }
  if (expectation.exact_config_schema_version &&
      metadata.config_schema_version !=
          *expectation.exact_config_schema_version) {
    return mismatch(
        "config_schema_version",
        std::to_string(*expectation.exact_config_schema_version),
        std::to_string(metadata.config_schema_version));
  }
  if (expectation.exact_build_generation &&
      metadata.build_version != *expectation.exact_build_generation) {
    return mismatch("build_generation",
                    std::string(*expectation.exact_build_generation),
                    metadata.build_version);
  }
  return Result<PluginMetadata>::Ok(std::move(metadata));
}

// Validates a runtime plugin without constructing an algorithm instance. This
// is used during session creation so an unavailable/incorrect plugin fails
// before any stage starts.
inline Result<PluginMetadata> inspect_plugin_v1(
    const std::string& so_name, const std::string& expected_kind,
    PluginContractExpectation expectation = {}) {
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
  return validate_plugin_v1(api, so_name, expected_kind, expectation);
}

// ABI v1 keeps the algorithm interface as a C++ virtual object and therefore
// supports plugins built with the same compiler, standard library, C++
// standard, Eigen and PCL ABI as the host.
template <typename Module>
Result<std::shared_ptr<Module>> load_plugin_v1(
    const std::string& so_name, const std::string& expected_kind,
    const std::string& config_json, PluginMetadata* metadata = nullptr,
    void* host_context = nullptr, PluginContractExpectation expectation = {}) {
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
  auto validated = validate_plugin_v1(api, so_name, expected_kind,
                                      expectation);
  if (!validated) {
    return Result<std::shared_ptr<Module>>::Failure(validated.GetError());
  }
  PluginMetadata validated_metadata = std::move(validated).Value();

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
    *metadata = std::move(validated_metadata);
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
