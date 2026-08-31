#include "host/gui_plugin_module.hpp"

#include <dlfcn.h>

#include <cstdint>
#include <exception>
#include <memory>
#include <string>
#include <utility>

namespace open_lmm {
namespace {

constexpr const char* kGuiKind = "gui";
constexpr const char* kGuiCapability = "gui:services-v3";

class PendingHandle final {
 public:
  explicit PendingHandle(void* handle) : handle_(handle) {}
  ~PendingHandle() {
    if (handle_) (void)dlclose(handle_);
  }
  PendingHandle(const PendingHandle&) = delete;
  PendingHandle& operator=(const PendingHandle&) = delete;

  [[nodiscard]] void* Get() const { return handle_; }
  [[nodiscard]] void* Release() { return std::exchange(handle_, nullptr); }

 private:
  void* handle_ = nullptr;
};

Result<std::unique_ptr<GuiPluginModule>> LoadFailure(
    const std::string& path, std::string message) {
  return Result<std::unique_ptr<GuiPluginModule>>::Failure(
      Error::PluginLoadFailed(std::move(message)).WithPlugin(path));
}

std::string SafeString(const char* value) { return value ? value : ""; }

}  // namespace

Result<std::unique_ptr<GuiPluginModule>> GuiPluginModule::Load(
    const std::string& path, void* host_context) {
  void* raw_handle = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!raw_handle) {
    const char* error = dlerror();
    return LoadFailure(path, "dlopen failed for " + path + ": " +
                                 (error ? error : "unknown error"));
  }
  PendingHandle handle(raw_handle);

  dlerror();
  void* raw_symbol = dlsym(handle.Get(), OPEN_LMM_PLUGIN_ENTRY_SYMBOL);
  const char* symbol_error = dlerror();
  if (symbol_error || !raw_symbol) {
    return LoadFailure(
        path, "missing '" OPEN_LMM_PLUGIN_ENTRY_SYMBOL "' in " + path +
                  ": " + (symbol_error ? symbol_error : "null symbol"));
  }

  const OpenLmmPluginApiV1* api = nullptr;
  try {
    api = reinterpret_cast<OpenLmmPluginEntryV1>(raw_symbol)();
  } catch (const std::exception& error) {
    return LoadFailure(path, "plugin entry threw an exception in " + path +
                                 ": " + error.what());
  } catch (...) {
    return LoadFailure(path,
                       "plugin entry threw an unknown exception in " + path);
  }

  if (!api) return LoadFailure(path, "plugin entry returned nullptr in " + path);
  if (api->abi_version != OPEN_LMM_PLUGIN_ABI_VERSION_V1) {
    return LoadFailure(path, "plugin ABI version mismatch in " + path);
  }
  if (!api->plugin_kind || !api->plugin_name || !api->create || !api->destroy) {
    return LoadFailure(path, "plugin ABI-v1 entry is incomplete in " + path);
  }
  if (SafeString(api->plugin_kind) != kGuiKind) {
    return LoadFailure(path, "plugin kind mismatch in " + path);
  }
  if (!api->capability || SafeString(api->capability) != kGuiCapability) {
    return LoadFailure(path, "plugin capability mismatch in " + path);
  }

  OpenLmmPluginConfigV1 config{
      static_cast<std::uint32_t>(sizeof(OpenLmmPluginConfigV1)), "{}", 2,
      host_context};
  void* raw_instance = nullptr;
  try {
    raw_instance = api->create(&config);
  } catch (const std::exception& error) {
    return LoadFailure(path, "plugin create threw an exception in " + path +
                                 ": " + error.what());
  } catch (...) {
    return LoadFailure(path,
                       "plugin create threw an unknown exception in " + path);
  }
  if (!raw_instance) {
    return LoadFailure(path, "plugin create returned nullptr in " + path);
  }

  return Result<std::unique_ptr<GuiPluginModule>>::Ok(
      std::unique_ptr<GuiPluginModule>(new GuiPluginModule(
          handle.Release(), api, static_cast<GuiPlugin*>(raw_instance))));
}

GuiPluginModule::GuiPluginModule(GuiPluginModule&& other) noexcept
    : handle_(std::exchange(other.handle_, nullptr)),
      api_(std::exchange(other.api_, nullptr)),
      instance_(std::exchange(other.instance_, nullptr)) {}

GuiPluginModule::~GuiPluginModule() { Reset(); }

void GuiPluginModule::Reset() noexcept {
  if (instance_) {
    try {
      api_->destroy(static_cast<void*>(instance_));
    } catch (...) {
      // ABI-v1 destroy functions must not throw across the C boundary.
    }
    instance_ = nullptr;
  }
  api_ = nullptr;
  if (handle_) {
    (void)dlclose(handle_);
    handle_ = nullptr;
  }
}

}  // namespace open_lmm
