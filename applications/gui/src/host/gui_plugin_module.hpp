#pragma once

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/common/result.hpp>
#include <open_lmm/gui/gui_plugin.hpp>

#include <memory>
#include <string>

namespace open_lmm {

class GuiPluginModule final {
 public:
  static Result<std::unique_ptr<GuiPluginModule>> Load(
      const std::string& path, void* host_context = nullptr);

  ~GuiPluginModule();
  GuiPluginModule(const GuiPluginModule&) = delete;
  GuiPluginModule& operator=(const GuiPluginModule&) = delete;
  GuiPluginModule(GuiPluginModule&& other) noexcept;
  GuiPluginModule& operator=(GuiPluginModule&& other) = delete;

  [[nodiscard]] GuiPlugin& Plugin() const { return *instance_; }

 private:
  GuiPluginModule(void* handle, const OpenLmmPluginApiV1* api,
                  GuiPlugin* instance)
      : handle_(handle), api_(api), instance_(instance) {}

  void Reset() noexcept;

  void* handle_ = nullptr;
  const OpenLmmPluginApiV1* api_ = nullptr;
  GuiPlugin* instance_ = nullptr;
};

}  // namespace open_lmm
