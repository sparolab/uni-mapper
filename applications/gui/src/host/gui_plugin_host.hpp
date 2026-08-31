#pragma once
#include <open_lmm/gui/gui_plugin.hpp>
#include <memory>
#include <string>

namespace open_lmm {
class GuiPluginModule;

class GuiPluginHost {
 public:
  explicit GuiPluginHost(std::shared_ptr<GuiPlugin> plugin);
  ~GuiPluginHost();
  GuiPluginHost(const GuiPluginHost&) = delete;
  GuiPluginHost& operator=(const GuiPluginHost&) = delete;
  static Result<std::unique_ptr<GuiPluginHost>> Load(
      const std::string& path, void* host_context = nullptr);
  Result<void> Start(GuiServices services);
  void Stop();
  [[nodiscard]] bool IsOpen() const;

 private:
  explicit GuiPluginHost(std::unique_ptr<GuiPluginModule> module);
  [[nodiscard]] GuiPlugin* Plugin() const;

  std::shared_ptr<GuiPlugin> plugin_;
  std::unique_ptr<GuiPluginModule> module_;
  bool started_ = false;
};
}  // namespace open_lmm
