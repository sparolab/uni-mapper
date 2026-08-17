#pragma once
#include <open_lmm/gui/gui_plugin.hpp>
#include <memory>
#include <string>

namespace open_lmm {
class GuiPluginHost {
 public:
  explicit GuiPluginHost(std::shared_ptr<GuiPlugin> plugin);
  ~GuiPluginHost();
  GuiPluginHost(const GuiPluginHost&) = delete;
  GuiPluginHost& operator=(const GuiPluginHost&) = delete;
  static Result<std::unique_ptr<GuiPluginHost>> Load(const std::string& path);
  Result<void> Start(GuiServices services);
  void Stop();
  [[nodiscard]] bool IsOpen() const;
 private:
  std::shared_ptr<GuiPlugin> plugin_;
  bool started_ = false;
};
}  // namespace open_lmm
