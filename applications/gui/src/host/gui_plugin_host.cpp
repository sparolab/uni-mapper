#include "host/gui_plugin_host.hpp"
#include "host/gui_plugin_module.hpp"
#include <exception>
#include <utility>

namespace open_lmm {
GuiPluginHost::GuiPluginHost(std::shared_ptr<GuiPlugin> plugin)
    : plugin_(std::move(plugin)) {}
GuiPluginHost::GuiPluginHost(std::unique_ptr<GuiPluginModule> module)
    : module_(std::move(module)) {}
GuiPluginHost::~GuiPluginHost() { Stop(); }

Result<std::unique_ptr<GuiPluginHost>> GuiPluginHost::Load(
    const std::string& path, void* host_context) {
  auto loaded = GuiPluginModule::Load(path, host_context);
  if (!loaded) return Result<std::unique_ptr<GuiPluginHost>>::Failure(loaded.GetError());
  return Result<std::unique_ptr<GuiPluginHost>>::Ok(
      std::unique_ptr<GuiPluginHost>(
          new GuiPluginHost(std::move(loaded).Value())));
}

Result<void> GuiPluginHost::Start(GuiServices services) {
  auto* plugin = Plugin();
  if (!plugin) return Result<void>::Failure(Error::InvalidArgument("GUI plugin is null"));
  if (started_) return Result<void>::Failure(Error::InvalidArgument("GUI plugin is already started"));
  try {
    auto result = plugin->Start(std::move(services));
    if (result) started_ = true;
    return result;
  } catch (const std::exception& e) {
    return Result<void>::Failure(Error::PluginLoadFailed(std::string("GUI start failed: ") + e.what()));
  } catch (...) {
    return Result<void>::Failure(Error::PluginLoadFailed("GUI start failed with unknown exception"));
  }
}

void GuiPluginHost::Stop() {
  auto* plugin = Plugin();
  if (!plugin || !started_) return;
  plugin->RequestStop();
  plugin->Join();
  started_ = false;
}
GuiPlugin* GuiPluginHost::Plugin() const {
  return module_ ? &module_->Plugin() : plugin_.get();
}
bool GuiPluginHost::IsOpen() const {
  auto* plugin = Plugin();
  return started_ && plugin && plugin->IsOpen();
}
}  // namespace open_lmm
