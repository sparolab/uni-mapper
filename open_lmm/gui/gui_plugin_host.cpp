#include <open_lmm/gui/gui_plugin_host.hpp>
#include <open_lmm/utils/load_module.hpp>
#include <exception>
#include <utility>

namespace open_lmm {
GuiPluginHost::GuiPluginHost(std::shared_ptr<GuiPlugin> plugin)
    : plugin_(std::move(plugin)) {}
GuiPluginHost::~GuiPluginHost() { Stop(); }

Result<std::unique_ptr<GuiPluginHost>> GuiPluginHost::Load(const std::string& path) {
  auto loaded = load_plugin_v1<GuiPlugin>(
      path, "gui", "{}", nullptr, nullptr,
      PluginContractExpectation{.exact_capability = "gui:services-v3"});
  if (!loaded) return Result<std::unique_ptr<GuiPluginHost>>::Failure(loaded.GetError());
  return Result<std::unique_ptr<GuiPluginHost>>::Ok(
      std::make_unique<GuiPluginHost>(std::move(loaded).Value()));
}

Result<void> GuiPluginHost::Start(GuiServices services) {
  if (!plugin_) return Result<void>::Failure(Error::InvalidArgument("GUI plugin is null"));
  if (started_) return Result<void>::Failure(Error::InvalidArgument("GUI plugin is already started"));
  try {
    auto result = plugin_->Start(std::move(services));
    if (result) started_ = true;
    return result;
  } catch (const std::exception& e) {
    return Result<void>::Failure(Error::PluginLoadFailed(std::string("GUI start failed: ") + e.what()));
  } catch (...) {
    return Result<void>::Failure(Error::PluginLoadFailed("GUI start failed with unknown exception"));
  }
}

void GuiPluginHost::Stop() {
  if (!plugin_ || !started_) return;
  plugin_->RequestStop();
  plugin_->Join();
  started_ = false;
}
bool GuiPluginHost::IsOpen() const { return started_ && plugin_ && plugin_->IsOpen(); }
}  // namespace open_lmm
