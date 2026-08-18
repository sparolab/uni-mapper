#include <open_lmm/gui/gui_runtime_host.hpp>

#include <open_lmm/gui/gui_controller_bridge.hpp>
#include <open_lmm/gui/gui_plugin_host.hpp>

#include <utility>

namespace open_lmm {

struct GuiRuntimeHost::Impl {
  std::shared_ptr<RuntimeClient> runtime;
  std::unique_ptr<GuiPluginHost> plugin;
};

GuiRuntimeHost::GuiRuntimeHost(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
GuiRuntimeHost::~GuiRuntimeHost() = default;
GuiRuntimeHost::GuiRuntimeHost(GuiRuntimeHost&&) noexcept = default;
GuiRuntimeHost& GuiRuntimeHost::operator=(GuiRuntimeHost&&) noexcept = default;

Result<std::unique_ptr<GuiRuntimeHost>> GuiRuntimeHost::LoadAndStart(
    const std::string& plugin_path,
    std::shared_ptr<RuntimeClient> runtime,
    std::string config_file_path) {
  if (!runtime) {
    return Result<std::unique_ptr<GuiRuntimeHost>>::Failure(
        Error::InvalidArgument("GUI runtime client must not be null"));
  }
  auto loaded = GuiPluginHost::Load(plugin_path);
  if (!loaded) {
    return Result<std::unique_ptr<GuiRuntimeHost>>::Failure(loaded.GetError());
  }
  auto impl = std::make_unique<Impl>();
  impl->runtime = std::move(runtime);
  impl->plugin = std::move(loaded).Value();
  auto started = impl->plugin->Start(
      MakeGuiServices(impl->runtime, std::move(config_file_path)));
  if (!started) {
    return Result<std::unique_ptr<GuiRuntimeHost>>::Failure(
        started.GetError());
  }
  return Result<std::unique_ptr<GuiRuntimeHost>>::Ok(
      std::unique_ptr<GuiRuntimeHost>(new GuiRuntimeHost(std::move(impl))));
}

void GuiRuntimeHost::Stop() {
  if (impl_ && impl_->plugin) impl_->plugin->Stop();
}

bool GuiRuntimeHost::IsOpen() const {
  return impl_ && impl_->plugin && impl_->plugin->IsOpen();
}

}  // namespace open_lmm
