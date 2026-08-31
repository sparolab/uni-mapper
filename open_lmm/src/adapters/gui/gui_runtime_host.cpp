#include <open_lmm/gui/gui_runtime_host.hpp>

#include <adapters/gui/gui_controller_bridge.hpp>
#include <adapters/gui/gui_plugin_host.hpp>
#include <open_lmm/server/runtime_client.hpp>

#include <utility>

namespace open_lmm {

struct GuiRuntimeHost::Impl {
  std::shared_ptr<RuntimeClient> runtime;
  std::unique_ptr<GuiPluginHost> plugin;
};

GuiRuntimeHost::GuiRuntimeHost(std::unique_ptr<Impl> impl)
    : impl_(std::move(impl)) {}
GuiRuntimeHost::~GuiRuntimeHost() { Stop(); }
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
  auto feedback = impl->runtime->SetAlignmentFeedbackEnabled(true);
  if (!feedback) {
    return Result<std::unique_ptr<GuiRuntimeHost>>::Failure(feedback.GetError());
  }
  auto started = impl->plugin->Start(
      MakeGuiServices(impl->runtime, std::move(config_file_path)));
  if (!started) {
    (void)impl->runtime->SetAlignmentFeedbackEnabled(false);
    return Result<std::unique_ptr<GuiRuntimeHost>>::Failure(
        started.GetError());
  }
  return Result<std::unique_ptr<GuiRuntimeHost>>::Ok(
      std::unique_ptr<GuiRuntimeHost>(new GuiRuntimeHost(std::move(impl))));
}

void GuiRuntimeHost::Stop() {
  if (!impl_) return;
  if (impl_->plugin) impl_->plugin->Stop();
  if (impl_->runtime) (void)impl_->runtime->SetAlignmentFeedbackEnabled(false);
}

bool GuiRuntimeHost::IsOpen() const {
  return impl_ && impl_->plugin && impl_->plugin->IsOpen();
}

}  // namespace open_lmm
