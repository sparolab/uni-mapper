#include <open_lmm/common/plugin_api.h>
#include <open_lmm/gui/gui_plugin.hpp>

namespace {

class CompatGuiPlugin final : public open_lmm::GuiPlugin {
 public:
  open_lmm::Result<void> Start(open_lmm::GuiServices) override {
    open_ = true;
    return open_lmm::Result<void>::Ok();
  }

  [[nodiscard]] bool IsOpen() const override { return open_; }
  void RequestStop() override { open_ = false; }
  void Join() override {}

 private:
  bool open_ = false;
};

void* Create(const OpenLmmPluginConfigV1* config) {
  if (!config || config->struct_size < sizeof(OpenLmmPluginConfigV1)) {
    return nullptr;
  }
  return static_cast<void*>(new CompatGuiPlugin());
}

void Destroy(void* instance) {
  delete static_cast<CompatGuiPlugin*>(instance);
}

const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1,
    "gui",
    "compat-consumer",
    &Create,
    &Destroy,
    "gui:services-v3",
    1,
    "open-lmm-3.0"};

}  // namespace

extern "C" __attribute__((visibility("default")))
const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
