#include "fixtures/plugin_fixture_interface.hpp"

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/gui/gui_plugin.hpp>

#ifndef OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE
#define OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE 0
#endif

namespace {

class GuiPluginFixture final : public open_lmm::GuiPlugin {
 public:
  explicit GuiPluginFixture(PluginFixtureCounters* counters)
      : counters_(counters) {}

  open_lmm::Result<void> Start(open_lmm::GuiServices) override {
#if OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE == 4
    open_ = false;
#else
    open_ = true;
#endif
    return open_lmm::Result<void>::Ok();
  }
  [[nodiscard]] bool IsOpen() const override { return open_; }
  void RequestStop() override { open_ = false; }
  void Join() override {}

  PluginFixtureCounters* counters_ = nullptr;
  bool open_ = false;
};

void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  if (!config || config->struct_size < sizeof(OpenLmmPluginConfigV1)) {
    return nullptr;
  }
  auto* counters = static_cast<PluginFixtureCounters*>(config->host_context);
  if (counters) ++counters->creates;
  return static_cast<void*>(new GuiPluginFixture(counters));
}

void Destroy(void* value) noexcept {
  auto* plugin = static_cast<GuiPluginFixture*>(value);
  if (plugin && plugin->counters_) ++plugin->counters_->destroys;
  delete plugin;
}

#if OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE == 1
constexpr const char* kCapability = "gui:desktop";
#elif OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE == 2
constexpr const char* kCapability = "";
#elif OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE == 3
constexpr const char* kCapability = nullptr;
#else
constexpr const char* kCapability = "gui:services-v3";
#endif

const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "gui", "gui-fixture", &Create,
    &Destroy, kCapability, 1, "fixture-2"};

}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
