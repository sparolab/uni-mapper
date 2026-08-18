#include "plugin_fixture_interface.hpp"

#include <open_lmm/common/plugin_api.h>

#ifndef OPEN_LMM_PLUGIN_FIXTURE_MODE
#define OPEN_LMM_PLUGIN_FIXTURE_MODE 0
#endif

#if OPEN_LMM_PLUGIN_FIXTURE_MODE == 4
extern "C" int open_lmm_plugin_fixture_without_entry() { return 1; }
#else
namespace {

class PluginFixtureImpl final : public PluginFixture {
 public:
  explicit PluginFixtureImpl(PluginFixtureCounters* counters)
      : counters_(counters) {}
  int Value() const override { return 42; }
  PluginFixtureCounters* counters_;
};

void* Create(const OpenLmmPluginConfigV1* config) noexcept {
#if OPEN_LMM_PLUGIN_FIXTURE_MODE == 2
  (void)config;
  return nullptr;
#else
  if (!config || config->struct_size < sizeof(OpenLmmPluginConfigV1)) {
    return nullptr;
  }
  auto* counters = static_cast<PluginFixtureCounters*>(config->host_context);
  if (counters) ++counters->creates;
  PluginFixture* instance = new PluginFixtureImpl(counters);
  return static_cast<void*>(instance);
#endif
}

[[maybe_unused]] void Destroy(void* value) noexcept {
  auto* instance = static_cast<PluginFixture*>(value);
  auto* implementation = static_cast<PluginFixtureImpl*>(instance);
  if (implementation->counters_) ++implementation->counters_->destroys;
  delete instance;
}

const OpenLmmPluginApiV1 kApi{
#if OPEN_LMM_PLUGIN_FIXTURE_MODE == 1
    999u,
#else
    OPEN_LMM_PLUGIN_ABI_VERSION_V1,
#endif
    "test", "fixture", &Create,
#if OPEN_LMM_PLUGIN_FIXTURE_MODE == 3
    nullptr,
#else
    &Destroy,
#endif
    "test:lifecycle", 1, "fixture-1"};

}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
#endif
