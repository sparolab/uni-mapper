#include "fixture_trace.hpp"

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/gui/gui_plugin.hpp>

#include <fcntl.h>
#include <unistd.h>

#include <cstdlib>
#include <stdexcept>
#include <utility>

#ifndef OPEN_LMM_GUI_PROBE_FIXTURE_MODE
#define OPEN_LMM_GUI_PROBE_FIXTURE_MODE 0
#endif

namespace {

GuiLoaderProbeTrace* g_trace = nullptr;

class ProbeGui final : public open_lmm::GuiPlugin {
 public:
  explicit ProbeGui(GuiLoaderProbeTrace* trace) : trace_(trace) {}

  open_lmm::Result<void> Start(open_lmm::GuiServices) override {
    if (trace_) RecordProbeOrder(trace_->start_order, *trace_);
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 17
    return open_lmm::Result<void>::Failure(
        open_lmm::Error::PluginLoadFailed("probe start result failure"));
#elif OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 18
    throw std::runtime_error("probe start standard failure");
#else
    open_ = true;
    return open_lmm::Result<void>::Ok();
#endif
  }

  [[nodiscard]] bool IsOpen() const override { return open_; }

  void RequestStop() override {
    if (trace_) RecordProbeOrder(trace_->request_stop_order, *trace_);
    open_ = false;
  }

  void Join() override {
    if (trace_) RecordProbeOrder(trace_->join_order, *trace_);
  }

 private:
  GuiLoaderProbeTrace* trace_ = nullptr;
  bool open_ = false;
};

[[maybe_unused]] void* CreateGui(const OpenLmmPluginConfigV1* config) {
  g_trace = config ? static_cast<GuiLoaderProbeTrace*>(config->host_context)
                   : nullptr;
  if (g_trace) RecordProbeOrder(g_trace->create_order, *g_trace);
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 10
  return nullptr;
#elif OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 11
  throw std::runtime_error("probe create failure");
#else
  return static_cast<void*>(new ProbeGui(g_trace));
#endif
}

[[maybe_unused]] void DestroyGui(void* instance) {
  if (g_trace) RecordProbeOrder(g_trace->destroy_order, *g_trace);
  delete static_cast<ProbeGui*>(instance);
}

constexpr std::uint32_t kAbi =
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 4
    OPEN_LMM_PLUGIN_ABI_VERSION_V1 + 1;
#else
    OPEN_LMM_PLUGIN_ABI_VERSION_V1;
#endif

const char* Kind() {
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 5
  return "descriptor";
#elif OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 6
  return nullptr;
#else
  return "gui";
#endif
}

const char* Name() {
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 7
  return nullptr;
#else
  return "loader-probe";
#endif
}

const char* Capability() {
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 1
  return "gui:services-v2";
#elif OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 3
  return "";
#else
  return "gui:services-v3";
#endif
}

[[maybe_unused]] const OpenLmmPluginApiV1 kApi{
    kAbi,
    Kind(),
    Name(),
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 8
    nullptr,
#else
    &CreateGui,
#endif
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 9
    nullptr,
#else
    &DestroyGui,
#endif
    Capability(),
    1,
    "open-lmm-3.0"};

__attribute__((destructor)) void RecordUnload() {
  if (g_trace) RecordProbeOrder(g_trace->unload_order, *g_trace);
  const char* trace_file =
      std::getenv("OPEN_LMM_GUI_PROBE_UNLOAD_TRACE_FILE");
  if (!trace_file || !*trace_file) return;
  const int descriptor =
      open(trace_file, O_WRONLY | O_CREAT | O_APPEND | O_CLOEXEC, 0600);
  if (descriptor == -1) return;
  constexpr char kUnloadMarker[] = "unload\n";
  const ssize_t written =
      write(descriptor, kUnloadMarker, sizeof(kUnloadMarker) - 1);
  (void)written;
  (void)close(descriptor);
}

}  // namespace

#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE != 16
extern "C" __attribute__((visibility("default")))
const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
#if OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 13
  return nullptr;
#elif OPEN_LMM_GUI_PROBE_FIXTURE_MODE == 14
  throw std::runtime_error("probe entry failure");
#else
  return &kApi;
#endif
}
#endif
