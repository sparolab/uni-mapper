#include <open_lmm/gui/gui_runtime_host.hpp>
#include <open_lmm/server/runtime_client.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <thread>

namespace {

volatile std::sig_atomic_t g_stop_requested = 0;

extern "C" void RequestStop(int) { g_stop_requested = 1; }

std::filesystem::path DefaultPluginPath() {
  std::error_code error;
  const auto executable = std::filesystem::read_symlink("/proc/self/exe", error);
  if (error || executable.empty()) return {};
  return executable.parent_path().parent_path() /
         OPEN_LMM_GUI_INSTALL_LIBDIR / "libopen_lmm_iridescence_gui.so";
}

void WriteError(const std::string& message) { std::cerr << message << '\n'; }

}  // namespace

int main(int argc, char** argv) {
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0]
              << " <config_dir_path> [gui_plugin_path]\n";
    return 0;
  }
  if (argc != 2 && argc != 3) {
    std::cerr << "Usage: " << argv[0]
              << " <config_dir_path> [gui_plugin_path]\n";
    return 1;
  }

  const std::filesystem::path config_directory = argv[1];
  const std::filesystem::path plugin_path =
      argc == 3 ? std::filesystem::path(argv[2]) : DefaultPluginPath();
  if (plugin_path.empty() || !std::filesystem::is_regular_file(plugin_path)) {
    WriteError("GUI plugin was not found at " + plugin_path.string());
    return 1;
  }

  auto runtime = std::make_shared<open_lmm::RuntimeClient>(4);
  auto opened = runtime->Open({config_directory, "gui", std::nullopt});
  if (!opened) {
    WriteError(opened.GetError().Message());
    return 1;
  }

  auto loaded = open_lmm::GuiRuntimeHost::LoadAndStart(
      plugin_path.string(), runtime,
      (config_directory / "config.json").string());
  if (!loaded) {
    WriteError(loaded.GetError().Message());
    const auto closed = runtime->Close();
    if (!closed) WriteError("GUI cleanup failed: " + closed.GetError().Message());
    return 1;
  }

  std::signal(SIGINT, RequestStop);
  std::signal(SIGTERM, RequestStop);

  auto host = std::move(loaded).Value();
  while (!g_stop_requested && host->IsOpen()) {
    std::this_thread::sleep_for(std::chrono::milliseconds(25));
  }
  host->Stop();
  host.reset();

  const auto closed = runtime->Close();
  if (!closed) {
    WriteError(closed.GetError().Message());
    return 1;
  }
  return 0;
}
