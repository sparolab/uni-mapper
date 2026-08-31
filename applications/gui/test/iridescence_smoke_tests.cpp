#include "support/runtime/runtime_config_fixture.hpp"

#include <open_lmm/common/plugin_api.h>

#include <dlfcn.h>

#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <csignal>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>

namespace {

constexpr auto kStartupObservation = std::chrono::seconds(3);
constexpr auto kShutdownTimeout = std::chrono::seconds(10);
constexpr auto kPollInterval = std::chrono::milliseconds(10);

bool ValidatePluginMetadata(const char* path) {
  void* handle = dlopen(path, RTLD_NOW | RTLD_LOCAL);
  if (!handle) return false;
  dlerror();
  auto* symbol = dlsym(handle, OPEN_LMM_PLUGIN_ENTRY_SYMBOL);
  const char* symbol_error = dlerror();
  const OpenLmmPluginApiV1* api = nullptr;
  if (!symbol_error && symbol) {
    try {
      api = reinterpret_cast<OpenLmmPluginEntryV1>(symbol)();
    } catch (...) {
      api = nullptr;
    }
  }
  const bool valid =
      api && api->abi_version == OPEN_LMM_PLUGIN_ABI_VERSION_V1 &&
      api->plugin_kind && std::string(api->plugin_kind) == "gui" &&
      api->plugin_name && std::string(api->plugin_name) == "iridescence" &&
      api->capability &&
      std::string(api->capability) == "gui:services-v3" &&
      api->config_schema_version == 1 && api->build_version &&
      std::string(api->build_version) == "open-lmm-3.0";
  (void)dlclose(handle);
  return valid;
}

bool WaitForExit(pid_t child, std::chrono::steady_clock::duration timeout,
                 int& status) {
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    const auto result = waitpid(child, &status, WNOHANG);
    if (result == child) return true;
    if (result == -1) return false;
    std::this_thread::sleep_for(kPollInterval);
  }
  return false;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  if (!ValidatePluginMetadata(argv[2])) {
    std::cerr << "Iridescence plugin metadata does not match the v3 golden\n";
    return 7;
  }
  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path() /
                    ("open_lmm_gui_iridescence_smoke_" +
                     std::to_string(getpid()));
  const auto config = root / "config";
  open_lmm::test::WriteRuntimeConfigFixture(
      config, root / "data", root / "output");

  const pid_t child = fork();
  if (child == -1) {
    std::cerr << "failed to fork standalone GUI process\n";
    return 3;
  }
  if (child == 0) {
    execl(argv[1], argv[1], config.c_str(), argv[2],
          static_cast<char*>(nullptr));
    _exit(127);
  }

  int status = 0;
  if (WaitForExit(child, kStartupObservation, status)) {
    std::cerr << "Iridescence GUI exited before the display smoke completed: "
              << status << '\n';
    std::error_code cleanup_error;
    fs::remove_all(root, cleanup_error);
    return 4;
  }

  if (kill(child, SIGTERM) != 0 ||
      !WaitForExit(child, kShutdownTimeout, status)) {
    kill(child, SIGKILL);
    waitpid(child, &status, 0);
    std::cerr << "Iridescence GUI did not stop and join within the timeout\n";
    std::error_code cleanup_error;
    fs::remove_all(root, cleanup_error);
    return 5;
  }

  std::error_code cleanup_error;
  fs::remove_all(root, cleanup_error);
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "Iridescence GUI shutdown failed: " << status << '\n';
    return 6;
  }
  return 0;
}
