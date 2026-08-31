#include <open_lmm/gui/gui_plugin.hpp>
#include <open_lmm/gui/gui_runtime_host.hpp>
#include <open_lmm/server/runtime_client.hpp>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

#include <unistd.h>

#include "runtime_config_fixture.hpp"

int main(int argc, char** argv) {
  static_assert(std::is_move_constructible_v<open_lmm::GuiRuntimeHost>);
  static_assert(!std::is_copy_constructible_v<open_lmm::GuiRuntimeHost>);
  if (argc != 2) return 2;

  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path() /
                    ("open_lmm_gui_compat_consumer_" +
                     std::to_string(getpid()));
  const auto config = root / "config";
  WriteRuntimeConfigFixture(config, root / "data", root / "output");

  auto runtime = std::make_shared<open_lmm::RuntimeClient>(2);
  const auto opened = runtime->Open({config, "gui-compat", std::nullopt});
  if (!opened) return 3;

  auto loaded = open_lmm::GuiRuntimeHost::LoadAndStart(
      argv[1], runtime, (config / "config.json").string());
  if (!loaded) {
    (void)runtime->Close();
    return 4;
  }
  auto host = std::move(loaded).Value();
  if (!host->IsOpen()) return 5;
  host->Stop();
  if (host->IsOpen()) return 6;
  host.reset();

  const auto closed = runtime->Close();
  std::error_code cleanup_error;
  fs::remove_all(root, cleanup_error);
  return closed ? 0 : 7;
}
