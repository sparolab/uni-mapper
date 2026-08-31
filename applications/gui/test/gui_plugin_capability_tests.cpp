#include "host/gui_plugin_host.hpp"

#include <cstdlib>
#include <iostream>
#include <utility>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  Check(argc == 5, "GUI plugin fixture paths are required");
  auto valid = open_lmm::GuiPluginHost::Load(argv[1]);
  Check(valid.IsOk(), "matching GUI services capability loads");
  auto host = std::move(valid).Value();
  Check(host->Start({}).IsOk() && host->IsOpen(),
        "matching GUI services capability starts");
  host->Stop();
  Check(!host->IsOpen(), "matching GUI services capability stops");
  host.reset();

  for (int index = 2; index != argc; ++index) {
    const auto rejected = open_lmm::GuiPluginHost::Load(argv[index]);
    Check(!rejected,
          "incompatible GUI capability is rejected before create");
  }
  std::cout << "GUI plugin capability tests passed\n";
  return 0;
}
