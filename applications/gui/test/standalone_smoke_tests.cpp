#include "support/runtime/runtime_config_fixture.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

std::string ShellQuote(const std::string& value) {
  std::string quoted = "'";
  for (const char character : value) {
    if (character == '\'') {
      quoted += "'\\''";
    } else {
      quoted += character;
    }
  }
  return quoted + "'";
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 3) return 2;
  namespace fs = std::filesystem;
  const auto root = fs::temp_directory_path() /
                    ("open_lmm_gui_smoke_" + std::to_string(getpid()));
  const auto config = root / "config";
  open_lmm::test::WriteRuntimeConfigFixture(
      config, root / "data", root / "output");

  const std::string command = ShellQuote(argv[1]) + " " +
                              ShellQuote(config.string()) + " " +
                              ShellQuote(argv[2]);
  const int status = std::system(command.c_str());
  std::error_code cleanup_error;
  fs::remove_all(root, cleanup_error);
  if (status == -1 || !WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    std::cerr << "standalone GUI smoke failed: " << status << '\n';
    return 3;
  }
  return 0;
}
