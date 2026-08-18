#include <iostream>
#include <string>

#include <open_lmm/server/map_server.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>
#include <open_lmm/utils/logging.hpp>

int main(int argc, char** argv) {
  open_lmm::InitializeLogging();
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0] << " <config_dir_path>\n";
    return 0;
  }
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_dir_path>" << std::endl;
    return 1;
  }

  auto configured = open_lmm::LoadBootstrapConfig(argv[1]);
  if (!configured) {
    open_lmm::LogError(configured.GetError().Message());
    return 1;
  }

  open_lmm::MapServer map_server(std::move(configured).Value());
  auto result = map_server.process();
  if (!result) {
    open_lmm::LogError(result.GetError().Message());
    return 1;
  }

  return 0;
}
