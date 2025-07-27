// open_lmm
#include <open_lmm/server/map_server.hpp>
#include <tqdmcpp/tqdmcpp.hpp>

#include "utils/config.hpp"

int main(int argc, char** argv) {
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_dir_path>" << std::endl;
    return 1;
  }
  std::string config_dir_path = argv[1];
  open_lmm::GlobalConfig::instance(config_dir_path);
  open_lmm::MapServer map_server;
  map_server.process();

  return 0;
}