#include "support/benchmark/fixture_generator.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

int main(int argc, char** argv) {
  if (argc != 3 && argc != 5) {
    std::cerr << "usage: open_lmm_benchmark_generate_fixture "
                 "<small-v1|medium-v1> <new-output-directory> "
                 "[--map-update sequential|parallel]\n";
    return 2;
  }
  try {
    open_lmm::test::benchmark::FixtureGenerationOptions options;
    if (argc == 5) {
      if (std::string(argv[3]) != "--map-update") {
        throw std::invalid_argument("unknown fixture option");
      }
      const std::string mode = argv[4];
      if (mode != "sequential" && mode != "parallel") {
        throw std::invalid_argument(
            "MapUpdate mode must be sequential or parallel");
      }
      options.enable_map_update = true;
      options.parallel_map_update = mode == "parallel";
    }
    const auto fixture = open_lmm::test::benchmark::GenerateFixture(
        std::filesystem::path(argv[2]), argv[1], options);
    std::cout << fixture.manifest_path << '\n';
    return 0;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 2;
  }
}
