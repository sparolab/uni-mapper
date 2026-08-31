#include "replay_contract.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
namespace replay = open_lmm::test::replay;

namespace {

struct Arguments {
  std::string kind;
  fs::path input;
};

Arguments ParseArguments(int argc, char** argv) {
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    auto consume = [&](std::string& destination) {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      destination = argv[index];
    };
    if (option == "--kind") {
      consume(arguments.kind);
    } else if (option == "--input") {
      std::string path;
      consume(path);
      arguments.input = path;
    } else if (option == "--help") {
      std::cout << "usage: open_lmm_replay_validate --kind case|report|baseline "
                   "--input DOCUMENT.json\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.kind.empty() || arguments.input.empty()) {
    throw std::invalid_argument("--kind and --input are required");
  }
  if (arguments.kind != "case" && arguments.kind != "report" &&
      arguments.kind != "baseline") {
    throw std::invalid_argument("--kind must be case, report, or baseline");
  }
  return arguments;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = ParseArguments(argc, argv);
    const auto document = replay::LoadJsonFile(arguments.input);
    replay::ValidationResult result;
    if (arguments.kind == "case") result = replay::ValidateCaseManifest(document);
    else if (arguments.kind == "report") result = replay::ValidateReplayReport(document);
    else result = replay::ValidateReplayBaseline(document);
    if (!result.Ok()) {
      std::cerr << result.Summary();
      return 1;
    }
    std::cout << "valid replay " << arguments.kind << ": "
              << arguments.input << '\n';
    return 0;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid replay validation request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "replay validation failed: " << error.what() << '\n';
    return 2;
  }
}
