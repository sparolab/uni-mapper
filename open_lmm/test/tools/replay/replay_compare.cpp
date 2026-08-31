#include "replay_contract.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

namespace fs = std::filesystem;
using open_lmm::test::replay::CompareReplayReport;
using open_lmm::test::replay::LoadJsonFile;

namespace {

struct Arguments {
  fs::path baseline;
  fs::path report;
  fs::path diff;
};

Arguments ParseArguments(int argc, char** argv) {
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    auto consume = [&](fs::path& destination) {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      destination = argv[index];
    };
    if (option == "--baseline") consume(arguments.baseline);
    else if (option == "--report") consume(arguments.report);
    else if (option == "--diff") consume(arguments.diff);
    else if (option == "--help") {
      std::cout << "usage: open_lmm_replay_compare --baseline BASELINE.json "
                   "--report REPORT.json [--diff DIFF.json]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.baseline.empty() || arguments.report.empty()) {
    throw std::invalid_argument("--baseline and --report are required");
  }
  return arguments;
}

void WriteJson(const fs::path& path, const nlohmann::json& value) {
  std::ofstream output(path);
  if (!output) throw std::runtime_error("failed to open diff output: " + path.string());
  output << value.dump(2) << '\n';
  if (!output) throw std::runtime_error("failed to write diff output: " + path.string());
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = ParseArguments(argc, argv);
    const auto result = CompareReplayReport(LoadJsonFile(arguments.baseline),
                                            LoadJsonFile(arguments.report));
    const nlohmann::json output = result.ToJson();
    std::cout << output.dump(2) << '\n';
    if (!arguments.diff.empty()) WriteJson(arguments.diff, output);
    return result.Passed() ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid replay comparison: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "replay comparison failed: " << error.what() << '\n';
    return 2;
  }
}
