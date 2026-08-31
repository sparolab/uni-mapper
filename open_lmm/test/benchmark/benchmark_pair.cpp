#include "support/benchmark/benchmark_pair.hpp"
#include "support/benchmark/benchmark_report.hpp"

#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

namespace fs = std::filesystem;
using open_lmm::test::benchmark::PairParityEvidence;

struct Options {
  fs::path sequential;
  fs::path parallel;
  fs::path sequential_owner;
  fs::path parallel_owner;
  fs::path output;
  std::vector<PairParityEvidence> parity;
};

Options Parse(int argc, char** argv) {
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (index + 1 >= argc) throw std::invalid_argument("missing option value");
    const std::string value = argv[++index];
    if (option == "--sequential-bundle") options.sequential = value;
    else if (option == "--parallel-bundle") options.parallel = value;
    else if (option == "--sequential-owner-bundle")
      options.sequential_owner = value;
    else if (option == "--parallel-owner-bundle")
      options.parallel_owner = value;
    else if (option == "--output") options.output = value;
    else if (option == "--parity") {
      const auto first = value.find(':');
      const auto second = value.find(':', first == std::string::npos
                                              ? first
                                              : first + 1);
      if (first == std::string::npos || second == std::string::npos) {
        throw std::invalid_argument(
            "parity must use ITERATION:pass|fail:LOG_PATH");
      }
      PairParityEvidence evidence;
      evidence.iteration = std::stoull(value.substr(0, first));
      const std::string result = value.substr(first + 1, second - first - 1);
      if (result != "pass" && result != "fail") {
        throw std::invalid_argument("parity result must be pass or fail");
      }
      evidence.passed = result == "pass";
      evidence.log_path = value.substr(second + 1);
      options.parity.push_back(std::move(evidence));
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (options.sequential.empty() || options.parallel.empty() ||
      options.sequential_owner.empty() || options.parallel_owner.empty() ||
      options.output.empty() || options.parity.empty()) {
    throw std::invalid_argument("all pair bundles, parity logs and output are required");
  }
  return options;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = Parse(argc, argv);
    const auto report = open_lmm::test::benchmark::BuildMapUpdatePairReport(
        options.sequential, options.parallel, options.sequential_owner,
        options.parallel_owner, options.parity);
    open_lmm::test::benchmark::WriteJsonExclusive(options.output, report);
    std::cout << options.output << '\n';
    const std::string comparison = report.at("comparison");
    return report.at("result") == "pass" &&
                   (comparison == "pass" || comparison == "uncalibrated")
               ? 0
               : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
