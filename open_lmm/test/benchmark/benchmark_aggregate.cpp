#include "support/benchmark/benchmark_bundle.hpp"
#include "support/benchmark/benchmark_report.hpp"

#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

int Run(int argc, char** argv) {
  std::filesystem::path output;
  std::optional<std::filesystem::path> baseline;
  std::vector<std::filesystem::path> reports;
  std::set<std::string> singleton_options;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (option != "--report" && !singleton_options.insert(option).second) {
      throw std::invalid_argument("duplicate option: " + option);
    }
    if (++index >= argc) {
      throw std::invalid_argument("missing value for " + option);
    }
    const std::filesystem::path value = argv[index];
    if (option == "--output") output = value;
    else if (option == "--baseline") baseline = value;
    else if (option == "--report") reports.push_back(value);
    else throw std::invalid_argument("unknown option: " + option);
  }
  if (output.empty() || reports.empty()) {
    throw std::invalid_argument(
        "usage: open_lmm_benchmark_aggregate --output <new-file> "
        "[--baseline <reviewed-file>] --report <raw-file>...");
  }
  std::set<std::filesystem::path> unique_reports(reports.begin(),
                                                  reports.end());
  if (unique_reports.size() != reports.size()) {
    throw std::invalid_argument("duplicate raw report path");
  }
  auto bundle = open_lmm::test::benchmark::AggregatePerformanceReports(
      reports, baseline);
  open_lmm::test::benchmark::WriteJsonExclusive(output, bundle);
  std::cout << output << '\n';
  return bundle.at("comparison") == "pass" ||
                 bundle.at("comparison") == "uncalibrated"
             ? 0
             : 1;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
