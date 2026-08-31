#include "support/soak/soak_metrics.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

namespace {

Json Read(const fs::path& path) {
  std::ifstream input(path);
  Json report;
  if (!input || !(input >> report))
    throw std::runtime_error("failed to read family report " + path.string());
  const auto validation = soak::ValidateSoakReport(report);
  if (!validation.Ok()) {
    throw std::runtime_error("invalid family report " + path.string() +
                             ":\n" + validation.Summary());
  }
  return report;
}

void RequireSame(const Json& first, const Json& report, const char* key) {
  if (first.at(key) != report.at(key))
    throw std::runtime_error(std::string("family report mismatch: ") + key);
}

}  // namespace

int main(int argc, char** argv) {
  try {
    if (argc < 4 || std::string(argv[1]) != "--output")
      throw std::invalid_argument(
          "usage: open_lmm_soak_aggregate --output NEW_FILE REPORT...");
    const fs::path output = argv[2];
    Json reports = Json::array();
    std::set<std::string> scenarios;
    for (int index = 3; index < argc; ++index) {
      Json report = Read(argv[index]);
      if (!reports.empty()) {
        for (const char* key : {"profile", "iterations", "warmup_iterations",
                                "seed", "git"}) {
          RequireSame(reports.front(), report, key);
        }
      }
      if (!scenarios.insert(report.at("scenario").get<std::string>()).second)
        throw std::runtime_error("duplicate family scenario");
      if (report.at("result") != "pass")
        throw std::runtime_error("family report is not passing");
      reports.push_back(std::move(report));
    }
    const Json& first = reports.front();
    Json bundle{{"schema_version", 1},
                {"run_id", "all-headless-" +
                               first.at("profile").get<std::string>()},
                {"profile", first.at("profile")},
                {"iterations", first.at("iterations")},
                {"warmup_iterations", first.at("warmup_iterations")},
                {"seed", first.at("seed")},
                {"git", first.at("git")},
                {"reports", std::move(reports)},
                {"result", "pass"}};
    soak::WriteJsonExclusive(output, bundle);
    std::cout << "soak bundle=PASS families=" << scenarios.size() << '\n';
    return 0;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid aggregate request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak aggregate failure: " << error.what() << '\n';
    return 1;
  }
}
