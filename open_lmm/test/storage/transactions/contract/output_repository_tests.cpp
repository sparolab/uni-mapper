#include <storage/transactions/output_repository.hpp>

#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

void WriteText(const fs::path& path, const std::string& text) {
  std::ofstream(path) << text;
}

void TestRollbackAndAtomicReplacement() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto directory = fs::temp_directory_path() /
                         ("open_lmm_output_transaction_" +
                          std::to_string(nonce));
  fs::create_directories(directory);
  const auto temporary = directory / "value.tmp";
  const auto destination = directory / "value.txt";

  WriteText(temporary, "discard");
  {
    PendingOutputSet pending;
    pending.Add(temporary, destination);
  }
  Check(!fs::exists(temporary) && !fs::exists(destination),
        "uncommitted file set removes temporary output");

  WriteText(destination, "old");
  WriteText(temporary, "new");
  {
    PendingOutputSet pending;
    pending.Add(temporary, destination);
    Check(pending.Commit().IsOk(), "complete file set commits");
  }
  Check(ReadText(destination) == "new" &&
            !fs::exists(destination.string() + ".open_lmm_backup"),
        "commit replaces destination and removes recovery backup");
  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

void TestConfigStagingUsesPrettyJson() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto directory = fs::temp_directory_path() /
                         ("open_lmm_config_format_" + std::to_string(nonce));
  fs::create_directories(directory);
  const auto destination = directory / "config.json";

  PendingOutputSet pending;
  auto staged = StageConfigFile(
      destination, R"({"outer":{"enabled":true,"value":1}})", pending);
  Check(staged.IsOk(), "valid canonical JSON stages");
  Check(pending.Commit().IsOk(), "pretty config file commits");
  Check(ReadText(destination) ==
            "{\n"
            "  \"outer\": {\n"
            "    \"enabled\": true,\n"
            "    \"value\": 1\n"
            "  }\n"
            "}\n",
        "persisted config uses stable two-space formatting");

  PendingOutputSet invalid;
  Check(!StageConfigFile(directory / "invalid.json", "{", invalid) &&
            !fs::exists(directory / "invalid.json"),
        "invalid canonical JSON fails before creating output");
  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

void TestConfigStagingPreservesExistingObjectOrder() {
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const auto directory = fs::temp_directory_path() /
                         ("open_lmm_config_order_" + std::to_string(nonce));
  fs::create_directories(directory);
  const auto destination = directory / "config.json";
  WriteText(destination,
            "{\n"
            "  // Human-owned grouping is intentionally not alphabetical.\n"
            "  \"zeta\": 0,\n"
            "  \"nested\": {\"second\": 0, \"first\": 0},\n"
            "  \"deprecated\": true\n"
            "}\n");

  PendingOutputSet pending;
  auto staged = StageConfigFile(
      destination,
      R"({"added":3,"nested":{"first":1,"new":2,"second":2},"zeta":9})",
      pending);
  Check(staged.IsOk() && pending.Commit().IsOk(),
        "ordered config replacement commits");
  Check(ReadText(destination) ==
            "{\n"
            "  \"zeta\": 9,\n"
            "  \"nested\": {\n"
            "    \"second\": 2,\n"
            "    \"first\": 1,\n"
            "    \"new\": 2\n"
            "  },\n"
            "  \"added\": 3\n"
            "}\n",
        "existing object order survives while values and key set stay canonical");
  std::error_code ignored;
  fs::remove_all(directory, ignored);
}

}  // namespace

int main() {
  TestRollbackAndAtomicReplacement();
  TestConfigStagingUsesPrettyJson();
  TestConfigStagingPreservesExistingObjectOrder();
  std::cout << "output repository contract tests passed\n";
  return 0;
}
