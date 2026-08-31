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

}  // namespace

int main() {
  TestRollbackAndAtomicReplacement();
  std::cout << "output repository contract tests passed\n";
  return 0;
}
