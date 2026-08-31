#include <storage/transactions/file_set_transaction.hpp>

#include "property_generator.hpp"

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

void Write(const fs::path& path, const std::string& value) {
  std::ofstream stream(path);
  stream << value;
}

std::string Read(const fs::path& path) {
  std::ifstream stream(path);
  std::string value;
  stream >> value;
  return value;
}

}  // namespace

int main() {
  using open_lmm::test::property::Fail;
  using open_lmm::test::property::Generator;
  const uint64_t seed = open_lmm::test::property::Seed();
  const std::size_t cases = open_lmm::test::property::Cases(100);
  Generator generator(seed);
  const fs::path base = fs::temp_directory_path() /
                        ("open_lmm_file_property_" + std::to_string(seed));
  std::error_code ignored;
  fs::remove_all(base, ignored);

  for (std::size_t index = 0; index < cases; ++index) {
    const fs::path root = base / std::to_string(index);
    fs::create_directories(root);
    const std::size_t count = 1 + generator.Index(4);
    const bool should_fail = (generator.Next() & 1U) != 0;
    std::vector<open_lmm::FileReplacement> replacements;
    std::vector<fs::path> destinations;
    for (std::size_t item = 0; item < count; ++item) {
      const fs::path temporary =
          root / ("item_" + std::to_string(item) + ".tmp");
      fs::path destination =
          root / ("item_" + std::to_string(item) + ".final");
      if (should_fail && item + 1 == count) {
        destination = root / "missing" / "failure.final";
      } else {
        Write(destination, "old_" + std::to_string(item));
      }
      Write(temporary, "new_" + std::to_string(item));
      replacements.emplace_back(temporary, destination);
      destinations.push_back(destination);
    }

    const auto result = open_lmm::CommitFileSet(replacements);
    if (should_fail) {
      if (result) {
        Fail("file-set-failure", seed, index,
             "injected pre-commit failure reported success");
      }
      for (std::size_t item = 0; item + 1 < count; ++item) {
        if (!fs::is_regular_file(destinations[item]) ||
            Read(destinations[item]) != "old_" + std::to_string(item)) {
          Fail("file-set-atomic-rollback", seed, index,
               "partial candidate was externally visible");
        }
      }
      if (fs::exists(destinations.back())) {
        Fail("file-set-failed-destination", seed, index,
             "failed destination became visible");
      }
    } else {
      if (!result || result.Value().recovery_required) {
        Fail("file-set-success", seed, index,
             "valid transaction did not commit cleanly");
      }
      for (std::size_t item = 0; item < count; ++item) {
        if (Read(destinations[item]) != "new_" + std::to_string(item)) {
          Fail("file-set-complete-publication", seed, index,
               "successful transaction was incomplete");
        }
      }
    }
  }
  fs::remove_all(base, ignored);
  std::cout << "file-set transaction properties passed seed=" << seed
            << " cases=" << cases << '\n';
  return 0;
}
