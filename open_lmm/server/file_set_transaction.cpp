#include "file_set_transaction.hpp"

#include <string>

namespace fs = std::filesystem;
namespace open_lmm {
namespace {
struct Entry {
  fs::path temporary;
  fs::path final;
  fs::path backup;
  bool had_original = false;
  bool installed = false;
};

void RollBack(std::vector<Entry>& entries) {
  for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
    std::error_code ignored;
    if (it->installed) fs::remove(it->final, ignored);
    if (it->had_original && fs::exists(it->backup, ignored)) {
      ignored.clear();
      fs::rename(it->backup, it->final, ignored);
    }
  }
}
}  // namespace

Result<void> CommitFileSet(const std::vector<FileReplacement>& replacements) {
  std::vector<Entry> entries;
  entries.reserve(replacements.size());
  for (const auto& [temporary, final] : replacements) {
    std::error_code error;
    if (!fs::is_regular_file(temporary, error)) {
      return Result<void>::Failure(Error::FileNotFound(temporary.string()));
    }
    Entry entry{temporary, final, final.string() + ".open_lmm_backup"};
    if (fs::exists(entry.backup, error)) {
      return Result<void>::Failure(Error::IoError(
          "stale map backup blocks commit: " + entry.backup.string()));
    }
    entry.had_original = fs::exists(final, error);
    entries.push_back(std::move(entry));
  }

  for (auto& entry : entries) {
    if (!entry.had_original) continue;
    std::error_code error;
    fs::rename(entry.final, entry.backup, error);
    if (error) {
      RollBack(entries);
      return Result<void>::Failure(Error::IoError(
          "failed to preserve existing map file: " + error.message()));
    }
  }
  for (auto& entry : entries) {
    std::error_code error;
    fs::rename(entry.temporary, entry.final, error);
    if (error) {
      RollBack(entries);
      return Result<void>::Failure(Error::IoError(
          "failed to commit map file set: " + error.message()));
    }
    entry.installed = true;
  }
  for (const auto& entry : entries) {
    if (!entry.had_original) continue;
    std::error_code ignored;
    fs::remove(entry.backup, ignored);
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
