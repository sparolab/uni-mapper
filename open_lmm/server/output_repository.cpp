#include "output_repository.hpp"

namespace fs = std::filesystem;
namespace open_lmm {

PendingOutputSet::~PendingOutputSet() { Rollback(); }

void PendingOutputSet::Add(fs::path temporary, fs::path destination) {
  files_.emplace_back(std::move(temporary), std::move(destination));
}

const std::vector<FileReplacement>& PendingOutputSet::Files() const {
  return files_;
}

Result<void> PendingOutputSet::Commit() {
  auto result = CommitFileSet(files_);
  if (result) committed_ = true;
  return result;
}

void PendingOutputSet::Rollback() {
  if (committed_) return;
  for (const auto& [temporary, destination] : files_) {
    (void)destination;
    std::error_code ignored;
    fs::remove(temporary, ignored);
  }
  files_.clear();
}

}  // namespace open_lmm
