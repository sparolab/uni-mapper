#pragma once

#include <filesystem>
#include <optional>
#include <utility>
#include <vector>

#include <open_lmm/common/result.hpp>

namespace open_lmm {

using FileReplacement =
    std::pair<std::filesystem::path, std::filesystem::path>;  // temp, final

struct FileSetCommitOutcome {
  // Present only when every destination was committed but post-commit backup
  // cleanup requires operator recovery.
  std::optional<Error> recovery_required;
};

// Replaces a set of files as one recoverable operation. Existing destination
// files are restored if any detected rename fails before the whole set is
// committed. This is failure-atomic while the process/filesystem remain
// operational; it is not a power-loss or sudden-termination durability
// guarantee. Post-commit cleanup faults return recovery_required and leave an
// owner-only manifest for manual reconciliation.
Result<FileSetCommitOutcome> CommitFileSet(
    const std::vector<FileReplacement>& replacements);

}  // namespace open_lmm
