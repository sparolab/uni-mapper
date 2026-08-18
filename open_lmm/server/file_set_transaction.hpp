#pragma once

#include <filesystem>
#include <utility>
#include <vector>

#include <open_lmm/common/result.hpp>

namespace open_lmm {

using FileReplacement =
    std::pair<std::filesystem::path, std::filesystem::path>;  // temp, final

// Replaces a set of files as one recoverable operation. Existing destination
// files are restored if any rename fails before the whole set is committed.
Result<void> CommitFileSet(const std::vector<FileReplacement>& replacements);

}  // namespace open_lmm
