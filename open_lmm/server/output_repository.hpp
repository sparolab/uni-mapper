#pragma once

#include <filesystem>
#include <string_view>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/file_set_transaction.hpp>

namespace open_lmm {

// Owns every temporary file produced by one runtime transaction. Destruction
// rolls back uncommitted temporaries; Commit installs the complete set through
// the recoverable file-set barrier.
class PendingOutputSet {
 public:
  PendingOutputSet() = default;
  ~PendingOutputSet();
  PendingOutputSet(const PendingOutputSet&) = delete;
  PendingOutputSet& operator=(const PendingOutputSet&) = delete;
  PendingOutputSet(PendingOutputSet&&) = delete;
  PendingOutputSet& operator=(PendingOutputSet&&) = delete;

  void Add(std::filesystem::path temporary,
           std::filesystem::path destination);
  [[nodiscard]] const std::vector<FileReplacement>& Files() const;
  Result<void> Commit();
  void Rollback();

 private:
  std::vector<FileReplacement> files_;
  bool committed_ = false;
};

class OutputRepository {
 public:
  [[nodiscard]] PendingOutputSet Begin() const { return {}; }
};

// Stages one canonical JSON document beside its destination using an
// owner-only, exclusive, no-follow temporary. The PendingOutputSet owns
// cleanup and the caller chooses the state/file commit boundary.
Result<void> StageConfigFile(const std::filesystem::path& destination,
                             std::string_view canonical_json,
                             PendingOutputSet& pending);

}  // namespace open_lmm
