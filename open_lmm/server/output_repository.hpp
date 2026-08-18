#pragma once

#include <filesystem>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/file_set_transaction.hpp>

namespace open_lmm {

// Owns every temporary file produced by one session transaction. Destruction
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

}  // namespace open_lmm
