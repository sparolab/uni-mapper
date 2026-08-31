#include <storage/transactions/output_repository.hpp>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fs = std::filesystem;
namespace open_lmm {

PendingOutputSet::~PendingOutputSet() { Rollback(); }

void PendingOutputSet::Add(fs::path temporary, fs::path destination) {
  files_.emplace_back(std::move(temporary), std::move(destination));
}

const std::vector<FileReplacement>& PendingOutputSet::Files() const {
  return files_;
}

Result<FileSetCommitOutcome> PendingOutputSet::Commit() {
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

Result<void> StageConfigFile(const fs::path& destination,
                             std::string_view canonical_json,
                             PendingOutputSet& pending) {
  if (destination.empty() || canonical_json.empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "config staging requires destination and canonical JSON"));
  }
  static std::atomic<uint64_t> sequence{0};
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  fs::path temporary = destination;
  temporary += ".open_lmm_candidate_" + std::to_string(nonce) + "_" +
               std::to_string(sequence.fetch_add(1, std::memory_order_relaxed)) +
               ".tmp";
  const int raw_fd = ::open(temporary.c_str(),
                            O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                            S_IRUSR | S_IWUSR);
  if (raw_fd < 0) {
    return Result<void>::Failure(Error::IoError(
        "failed to create config transaction file " + temporary.string() +
        ": " + std::strerror(errno)));
  }
  int descriptor = raw_fd;
  const auto cleanup = [&] {
    if (descriptor >= 0) ::close(descriptor);
    std::error_code ignored;
    fs::remove(temporary, ignored);
  };
  std::string contents(canonical_json);
  contents.push_back('\n');
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const ssize_t count = ::write(descriptor, contents.data() + offset,
                                  contents.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      const int error_number = errno ? errno : EIO;
      cleanup();
      descriptor = -1;
      return Result<void>::Failure(Error::IoError(
          "failed to write config transaction file " + temporary.string() +
          ": " + std::strerror(error_number)));
    }
    offset += static_cast<std::size_t>(count);
  }
  int error_number = 0;
  if (::fsync(descriptor) != 0) error_number = errno;
  if (::close(descriptor) != 0 && error_number == 0) error_number = errno;
  descriptor = -1;
  if (error_number != 0) {
    cleanup();
    return Result<void>::Failure(Error::IoError(
        "failed to finalize config transaction file " + temporary.string() +
        ": " + std::strerror(error_number)));
  }
  pending.Add(std::move(temporary), destination);
  return Result<void>::Ok();
}

}  // namespace open_lmm
