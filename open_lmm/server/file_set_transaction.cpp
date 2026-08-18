#include "file_set_transaction.hpp"

#include <chrono>
#include <atomic>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <set>
#include <sstream>
#include <string>
#include <sys/stat.h>
#include <unistd.h>

#include <open_lmm/utils/logging.hpp>

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

std::string EscapeJson(const std::string& value) {
  std::string escaped;
  escaped.reserve(value.size());
  for (const char character : value) {
    switch (character) {
      case '\\': escaped += "\\\\"; break;
      case '"': escaped += "\\\""; break;
      case '\n': escaped += "\\n"; break;
      case '\r': escaped += "\\r"; break;
      case '\t': escaped += "\\t"; break;
      default: escaped += character; break;
    }
  }
  return escaped;
}

fs::path WriteRecoveryManifest(const std::vector<Entry>& entries,
                               const std::string& reason,
                               std::error_code& error) {
  fs::path directory = fs::current_path(error);
  if (!entries.empty() && !entries.front().final.parent_path().empty()) {
    directory = entries.front().final.parent_path();
    error.clear();
  }
  std::ostringstream output;
  output << "{\n  \"version\": 1,\n  \"recovery_required\": true,\n"
         << "  \"reason\": \"" << EscapeJson(reason) << "\",\n"
         << "  \"entries\": [\n";
  for (std::size_t index = 0; index < entries.size(); ++index) {
    const auto& entry = entries[index];
    output << "    {\"temporary\": \""
           << EscapeJson(entry.temporary.string()) << "\", \"final\": \""
           << EscapeJson(entry.final.string()) << "\", \"backup\": \""
           << EscapeJson(entry.backup.string()) << "\", \"had_original\": "
           << (entry.had_original ? "true" : "false")
           << ", \"installed\": " << (entry.installed ? "true" : "false")
           << "}" << (index + 1 == entries.size() ? "\n" : ",\n");
  }
  output << "  ]\n}\n";
  const std::string contents = output.str();

  static std::atomic<uint64_t> sequence{0};
  fs::path manifest;
  int descriptor = -1;
  for (int attempt = 0; attempt < 16; ++attempt) {
    const auto nonce =
        std::chrono::steady_clock::now().time_since_epoch().count();
    manifest = directory /
        (".open_lmm_recovery_" + std::to_string(nonce) + "_" +
         std::to_string(sequence.fetch_add(1, std::memory_order_relaxed)) +
         ".json");
    descriptor = ::open(manifest.c_str(),
                        O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC | O_NOFOLLOW,
                        S_IRUSR | S_IWUSR);
    if (descriptor >= 0) break;
    if (errno != EEXIST) {
      error = std::error_code(errno, std::generic_category());
      return manifest;
    }
  }
  if (descriptor < 0) {
    error = std::make_error_code(std::errc::file_exists);
    return manifest;
  }
  std::size_t offset = 0;
  while (offset < contents.size()) {
    const ssize_t count = ::write(descriptor, contents.data() + offset,
                                  contents.size() - offset);
    if (count < 0 && errno == EINTR) continue;
    if (count <= 0) {
      error = std::error_code(errno ? errno : EIO, std::generic_category());
      break;
    }
    offset += static_cast<std::size_t>(count);
  }
  if (!error && ::fsync(descriptor) != 0)
    error = std::error_code(errno, std::generic_category());
  if (::close(descriptor) != 0 && !error)
    error = std::error_code(errno, std::generic_category());
  if (error) {
    std::error_code ignored;
    fs::remove(manifest, ignored);
  }
  return manifest;
}

Result<fs::path> PathKey(const fs::path& path, const char* role) {
  if (path.empty()) {
    return Result<fs::path>::Failure(
        Error::InvalidArgument(std::string(role) + " path must be non-empty"));
  }
  std::error_code error;
  auto absolute = fs::absolute(path, error);
  if (error) {
    return Result<fs::path>::Failure(Error::IoError(
        "failed to resolve " + std::string(role) + " path " + path.string() +
        ": " + error.message()));
  }
  return Result<fs::path>::Ok(absolute.lexically_normal());
}

Error RecoveryRequired(const std::string& reason,
                       const std::vector<Entry>& entries) {
  std::error_code manifest_error;
  const fs::path manifest =
      WriteRecoveryManifest(entries, reason, manifest_error);
  std::string detail = reason + "; manual recovery required";
  if (manifest_error) {
    detail += "; recovery manifest could not be written: " +
              manifest_error.message();
  } else {
    detail += "; recovery manifest: " + manifest.string();
  }
  return Error::IoError(detail)
      .WithExecution("file_transaction", "recovery_required")
      .MarkFatalSession();
}

Result<void> RollBack(std::vector<Entry>& entries) {
  std::ostringstream failures;
  bool failed = false;
  for (auto it = entries.rbegin(); it != entries.rend(); ++it) {
    std::error_code error;
    if (it->installed) {
      const bool removed = fs::remove(it->final, error);
      if (error || !removed) {
        failed = true;
        failures << "remove " << it->final << ": "
                 << (error ? error.message() : "installed file is missing")
                 << "; ";
        // Never overwrite a still-installed candidate with the backup.
        continue;
      }
      it->installed = false;
    }
    error.clear();
    if (it->had_original) {
      const bool backup_exists = fs::exists(it->backup, error);
      if (error) {
        failed = true;
        failures << "inspect " << it->backup << ": " << error.message() << "; ";
        continue;
      }
      if (!backup_exists) {
        failed = true;
        failures << "restore " << it->backup
                 << ": original backup is missing; ";
        continue;
      }
      fs::rename(it->backup, it->final, error);
      if (error) {
        failed = true;
        failures << "restore " << it->backup << ": " << error.message() << "; ";
      } else {
        it->had_original = false;
      }
    }
  }
  if (failed) {
    return Result<void>::Failure(
        RecoveryRequired("file-set rollback failed: " + failures.str(), entries));
  }
  return Result<void>::Ok();
}
}  // namespace

Result<void> CommitFileSet(const std::vector<FileReplacement>& replacements) {
  std::vector<Entry> entries;
  entries.reserve(replacements.size());
  std::set<fs::path> temporaries;
  std::set<fs::path> destinations;
  std::set<fs::path> backups;
  for (const auto& [temporary, final] : replacements) {
    auto temporary_key = PathKey(temporary, "temporary");
    auto final_key = PathKey(final, "destination");
    if (!temporary_key)
      return Result<void>::Failure(temporary_key.GetError());
    if (!final_key) return Result<void>::Failure(final_key.GetError());
    const fs::path backup = final_key.Value().string() + ".open_lmm_backup";
    if (!temporaries.insert(temporary_key.Value()).second) {
      return Result<void>::Failure(
          Error::InvalidArgument("duplicate temporary file in transaction: " +
                                 temporary_key.Value().string()));
    }
    if (!destinations.insert(final_key.Value()).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "duplicate destination file in transaction: " +
          final_key.Value().string()));
    }
    if (!backups.insert(backup).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "duplicate backup file in transaction: " + backup.string()));
    }
  }
  for (const auto& path : temporaries) {
    if (destinations.contains(path) || backups.contains(path)) {
      return Result<void>::Failure(Error::InvalidArgument(
          "temporary file aliases a transaction destination or backup: " +
          path.string()));
    }
  }
  for (const auto& path : destinations) {
    if (backups.contains(path)) {
      return Result<void>::Failure(Error::InvalidArgument(
          "destination file aliases a transaction backup: " + path.string()));
    }
  }

  for (const auto& [temporary, final] : replacements) {
    std::error_code error;
    const bool temporary_is_regular = fs::is_regular_file(temporary, error);
    if (error) {
      return Result<void>::Failure(Error::IoError(
          "failed to inspect temporary file " + temporary.string() + ": " +
          error.message()));
    }
    if (!temporary_is_regular) {
      return Result<void>::Failure(Error::FileNotFound(temporary.string()));
    }
    Entry entry{temporary, final, final.string() + ".open_lmm_backup"};
    const bool backup_exists = fs::exists(entry.backup, error);
    if (error) {
      return Result<void>::Failure(Error::IoError(
          "failed to inspect file backup " + entry.backup.string() + ": " +
          error.message()));
    }
    if (backup_exists) {
      return Result<void>::Failure(Error::IoError(
          "stale file backup blocks commit: " + entry.backup.string()));
    }
    entry.had_original = fs::exists(final, error);
    if (error) {
      return Result<void>::Failure(Error::IoError(
          "failed to inspect destination " + final.string() + ": " +
          error.message()));
    }
    entries.push_back(std::move(entry));
  }

  for (auto& entry : entries) {
    if (!entry.had_original) continue;
    std::error_code error;
    fs::rename(entry.final, entry.backup, error);
    if (error) {
      auto rollback = RollBack(entries);
      if (!rollback) return rollback;
      return Result<void>::Failure(Error::IoError(
          "failed to preserve existing file: " + error.message()));
    }
  }
  for (auto& entry : entries) {
    std::error_code error;
    fs::rename(entry.temporary, entry.final, error);
    if (error) {
      auto rollback = RollBack(entries);
      if (!rollback) return rollback;
      return Result<void>::Failure(Error::IoError(
          "failed to commit file set: " + error.message()));
    }
    entry.installed = true;
  }
  std::ostringstream cleanup_failures;
  bool cleanup_failed = false;
  for (auto& entry : entries) {
    if (!entry.had_original) continue;
    std::error_code error;
    if (!fs::remove(entry.backup, error) || error) {
      cleanup_failed = true;
      cleanup_failures << "failed to clean committed file backup "
                       << entry.backup << ": "
                       << (error ? error.message() : "backup was not removed")
                       << "; ";
      continue;
    }
    entry.had_original = false;
  }
  if (cleanup_failed) {
    // Every final is already installed at this point. Treat backup deletion as
    // post-commit recovery housekeeping: reporting failure here would leave
    // callers believing the old state is authoritative while disk contains
    // the new file set. RecoveryRequired writes a manifest for the retained
    // backups; a later transaction remains blocked until they are reconciled.
    const Error recovery = RecoveryRequired(cleanup_failures.str(), entries);
    LogError("[file_transaction/recovery_required] " + recovery.Message());
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
