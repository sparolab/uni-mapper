#include "file_set_transaction.hpp"

#include <chrono>
#include <fstream>
#include <sstream>
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
  const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
  const fs::path manifest = directory /
      (".open_lmm_recovery_" + std::to_string(nonce) + ".json");
  std::ofstream output(manifest, std::ios::out | std::ios::trunc);
  if (!output) {
    error = std::make_error_code(std::errc::io_error);
    return manifest;
  }
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
  output.flush();
  if (!output) error = std::make_error_code(std::errc::io_error);
  return manifest;
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
    (void)RecoveryRequired(cleanup_failures.str(), entries);
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
