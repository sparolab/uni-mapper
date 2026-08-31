#include "replay_input_lock.hpp"

#include "replay_sha256.hpp"

#include <fstream>
#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm::test::replay {
namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;

bool IsSafeRelativePath(const fs::path& path) {
  if (path.empty() || path.is_absolute() || path.has_root_name() ||
      path.has_root_directory() || path.lexically_normal() != path) {
    return false;
  }
  for (const auto& component : path) {
    if (component == "." || component == "..") return false;
  }
  return true;
}

std::string DigestText(const Json& value) {
  std::string digest = value.get<std::string>();
  if (digest.starts_with("sha256:")) digest.erase(0, 7);
  return digest;
}

bool IsWithin(const fs::path& root, const fs::path& path) {
  const fs::path relative = path.lexically_relative(root);
  if (relative.empty() || relative.is_absolute()) return false;
  for (const auto& component : relative) {
    if (component == "..") return false;
  }
  return true;
}

void VerifyFile(const fs::path& root, const Json& file_lock,
                std::string_view owner) {
  const fs::path relative = file_lock.at("path").get<std::string>();
  if (!IsSafeRelativePath(relative)) {
    throw std::runtime_error(std::string(owner) +
                             " contains an unsafe path: " +
                             relative.string());
  }
  const fs::path path = root / relative;
  std::error_code error;
  const fs::path canonical_root = fs::canonical(root, error);
  if (error) {
    throw std::runtime_error(std::string(owner) + " root is unreadable");
  }
  const fs::path canonical_path = fs::canonical(path, error);
  if (error || !IsWithin(canonical_root, canonical_path) ||
      fs::is_symlink(fs::symlink_status(path, error)) || error ||
      !fs::is_regular_file(canonical_path)) {
    throw std::runtime_error(std::string(owner) + " file is missing: " +
                             relative.string());
  }
  const std::string expected = DigestText(file_lock.at("sha256"));
  const std::string actual = Sha256File(canonical_path);
  if (actual != expected) {
    throw std::runtime_error(std::string(owner) +
                             " SHA-256 mismatch: " + relative.string());
  }
}

struct VerifiedScanIndex {
  std::vector<std::uint64_t> frames;
  std::set<fs::path> canonical_files;
  std::set<fs::path> directories;
};

VerifiedScanIndex VerifyScanIndex(const fs::path& data_root,
                                  const Json& lock) {
  VerifyFile(data_root, lock, "scan index");
  std::error_code root_error;
  const fs::path canonical_root = fs::canonical(data_root, root_error);
  if (root_error) throw std::runtime_error("data root is unreadable");
  const fs::path index_path =
      data_root / lock.at("path").get<std::string>();
  std::ifstream input(index_path);
  if (!input) throw std::runtime_error("failed to read scan index");
  std::set<std::string> paths;
  std::set<std::uint64_t> frame_ids;
  VerifiedScanIndex verified;
  for (std::string line; std::getline(input, line);) {
    if (line.empty()) continue;
    const std::size_t frame_separator = line.find(' ');
    const std::size_t digest_begin = line.find_first_not_of(' ', frame_separator);
    if (frame_separator == std::string::npos ||
        digest_begin == std::string::npos || line.size() < digest_begin + 67 ||
        line[digest_begin + 64] != ' ' ||
        line[digest_begin + 65] != ' ') {
      throw std::runtime_error(
          "scan index entries must use '<frame-id> <sha256>  <relative-path>'");
    }
    std::uint64_t frame = 0;
    try {
      std::size_t consumed = 0;
      frame = std::stoull(line.substr(0, frame_separator), &consumed);
      if (consumed != frame_separator) throw std::invalid_argument("suffix");
    } catch (const std::exception&) {
      throw std::runtime_error("scan index contains an invalid frame id");
    }
    const std::string digest = line.substr(digest_begin, 64);
    const fs::path relative = line.substr(digest_begin + 66);
    if (!IsSafeRelativePath(relative) ||
        !paths.insert(relative.string()).second ||
        !frame_ids.insert(frame).second) {
      throw std::runtime_error(
          "scan index contains an unsafe path or duplicate frame/path");
    }
    const fs::path scan = data_root / relative;
    std::error_code error;
    const fs::path canonical_scan = fs::canonical(scan, error);
    if (error || !IsWithin(canonical_root, canonical_scan) ||
        fs::is_symlink(fs::symlink_status(scan, error)) || error ||
        !fs::is_regular_file(canonical_scan) ||
        Sha256File(canonical_scan) != digest) {
      throw std::runtime_error("scan content does not match index: " +
                               relative.string());
    }
    verified.frames.push_back(frame);
    verified.canonical_files.insert(canonical_scan);
    verified.directories.insert(canonical_scan.parent_path());
  }
  if (input.bad() || verified.frames.empty()) {
    throw std::runtime_error("scan index is empty or unreadable");
  }
  for (const fs::path& directory : verified.directories) {
    std::error_code error;
    for (fs::directory_iterator entry(directory, error), end;
         !error && entry != end; entry.increment(error)) {
      const fs::path status_path = entry->path();
      const fs::file_status status = entry->symlink_status(error);
      if (error || fs::is_symlink(status) ||
          (fs::is_regular_file(status) &&
           !verified.canonical_files.contains(fs::canonical(status_path)))) {
        throw std::runtime_error(
            "scan directory contains an unindexed file or link: " +
            status_path.lexically_relative(data_root).string());
      }
    }
    if (error) {
      throw std::runtime_error("failed to enumerate locked scan directory");
    }
  }
  return verified;
}

std::size_t NonEmptyLineCount(const fs::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("failed to read pose file");
  std::size_t count = 0;
  for (std::string line; std::getline(input, line);) {
    if (!line.empty()) ++count;
  }
  if (input.bad()) throw std::runtime_error("failed while reading pose file");
  return count;
}

}  // namespace

void VerifyReplayInputs(const Json& manifest, const fs::path& data_root,
                        const fs::path& config_root) {
  if (!fs::is_directory(data_root)) {
    throw std::runtime_error("data root is not a directory");
  }
  if (!fs::is_directory(config_root)) {
    throw std::runtime_error("config root is not a directory");
  }
  for (const Json& file : manifest.at("config").at("files")) {
    VerifyFile(config_root, file, "config");
  }
  for (const Json& agent : manifest.at("dataset").at("agents")) {
    VerifyFile(data_root, agent.at("pose_file"), "pose");
    const VerifiedScanIndex scan_index =
        VerifyScanIndex(data_root, agent.at("scan_index"));
    const std::vector<std::uint64_t> frames =
        agent.at("frames").get<std::vector<std::uint64_t>>();
    const fs::path pose =
        data_root / agent.at("pose_file").at("path").get<std::string>();
    const std::size_t poses = NonEmptyLineCount(pose);
    if (scan_index.frames != frames || poses != frames.size()) {
      throw std::runtime_error(
          "manifest frame count must match locked scans and poses for " +
          agent.at("id").get<std::string>());
    }
  }
}

}  // namespace open_lmm::test::replay
