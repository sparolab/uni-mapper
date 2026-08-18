#pragma once

#include <open_lmm/common/runtime_contracts.hpp>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace open_lmm {

struct ClientSessionSnapshot {
  SessionId id;
  std::string label;
  RuntimeSessionState state = RuntimeSessionState::kCreating;
  std::filesystem::path output_directory;
};

class RuntimeClient {
 public:
  explicit RuntimeClient(std::size_t maximum_sessions = 8);
  ~RuntimeClient();
  RuntimeClient(RuntimeClient&&) noexcept;
  RuntimeClient& operator=(RuntimeClient&&) noexcept;
  RuntimeClient(const RuntimeClient&) = delete;
  RuntimeClient& operator=(const RuntimeClient&) = delete;

  Result<SessionId> CreateSession(const BootstrapRequest& request);
  Result<JobId> SubmitRunAll(const SessionId& session_id);
  Result<void> Cancel(const SessionId& session_id, JobId job_id);
  Result<ClientSessionSnapshot> Snapshot(const SessionId& session_id) const;
  Result<void> CloseSession(const SessionId& session_id, CloseMode mode);
  [[nodiscard]] std::vector<SessionId> SessionIds() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace open_lmm
