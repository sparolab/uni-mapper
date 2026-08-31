#pragma once

#include <foundation/concurrency/bounded_executor.hpp>

#include <atomic>
#include <array>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>

namespace open_lmm {

enum class MemoryClass : uint8_t {
  kResidentPayload,
  kTransientTask,
  kHeavyMap,
};

class MemoryReservation {
 public:
  MemoryReservation() = default;
  ~MemoryReservation();
  MemoryReservation(MemoryReservation&& other) noexcept;
  MemoryReservation& operator=(MemoryReservation&& other) noexcept;
  MemoryReservation(const MemoryReservation&) = delete;
  MemoryReservation& operator=(const MemoryReservation&) = delete;

  [[nodiscard]] uint64_t Bytes() const noexcept { return bytes_; }
  [[nodiscard]] MemoryClass Class() const noexcept { return memory_class_; }
  Result<void> Resize(uint64_t bytes);
  void Reset() noexcept;

 private:
  MemoryReservation(std::shared_ptr<std::atomic<uint64_t>> total_counter,
                    std::shared_ptr<std::atomic<uint64_t>> class_counter,
                    std::shared_ptr<std::atomic<uint64_t>> total_peak,
                    std::shared_ptr<std::atomic<uint64_t>> class_peak,
                    std::shared_ptr<std::atomic<uint64_t>> failure_counter,
                    uint64_t limit, uint64_t bytes,
                    MemoryClass memory_class) noexcept;
  std::shared_ptr<std::atomic<uint64_t>> total_counter_;
  std::shared_ptr<std::atomic<uint64_t>> class_counter_;
  std::shared_ptr<std::atomic<uint64_t>> total_peak_;
  std::shared_ptr<std::atomic<uint64_t>> class_peak_;
  std::shared_ptr<std::atomic<uint64_t>> failure_counter_;
  uint64_t limit_ = 0;
  uint64_t bytes_ = 0;
  MemoryClass memory_class_ = MemoryClass::kTransientTask;
  friend class ResourceGovernor;
};

struct ResourceBudget {
  std::size_t max_agent_tasks = 2;
  std::size_t max_cpu_threads = 2;
  uint64_t soft_memory_bytes = 4ULL * 1024ULL * 1024ULL * 1024ULL;
};

struct ResourceGovernorDiagnostics {
  ResourceBudget budget;
  uint64_t reserved_total_bytes = 0;
  std::array<uint64_t, 3> reserved_by_class = {0, 0, 0};
  uint64_t peak_reserved_total_bytes = 0;
  std::array<uint64_t, 3> peak_reserved_by_class = {0, 0, 0};
  uint64_t admission_failures = 0;
  bool heavy_phase_active = false;
  BoundedExecutorSnapshot executor;
};

class ResourceGovernor {
 public:
  explicit ResourceGovernor(std::size_t max_agent_tasks);
  explicit ResourceGovernor(ResourceBudget budget);

  [[nodiscard]] bool TryReserveMemory(uint64_t bytes) noexcept;
  void ReleaseMemory(uint64_t bytes) noexcept;
  Result<MemoryReservation> ReserveMemory(
      uint64_t bytes, MemoryClass memory_class,
      const std::shared_ptr<CancellationToken>& cancellation = {});
  Result<MemoryReservation> ReserveReplacementMemory(
      uint64_t bytes, uint64_t replaced_resident_bytes,
      uint64_t provisional_overlap_bytes,
      const std::shared_ptr<CancellationToken>& cancellation = {});
  Result<void> ValidateReplacementMemory(
      uint64_t replaced_resident_bytes) const;
  Result<void> AcquireHeavyMemoryPhase(
      const std::shared_ptr<CancellationToken>& cancellation);
  void ReleaseHeavyMemoryPhase() noexcept;

  [[nodiscard]] uint64_t ReservedMemoryBytes() const noexcept {
    return reserved_memory_bytes_->load(std::memory_order_acquire);
  }
  [[nodiscard]] uint64_t ReservedMemoryBytes(
      MemoryClass memory_class) const noexcept;
  [[nodiscard]] uint64_t MemoryAdmissionFailures() const noexcept {
    return memory_admission_failures_->load(std::memory_order_acquire);
  }
  [[nodiscard]] const ResourceBudget& Budget() const noexcept {
    return budget_;
  }
  [[nodiscard]] BoundedExecutor& AgentExecutor() noexcept {
    return agent_executor_;
  }
  [[nodiscard]] const BoundedExecutor& AgentExecutor() const noexcept {
    return agent_executor_;
  }
  [[nodiscard]] ResourceGovernorDiagnostics Diagnostics() const;
  void ResetDiagnosticPeaksToCurrent();

 private:
  ResourceBudget budget_;
  std::shared_ptr<std::atomic<uint64_t>> reserved_memory_bytes_ =
      std::make_shared<std::atomic<uint64_t>>(0);
  std::array<std::shared_ptr<std::atomic<uint64_t>>, 3>
      reserved_memory_by_class_ = {
          std::make_shared<std::atomic<uint64_t>>(0),
          std::make_shared<std::atomic<uint64_t>>(0),
          std::make_shared<std::atomic<uint64_t>>(0)};
  std::shared_ptr<std::atomic<uint64_t>> peak_reserved_memory_bytes_ =
      std::make_shared<std::atomic<uint64_t>>(0);
  std::array<std::shared_ptr<std::atomic<uint64_t>>, 3>
      peak_reserved_memory_by_class_ = {
          std::make_shared<std::atomic<uint64_t>>(0),
          std::make_shared<std::atomic<uint64_t>>(0),
          std::make_shared<std::atomic<uint64_t>>(0)};
  std::shared_ptr<std::atomic<uint64_t>> memory_admission_failures_ =
      std::make_shared<std::atomic<uint64_t>>(0);
  BoundedExecutor agent_executor_;
  mutable std::mutex heavy_phase_mutex_;
  std::condition_variable heavy_phase_ready_;
  bool heavy_phase_active_ = false;
};

}  // namespace open_lmm
