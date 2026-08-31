#include "resource_governor.hpp"

#include <algorithm>
#include <chrono>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace open_lmm {

namespace {

bool TryAdd(const std::shared_ptr<std::atomic<uint64_t>>& counter,
            uint64_t limit, uint64_t bytes) noexcept {
  uint64_t current = counter->load(std::memory_order_acquire);
  while (current <= limit && bytes <= limit - current) {
    if (counter->compare_exchange_weak(
            current, current + bytes, std::memory_order_acq_rel)) {
      return true;
    }
  }
  return false;
}

void Subtract(const std::shared_ptr<std::atomic<uint64_t>>& counter,
              uint64_t bytes) noexcept {
  uint64_t current = counter->load(std::memory_order_acquire);
  while (current != 0) {
    const uint64_t released = std::min(current, bytes);
    if (counter->compare_exchange_weak(
            current, current - released, std::memory_order_acq_rel)) return;
  }
}

ResourceBudget TaskOnlyBudget(std::size_t max_agent_tasks) {
  ResourceBudget budget;
  budget.max_agent_tasks = max_agent_tasks;
  budget.max_cpu_threads = max_agent_tasks;
  return budget;
}

std::size_t EffectiveWorkers(const ResourceBudget& budget) {
  return std::min(budget.max_agent_tasks, budget.max_cpu_threads);
}

std::size_t MemoryClassIndex(MemoryClass memory_class) noexcept {
  return static_cast<std::size_t>(memory_class);
}

const char* MemoryClassName(MemoryClass memory_class) noexcept {
  switch (memory_class) {
    case MemoryClass::kResidentPayload:
      return "resident_payload";
    case MemoryClass::kTransientTask:
      return "transient_task";
    case MemoryClass::kHeavyMap:
      return "heavy_map";
  }
  return "unknown";
}

uint64_t SaturatingAdd(uint64_t lhs, uint64_t rhs) noexcept {
  return rhs > std::numeric_limits<uint64_t>::max() - lhs
             ? std::numeric_limits<uint64_t>::max()
             : lhs + rhs;
}

Error MemoryAdmissionRejected(MemoryClass memory_class,
                              uint64_t current_bytes,
                              uint64_t requested_delta_bytes,
                              uint64_t limit_bytes,
                              std::string reason = {}) {
  std::string message =
      "memory admission rejected: class=" +
      std::string(MemoryClassName(memory_class)) +
      " current_bytes=" + std::to_string(current_bytes) +
      " requested_delta_bytes=" + std::to_string(requested_delta_bytes) +
      " requested_total_bytes=" +
      std::to_string(SaturatingAdd(current_bytes, requested_delta_bytes)) +
      " limit_bytes=" + std::to_string(limit_bytes);
  if (!reason.empty()) message += " reason=" + std::move(reason);
  return Error::InvalidArgument(std::move(message));
}

}  // namespace

MemoryReservation::MemoryReservation(
    std::shared_ptr<std::atomic<uint64_t>> total_counter,
    std::shared_ptr<std::atomic<uint64_t>> class_counter,
    std::shared_ptr<std::atomic<uint64_t>> failure_counter, uint64_t limit,
    uint64_t bytes, MemoryClass memory_class) noexcept
    : total_counter_(std::move(total_counter)),
      class_counter_(std::move(class_counter)),
      failure_counter_(std::move(failure_counter)), limit_(limit), bytes_(bytes),
      memory_class_(memory_class) {}

MemoryReservation::~MemoryReservation() { Reset(); }

MemoryReservation::MemoryReservation(MemoryReservation&& other) noexcept
    : total_counter_(std::move(other.total_counter_)),
      class_counter_(std::move(other.class_counter_)),
      failure_counter_(std::move(other.failure_counter_)),
      limit_(other.limit_),
      bytes_(std::exchange(other.bytes_, 0)),
      memory_class_(other.memory_class_) {}

MemoryReservation& MemoryReservation::operator=(
    MemoryReservation&& other) noexcept {
  if (this == &other) return *this;
  Reset();
  total_counter_ = std::move(other.total_counter_);
  class_counter_ = std::move(other.class_counter_);
  failure_counter_ = std::move(other.failure_counter_);
  limit_ = other.limit_;
  bytes_ = std::exchange(other.bytes_, 0);
  memory_class_ = other.memory_class_;
  return *this;
}

Result<void> MemoryReservation::Resize(uint64_t bytes) {
  if (!total_counter_ || !class_counter_) {
    return Result<void>::Failure(
        Error::InvalidArgument("memory reservation is not active"));
  }
  if (bytes > bytes_) {
    const uint64_t delta = bytes - bytes_;
    if (!TryAdd(total_counter_, limit_, delta)) {
      if (failure_counter_) {
        failure_counter_->fetch_add(1, std::memory_order_relaxed);
      }
      return Result<void>::Failure(MemoryAdmissionRejected(
          memory_class_, total_counter_->load(std::memory_order_acquire),
          delta, limit_));
    }
    class_counter_->fetch_add(delta, std::memory_order_acq_rel);
  } else if (bytes < bytes_) {
    const uint64_t delta = bytes_ - bytes;
    Subtract(class_counter_, delta);
    Subtract(total_counter_, delta);
  }
  bytes_ = bytes;
  return Result<void>::Ok();
}

void MemoryReservation::Reset() noexcept {
  if (total_counter_ && class_counter_ && bytes_ != 0) {
    Subtract(class_counter_, bytes_);
    Subtract(total_counter_, bytes_);
  }
  bytes_ = 0;
  total_counter_.reset();
  class_counter_.reset();
  failure_counter_.reset();
}

ResourceGovernor::ResourceGovernor(std::size_t max_agent_tasks)
    : ResourceGovernor(TaskOnlyBudget(max_agent_tasks)) {}

ResourceGovernor::ResourceGovernor(ResourceBudget budget)
    : budget_(budget),
      agent_executor_(EffectiveWorkers(budget),
                      EffectiveWorkers(budget)) {
  if (budget.max_agent_tasks == 0 || budget.max_cpu_threads == 0 ||
      budget.soft_memory_bytes == 0) {
    throw std::invalid_argument("resource budget values must be positive");
  }
}

bool ResourceGovernor::TryReserveMemory(uint64_t bytes) noexcept {
  if (TryAdd(reserved_memory_bytes_, budget_.soft_memory_bytes, bytes)) {
    return true;
  }
  memory_admission_failures_->fetch_add(1, std::memory_order_relaxed);
  return false;
}

void ResourceGovernor::ReleaseMemory(uint64_t bytes) noexcept {
  Subtract(reserved_memory_bytes_, bytes);
}

Result<MemoryReservation> ResourceGovernor::ReserveMemory(
    uint64_t bytes, MemoryClass memory_class,
    const std::shared_ptr<CancellationToken>& cancellation) {
  if (cancellation && cancellation->IsCancellationRequested()) {
    return Result<MemoryReservation>::Failure(
        Error::Cancelled("before memory reservation"));
  }
  if (!TryReserveMemory(bytes)) {
    return Result<MemoryReservation>::Failure(MemoryAdmissionRejected(
        memory_class,
        reserved_memory_bytes_->load(std::memory_order_acquire), bytes,
        budget_.soft_memory_bytes));
  }
  const std::size_t class_index = MemoryClassIndex(memory_class);
  if (class_index >= reserved_memory_by_class_.size()) {
    ReleaseMemory(bytes);
    return Result<MemoryReservation>::Failure(
        Error::InvalidArgument("unknown memory reservation class"));
  }
  reserved_memory_by_class_[class_index]->fetch_add(
      bytes, std::memory_order_acq_rel);
  return Result<MemoryReservation>::Ok(MemoryReservation(
      reserved_memory_bytes_, reserved_memory_by_class_[class_index],
      memory_admission_failures_, budget_.soft_memory_bytes, bytes,
      memory_class));
}

Result<MemoryReservation> ResourceGovernor::ReserveReplacementMemory(
    uint64_t bytes, uint64_t replaced_resident_bytes,
    uint64_t provisional_overlap_bytes,
    const std::shared_ptr<CancellationToken>& cancellation) {
  if (cancellation && cancellation->IsCancellationRequested()) {
    return Result<MemoryReservation>::Failure(
        Error::Cancelled("before replacement memory reservation"));
  }
  const uint64_t resident =
      ReservedMemoryBytes(MemoryClass::kResidentPayload);
  if (replaced_resident_bytes > resident) {
    return Result<MemoryReservation>::Failure(Error::InvalidArgument(
        "replacement memory credit exceeds resident ownership"));
  }
  uint64_t limit = budget_.soft_memory_bytes;
  for (const uint64_t allowance : {replaced_resident_bytes,
                                   provisional_overlap_bytes}) {
    limit = allowance > std::numeric_limits<uint64_t>::max() - limit
                ? std::numeric_limits<uint64_t>::max()
                : limit + allowance;
  }
  uint64_t current = reserved_memory_bytes_->load(std::memory_order_acquire);
  for (;;) {
    if (current > limit || bytes > limit - current) {
      memory_admission_failures_->fetch_add(1, std::memory_order_relaxed);
      return Result<MemoryReservation>::Failure(MemoryAdmissionRejected(
          MemoryClass::kResidentPayload, current, bytes, limit,
          "replacement_overlap"));
    }
    if (reserved_memory_bytes_->compare_exchange_weak(
            current, current + bytes, std::memory_order_acq_rel,
            std::memory_order_acquire)) {
      break;
    }
  }
  reserved_memory_by_class_[static_cast<std::size_t>(
      MemoryClass::kResidentPayload)]->fetch_add(bytes,
                                                 std::memory_order_acq_rel);
  return Result<MemoryReservation>::Ok(MemoryReservation(
      reserved_memory_bytes_,
      reserved_memory_by_class_[static_cast<std::size_t>(
          MemoryClass::kResidentPayload)],
      memory_admission_failures_, limit, bytes,
      MemoryClass::kResidentPayload));
}

Result<void> ResourceGovernor::ValidateReplacementMemory(
    uint64_t replaced_resident_bytes) const {
  const uint64_t resident =
      ReservedMemoryBytes(MemoryClass::kResidentPayload);
  if (replaced_resident_bytes > resident) {
    return Result<void>::Failure(Error::InvalidArgument(
        "replacement memory credit exceeds resident ownership"));
  }
  const uint64_t committed_limit =
      replaced_resident_bytes >
              std::numeric_limits<uint64_t>::max() -
                  budget_.soft_memory_bytes
          ? std::numeric_limits<uint64_t>::max()
          : budget_.soft_memory_bytes + replaced_resident_bytes;
  if (ReservedMemoryBytes() > committed_limit) {
    const uint64_t current = ReservedMemoryBytes();
    return Result<void>::Failure(MemoryAdmissionRejected(
        MemoryClass::kResidentPayload, current, 0, committed_limit,
        "replacement_result"));
  }
  return Result<void>::Ok();
}

uint64_t ResourceGovernor::ReservedMemoryBytes(
    MemoryClass memory_class) const noexcept {
  const std::size_t class_index = MemoryClassIndex(memory_class);
  if (class_index >= reserved_memory_by_class_.size()) return 0;
  return reserved_memory_by_class_[class_index]->load(std::memory_order_acquire);
}

Result<void> ResourceGovernor::AcquireHeavyMemoryPhase(
    const std::shared_ptr<CancellationToken>& cancellation) {
  std::unique_lock lock(heavy_phase_mutex_);
  while (heavy_phase_active_) {
    if (cancellation && cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(
          Error::Cancelled("while waiting for heavy memory phase"));
    }
    heavy_phase_ready_.wait_for(lock, std::chrono::milliseconds(10));
  }
  if (cancellation && cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(
        Error::Cancelled("before heavy memory phase"));
  }
  heavy_phase_active_ = true;
  return Result<void>::Ok();
}

void ResourceGovernor::ReleaseHeavyMemoryPhase() noexcept {
  {
    std::lock_guard lock(heavy_phase_mutex_);
    heavy_phase_active_ = false;
  }
  heavy_phase_ready_.notify_one();
}

}  // namespace open_lmm
