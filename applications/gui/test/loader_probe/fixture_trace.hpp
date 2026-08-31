#pragma once

#include <atomic>
#include <cstdint>

struct GuiLoaderProbeTrace {
  std::atomic<std::uint32_t> sequence{0};
  std::atomic<std::uint32_t> create_order{0};
  std::atomic<std::uint32_t> start_order{0};
  std::atomic<std::uint32_t> request_stop_order{0};
  std::atomic<std::uint32_t> join_order{0};
  std::atomic<std::uint32_t> destroy_order{0};
  std::atomic<std::uint32_t> unload_order{0};
};

inline void RecordProbeOrder(std::atomic<std::uint32_t>& destination,
                             GuiLoaderProbeTrace& trace) {
  destination.store(trace.sequence.fetch_add(1, std::memory_order_relaxed) + 1,
                    std::memory_order_release);
}
