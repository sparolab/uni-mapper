#pragma once

#include <open_lmm/common/cancellation.hpp>

#include <functional>
#include <future>
#include <memory>

struct PluginFixtureCounters {
  int creates = 0;
  int destroys = 0;
};

class PluginFixture {
 public:
  virtual ~PluginFixture() = default;
  [[nodiscard]] virtual int Value() const = 0;
  [[nodiscard]] virtual bool RunCancelable(
      const std::shared_ptr<open_lmm::CancellationToken>& cancellation,
      const std::function<void()>& entered,
      const std::shared_future<void>& release) = 0;
};
