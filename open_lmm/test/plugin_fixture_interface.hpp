#pragma once

struct PluginFixtureCounters {
  int creates = 0;
  int destroys = 0;
};

class PluginFixture {
 public:
  virtual ~PluginFixture() = default;
  [[nodiscard]] virtual int Value() const = 0;
};
