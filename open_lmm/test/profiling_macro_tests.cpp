#include <open_lmm/common/profiling.hpp>

#include <iostream>
#include <string>

namespace {

[[maybe_unused]] int side_effect(int& value) {
  return ++value;
}

}  // namespace

int main() {
  int evaluations = 0;

  {
    OPEN_LMM_ZONE();
  }
  {
    OPEN_LMM_ZONE_N("ProfilingMacroTest");
    const std::string zone_text = "profiling macro smoke test";
    OPEN_LMM_ZONE_TEXT(zone_text);
  }
  OPEN_LMM_PLOT("profiling.test", side_effect(evaluations));
  OPEN_LMM_FRAME_MARK();
  OPEN_LMM_THREAD_NAME("profiling-test");

#if OPEN_LMM_ENABLE_TRACY
  if (evaluations != 1) {
    std::cerr << "Tracy-enabled plot argument was not evaluated exactly once\\n";
    return 1;
  }
#else
  if (evaluations != 0) {
    std::cerr << "Disabled profiling macro evaluated its argument\\n";
    return 1;
  }
#endif

  return 0;
}
