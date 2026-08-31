#include "fixture_trace.hpp"
#include "host/gui_plugin_host.hpp"
#include "host/gui_plugin_module.hpp"

#include <dlfcn.h>

#include <dirent.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

using open_lmm::GuiPluginHost;
using open_lmm::GuiPluginModule;

constexpr std::size_t kValidationFixtureCount = 14;
constexpr std::size_t kFixtureCount = 16;
constexpr int kStressCycles = 1000;

#if defined(__has_feature)
#if __has_feature(address_sanitizer)
#define OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED 1
#endif
#endif
#if defined(__SANITIZE_ADDRESS__) && \
    !defined(OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED)
#define OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED 1
#endif
#if !defined(OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED)
#define OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED 0
#endif

constexpr bool kAddressSanitized = OPEN_LMM_GUI_PROBE_ADDRESS_SANITIZED != 0;
constexpr std::size_t kRssGrowthLimitBytes =
    (kAddressSanitized ? 24U : 16U) * 1024U * 1024U;

void Require(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

std::size_t OpenFileDescriptorCount() {
  std::size_t count = 0;
  if (DIR* directory = opendir("/proc/self/fd")) {
    while (readdir(directory)) ++count;
    closedir(directory);
  }
  return count >= 2 ? count - 2 : count;
}

std::size_t ResidentBytes() {
  std::ifstream statm("/proc/self/statm");
  std::size_t pages = 0;
  std::size_t resident = 0;
  statm >> pages >> resident;
  return resident * static_cast<std::size_t>(sysconf(_SC_PAGESIZE));
}

bool IsUnloaded(const std::string& path) {
  void* handle = dlopen(path.c_str(), RTLD_NOW | RTLD_NOLOAD);
  if (!handle) return true;
  (void)dlclose(handle);
  return false;
}

class UnloadHook final {
 public:
  UnloadHook() {
    static std::uint64_t sequence = 0;
    path_ = std::filesystem::temp_directory_path() /
            ("open_lmm_gui_unload_trace_" + std::to_string(getpid()) + "_" +
             std::to_string(++sequence));
    std::error_code error;
    std::filesystem::remove(path_, error);
    Require(setenv("OPEN_LMM_GUI_PROBE_UNLOAD_TRACE_FILE",
                   path_.c_str(), 1) == 0,
            "failed to configure the fixture unload trace");
  }

  ~UnloadHook() {
    (void)unsetenv("OPEN_LMM_GUI_PROBE_UNLOAD_TRACE_FILE");
    std::error_code error;
    std::filesystem::remove(path_, error);
  }

  UnloadHook(const UnloadHook&) = delete;
  UnloadHook& operator=(const UnloadHook&) = delete;

  [[nodiscard]] std::size_t Count() const {
    std::ifstream trace(path_);
    return static_cast<std::size_t>(
        std::count(std::istreambuf_iterator<char>(trace),
                   std::istreambuf_iterator<char>(), '\n'));
  }

 private:
  std::filesystem::path path_;
};

void CheckRejectedBeforeCreate(const std::string& path,
                               bool require_unload_hook = false) {
  auto unload = require_unload_hook ? std::make_unique<UnloadHook>() : nullptr;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(!loaded, path + " must be rejected");
  Require(trace.create_order.load() == 0,
          path + " must be rejected before create");
  Require(trace.destroy_order.load() == 0,
          path + " must not destroy an uncreated instance");
  Require(IsUnloaded(path), path + " must be unloaded after rejection");
  if (unload) {
    Require(unload->Count() == 1,
            path + " must execute its unload hook exactly once");
  }
}

void CheckRejectedAfterCreate(const std::string& path,
                              bool require_unload_hook = false) {
  auto unload = require_unload_hook ? std::make_unique<UnloadHook>() : nullptr;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(!loaded, path + " create failure must be rejected");
  Require(trace.create_order.load() != 0,
          path + " must exercise the create call");
  Require(trace.destroy_order.load() == 0,
          path + " must not destroy a null or unpublished instance");
  Require(trace.unload_order.load() > trace.create_order.load(),
          path + " must unload after create failure");
  Require(IsUnloaded(path), path + " must be unloaded after create failure");
  if (unload) {
    Require(unload->Count() == 1,
            path + " must execute its unload hook exactly once");
  }
}

void CheckOtherLoadFailure(const std::string& path,
                           bool require_unload_hook = false) {
  auto unload = require_unload_hook ? std::make_unique<UnloadHook>() : nullptr;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(!loaded, path + " must be rejected");
  Require(trace.create_order.load() == 0,
          path + " must not create an instance");
  Require(IsUnloaded(path), path + " must unload on failure");
  if (unload) {
    Require(unload->Count() == 1,
            path + " must execute its unload hook exactly once");
  }
}

void CheckMissingFile(const std::string& valid_path) {
  const auto missing =
      (std::filesystem::path(valid_path).parent_path() /
       "does-not-exist.so")
          .string();
  Require(!GuiPluginModule::Load(missing), "missing DSO must fail closed");
}

void CheckValidLifecycle(const std::string& path,
                         GuiLoaderProbeTrace* observed_trace = nullptr,
                         bool require_unload_hook = false) {
  auto unload = require_unload_hook ? std::make_unique<UnloadHook>() : nullptr;
  GuiLoaderProbeTrace local_trace;
  auto& trace = observed_trace ? *observed_trace : local_trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(loaded.IsOk(), "valid GUI fixture must load");
  auto module = std::move(loaded).Value();
  Require(module->Plugin().Start({}).IsOk(), "valid GUI fixture must start");
  Require(module->Plugin().IsOpen(), "started GUI fixture must be open");
  module->Plugin().RequestStop();
  module->Plugin().Join();
  module.reset();

  const auto start = trace.start_order.load();
  const auto stop = trace.request_stop_order.load();
  const auto join = trace.join_order.load();
  const auto destroy = trace.destroy_order.load();
  const auto unload_order = trace.unload_order.load();
  Require(trace.create_order.load() < start && start < stop && stop < join &&
              join < destroy && destroy < unload_order,
          "valid lifecycle must be create/start/stop/join/destroy/unload");
  Require(IsUnloaded(path), "valid GUI fixture must be unloaded");
  if (unload) {
    Require(unload->Count() == 1,
            path + " must execute its unload hook exactly once");
  }
}

void CheckNeverStarted(const std::string& path) {
  UnloadHook unload;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(loaded.IsOk(), "never-started fixture must load");
  auto module = std::move(loaded).Value();
  module.reset();
  Require(trace.create_order.load() != 0 &&
              trace.start_order.load() == 0 &&
              trace.request_stop_order.load() == 0 &&
              trace.join_order.load() == 0 &&
              trace.create_order.load() < trace.destroy_order.load() &&
              trace.destroy_order.load() < trace.unload_order.load(),
          "never-started instance must destroy before unload");
  Require(IsUnloaded(path) && unload.Count() == 1,
          "never-started fixture must unload exactly once");
}

void CheckMovedModule(const std::string& path) {
  UnloadHook unload;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginModule::Load(path, &trace);
  Require(loaded.IsOk(), "module-move fixture must load");
  auto source = std::move(loaded).Value();
  auto moved = std::make_unique<GuiPluginModule>(std::move(*source));
  source.reset();
  Require(moved->Plugin().Start({}).IsOk(), "moved module must start");
  moved->Plugin().RequestStop();
  moved->Plugin().Join();
  moved.reset();
  Require(trace.create_order.load() < trace.start_order.load() &&
              trace.start_order.load() < trace.request_stop_order.load() &&
              trace.request_stop_order.load() < trace.join_order.load() &&
              trace.join_order.load() < trace.destroy_order.load() &&
              trace.destroy_order.load() < trace.unload_order.load(),
          "moved module must retain ordered instance/DSO ownership");
  Require(IsUnloaded(path) && unload.Count() == 1,
          "moved module fixture must unload exactly once");
}

void CheckMovedHostAndDoubleStop(const std::string& path,
                                 GuiLoaderProbeTrace& trace) {
  UnloadHook unload;
  auto loaded = GuiPluginHost::Load(path, &trace);
  Require(loaded.IsOk(), "valid GUI host fixture must load");
  auto source = std::move(loaded).Value();
  auto moved = std::move(source);
  Require(!source && moved, "GUI host ownership must survive a move");
  Require(moved->Start({}).IsOk(), "moved GUI host must start");
  const auto duplicate_start = moved->Start({});
  Require(!duplicate_start &&
              duplicate_start.GetError().code ==
                  open_lmm::Error::Code::kInvalidArgument,
          "moved GUI host must reject duplicate start");
  moved->Stop();
  const auto stop_order = trace.request_stop_order.load();
  const auto join_order = trace.join_order.load();
  moved->Stop();
  Require(trace.request_stop_order.load() == stop_order &&
              trace.join_order.load() == join_order,
          "repeated Stop must not duplicate stop or join");
  moved.reset();
  Require(trace.create_order.load() < trace.start_order.load() &&
              trace.start_order.load() < stop_order &&
              stop_order < join_order &&
              join_order < trace.destroy_order.load() &&
              trace.destroy_order.load() < trace.unload_order.load(),
          "moved host must stop/join/destroy before unload");
  Require(IsUnloaded(path) && unload.Count() == 1,
          "moved host fixture must unload exactly once");
}

void LoadAndReturnEarly(const std::string& path, GuiLoaderProbeTrace& trace) {
  auto loaded = GuiPluginHost::Load(path, &trace);
  Require(loaded.IsOk(), "early-return GUI host fixture must load");
  auto host = std::move(loaded).Value();
  Require(host != nullptr, "early-return GUI host must own its module");
}

void CheckEarlyReturn(const std::string& path) {
  UnloadHook unload;
  GuiLoaderProbeTrace trace;
  LoadAndReturnEarly(path, trace);
  Require(trace.create_order.load() != 0 &&
              trace.start_order.load() == 0 &&
              trace.create_order.load() < trace.destroy_order.load() &&
              trace.destroy_order.load() < trace.unload_order.load(),
          "early return must destroy an unstarted instance before unload");
  Require(IsUnloaded(path) && unload.Count() == 1,
          "early-return fixture must unload exactly once");
}

void CheckStartFailure(const std::string& path,
                       std::string_view expected_message) {
  UnloadHook unload;
  GuiLoaderProbeTrace trace;
  auto loaded = GuiPluginHost::Load(path, &trace);
  Require(loaded.IsOk(), "start-failure GUI fixture must load");
  auto host = std::move(loaded).Value();
  const auto started = host->Start({});
  Require(!started &&
              started.GetError().code ==
                  open_lmm::Error::Code::kPluginLoadFailed &&
              started.GetError().Message().find(expected_message) !=
                  std::string::npos &&
              !host->IsOpen(),
          "Start failure must preserve its error and not publish open");
  host.reset();
  Require(trace.create_order.load() < trace.start_order.load() &&
              trace.request_stop_order.load() == 0 &&
              trace.join_order.load() == 0 &&
              trace.start_order.load() < trace.destroy_order.load() &&
              trace.destroy_order.load() < trace.unload_order.load(),
          "Start failure must destroy before unload without stop/join");
  Require(IsUnloaded(path) && unload.Count() == 1,
          "Start-failure fixture must unload exactly once");
}

void RunPublicContract(const std::vector<std::string>& paths) {
  GuiLoaderProbeTrace lifecycle_trace;
  CheckValidLifecycle(paths[0], &lifecycle_trace, true);
  for (std::size_t index = 1; index <= 8; ++index) {
    CheckRejectedBeforeCreate(paths[index], true);
  }
  for (std::size_t index = 9; index <= 10; ++index) {
    CheckRejectedAfterCreate(paths[index], true);
  }
  for (std::size_t index = 11; index < kValidationFixtureCount; ++index) {
    CheckOtherLoadFailure(paths[index], true);
  }
  CheckMissingFile(paths[0]);

  std::cout
      << "Loader-B public-contract probe passed: fixture_count="
      << kValidationFixtureCount
      << " rejected_before_create=8 rejected_after_create=2"
      << " other_load_failures=3 missing_file=1"
      << " lifecycle_orders=" << lifecycle_trace.create_order.load() << ','
      << lifecycle_trace.start_order.load() << ','
      << lifecycle_trace.request_stop_order.load() << ','
      << lifecycle_trace.join_order.load() << ','
      << lifecycle_trace.destroy_order.load() << ','
      << lifecycle_trace.unload_order.load()
      << " unload_hook_count_per_dso=1\n";
}

void RunLifetime(const std::vector<std::string>& paths) {
  CheckNeverStarted(paths[0]);
  CheckMovedModule(paths[0]);
  GuiLoaderProbeTrace moved_trace;
  CheckMovedHostAndDoubleStop(paths[0], moved_trace);
  CheckEarlyReturn(paths[0]);
  CheckStartFailure(paths[14], "probe start result failure");
  CheckStartFailure(paths[15], "probe start standard failure");
  std::cout << "Loader-B lifetime probe passed: never_started=1 moved_module=1"
               " moved_host=1"
               " duplicate_start=1 double_stop=1 early_return=1"
               " start_failure_result=1 start_failure_standard=1"
               " destroy_before_unload=all\n";
}

void CheckNegativeFixture(const std::vector<std::string>& paths,
                          std::size_t index) {
  if (index <= 8) {
    CheckRejectedBeforeCreate(paths[index]);
  } else if (index <= 10) {
    CheckRejectedAfterCreate(paths[index]);
  } else {
    CheckOtherLoadFailure(paths[index]);
  }
}

struct RssSample {
  std::size_t cycle = 0;
  std::size_t bytes = 0;
};

long double RssSlopeBytesPerCycle(const std::vector<RssSample>& samples) {
  long double sum_x = 0;
  long double sum_y = 0;
  long double sum_xx = 0;
  long double sum_xy = 0;
  for (const auto& sample : samples) {
    const auto x = static_cast<long double>(sample.cycle);
    const auto y = static_cast<long double>(sample.bytes);
    sum_x += x;
    sum_y += y;
    sum_xx += x * x;
    sum_xy += x * y;
  }
  const auto count = static_cast<long double>(samples.size());
  const auto denominator = count * sum_xx - sum_x * sum_x;
  if (denominator == 0) return 0;
  return (count * sum_xy - sum_x * sum_y) / denominator;
}

void AppendRssSample(std::vector<RssSample>& samples, std::size_t cycle) {
  samples.push_back({cycle, ResidentBytes()});
}

void RunStress(const std::vector<std::string>& paths) {
  for (std::size_t index = 1; index < kValidationFixtureCount; ++index) {
    for (int warmup = 0; warmup < 10; ++warmup) {
      CheckNegativeFixture(paths, index);
    }
  }
  for (int warmup = 0; warmup < 100; ++warmup) {
    CheckValidLifecycle(paths[0]);
  }

  const auto fd_baseline = OpenFileDescriptorCount();
  std::vector<RssSample> rss_samples;
  std::size_t measured_cycles = 0;
  AppendRssSample(rss_samples, measured_cycles);

  for (std::size_t index = 1; index < kValidationFixtureCount; ++index) {
    for (int iteration = 0; iteration < kStressCycles; ++iteration) {
      CheckNegativeFixture(paths, index);
      ++measured_cycles;
      if ((iteration + 1) % 100 == 0) {
        AppendRssSample(rss_samples, measured_cycles);
      }
    }
  }
  for (int iteration = 0; iteration < kStressCycles; ++iteration) {
    CheckMissingFile(paths[0]);
    ++measured_cycles;
    if ((iteration + 1) % 100 == 0) {
      AppendRssSample(rss_samples, measured_cycles);
    }
  }
  for (int iteration = 0; iteration < kStressCycles; ++iteration) {
    CheckValidLifecycle(paths[0]);
    ++measured_cycles;
    if ((iteration + 1) % 100 == 0) {
      AppendRssSample(rss_samples, measured_cycles);
    }
  }

  const auto fd_final = OpenFileDescriptorCount();
  const auto observed_min = std::min_element(
      rss_samples.begin(), rss_samples.end(),
      [](const RssSample& left, const RssSample& right) {
        return left.bytes < right.bytes;
      })->bytes;
  const auto final_rss = rss_samples.back().bytes;
  const auto slope = RssSlopeBytesPerCycle(rss_samples);
  const auto projected_growth = static_cast<std::size_t>(
      std::max<long double>(0, slope) * kStressCycles);

  std::ostringstream raw_samples;
  for (std::size_t index = 0; index < rss_samples.size(); ++index) {
    if (index != 0) raw_samples << ',';
    raw_samples << rss_samples[index].cycle << ':' << rss_samples[index].bytes;
  }
  std::cout << "Loader-B stress probe evidence: negative_fixture_count=13"
            << " negative_cycles_per_fixture=" << kStressCycles
            << " missing_file_cycles=" << kStressCycles
            << " valid_cycles=" << kStressCycles
            << " measured_cycles=" << measured_cycles
            << " loaded_dso_balance=0 fd_baseline=" << fd_baseline
            << " fd_final=" << fd_final
            << " rss_observed_min_bytes=" << observed_min
            << " rss_final_bytes=" << final_rss
            << " rss_slope_bytes_per_cycle="
            << static_cast<long long>(std::llround(slope))
            << " rss_projected_1000_cycle_growth_bytes=" << projected_growth
            << " rss_growth_limit_bytes=" << kRssGrowthLimitBytes
            << " address_sanitized=" << (kAddressSanitized ? 1 : 0)
            << " rss_samples=" << raw_samples.str() << '\n';

  Require(fd_final <= fd_baseline + 1,
          "repeated GUI load/unload must not leak file descriptors");
  Require(final_rss <= observed_min + kRssGrowthLimitBytes,
          "repeated GUI load/unload RSS must remain bounded from observed minimum");
  Require(projected_growth <= kRssGrowthLimitBytes,
          "repeated GUI load/unload RSS slope must remain bounded");
  std::cout << "Loader-B stress probe passed\n";
}

}  // namespace

int main(int argc, char** argv) {
  Require(argc == static_cast<int>(kFixtureCount + 2),
          "a mode and 16 GUI loader fixture paths are required");
  const std::string_view mode(argv[1]);
  const std::vector<std::string> paths(argv + 2, argv + argc);
  if (mode == "public-contract") {
    RunPublicContract(paths);
  } else if (mode == "lifetime") {
    RunLifetime(paths);
  } else if (mode == "stress") {
    RunStress(paths);
  } else {
    Require(false, "unknown GUI loader probe mode");
  }
  return 0;
}
