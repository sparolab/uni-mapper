#include <open_lmm/utils/load_module.hpp>
#include <spdlog/spdlog.h>
#include <string>

namespace open_lmm {

[[noreturn]] void module_load_fatal(const std::string& message) {
  spdlog::error("[load_module] {}", message);
  std::exit(1);
}

// Legacy helpers — 직접 사용 지양, load_module_from_so 사용 권장
void open_so(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("[load_module] open_so failed for {}: {}", so_name, dlerror());
  }
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::error("[load_module] dlopen failed for {}: {}", so_name, dlerror());
    return nullptr;
  }

  void* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    spdlog::error("[load_module] dlsym '{}' not found in {}: {}",
                  symbol_name, so_name, dlerror());
    dlclose(handle);
  }
  return func;
}

}  // namespace open_lmm
