#include <open_lmm/utils/load_module.hpp>
#include <dlfcn.h>
#include <spdlog/spdlog.h>
#include <string>

namespace open_lmm {

void open_so(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::warn("[load_module] Failed to open {}: {}", so_name, dlerror());
  }
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    spdlog::error("[load_module] Failed to open {}: {}", so_name, dlerror());
    return nullptr;
  }

  auto* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    spdlog::error("[load_module] Failed to find symbol '{}' in {}: {}",
                  symbol_name, so_name, dlerror());
  }

  return func;
}

}  // namespace open_lmm
