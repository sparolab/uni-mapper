#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/logging.hpp>
#include <string>

namespace open_lmm {
namespace {
std::string DynamicLoaderError() {
  const char* error = dlerror();
  return error ? error : "unknown dynamic loader error";
}
}  // namespace

// Legacy helpers retained for source compatibility. New plugins use ABI v1.
void open_so(const std::string& so_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    LogWarning("[load_module] open_so failed for " + so_name + ": " +
               DynamicLoaderError());
  }
}

void* load_symbol(const std::string& so_name, const std::string& symbol_name) {
  void* handle = dlopen(so_name.c_str(), RTLD_LAZY);
  if (handle == nullptr) {
    LogError("[load_module] dlopen failed for " + so_name + ": " +
             DynamicLoaderError());
    return nullptr;
  }

  void* func = dlsym(handle, symbol_name.c_str());
  if (func == nullptr) {
    LogError("[load_module] dlsym '" + symbol_name + "' not found in " +
             so_name + ": " + DynamicLoaderError());
    dlclose(handle);
  }
  return func;
}

}  // namespace open_lmm
