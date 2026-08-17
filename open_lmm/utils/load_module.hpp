#pragma once

#include <dlfcn.h>
#include <memory>
#include <open_lmm/common/result.hpp>
#include <string>

namespace open_lmm {

void open_so(const std::string& so_name);
void* load_symbol(const std::string& so_name, const std::string& symbol_name);

// 플러그인 .so를 열고 팩토리 함수를 호출해 인스턴스를 반환.
// 성공한 플러그인 핸들은 프로세스 수명 동안 유지한다.
// 플러그인이 만든 객체가 모듈 객체보다 오래 살 수 있으므로 실행 중 dlclose는 안전하지 않다.
template <typename Module>
Result<std::shared_ptr<Module>> load_module_from_so(
    const std::string& so_name, const std::string& func_name) {
  void* raw = dlopen(so_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!raw) {
    const char* error = dlerror();
    return Result<std::shared_ptr<Module>>::Failure(Error::PluginLoadFailed(
        "dlopen failed for " + so_name + ": " +
        (error ? error : "unknown error")));
  }
  dlerror();
  void* sym = dlsym(raw, func_name.c_str());
  const char* symbol_error = dlerror();
  if (symbol_error) {
    const std::string detail = "dlsym '" + func_name + "' not found in " +
        so_name + ": " + symbol_error;
    dlclose(raw);
    return Result<std::shared_ptr<Module>>::Failure(
        Error::PluginLoadFailed(detail));
  }

  auto* create_fn = reinterpret_cast<Module*(*)()>(sym);
  Module* instance = create_fn();
  if (!instance) {
    dlclose(raw);
    return Result<std::shared_ptr<Module>>::Failure(Error::PluginLoadFailed(
        "Factory '" + func_name + "' returned nullptr in " + so_name));
  }

  // Intentional process-lifetime handle: the OS unloads it at process exit.
  return Result<std::shared_ptr<Module>>::Ok(std::shared_ptr<Module>(instance));
}

}  // namespace open_lmm
