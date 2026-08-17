#pragma once

#include <cstdlib>
#include <dlfcn.h>
#include <memory>
#include <string>

namespace open_lmm {

void open_so(const std::string& so_name);
void* load_symbol(const std::string& so_name, const std::string& symbol_name);
[[noreturn]] void module_load_fatal(const std::string& message);

// 플러그인 .so를 열고 팩토리 함수를 호출해 인스턴스를 반환.
// 성공한 플러그인 핸들은 프로세스 수명 동안 유지한다.
// 플러그인이 만든 객체가 모듈 객체보다 오래 살 수 있으므로 실행 중 dlclose는 안전하지 않다.
template <typename Module>
std::shared_ptr<Module> load_module_from_so(const std::string& so_name,
                                             const std::string& func_name) {
  void* raw = dlopen(so_name.c_str(), RTLD_LAZY | RTLD_GLOBAL);
  if (!raw) {
    const char* error = dlerror();
    module_load_fatal("dlopen failed for " + so_name + ": " +
                      (error ? error : "unknown error"));
  }
  void* sym = dlsym(raw, func_name.c_str());
  if (!sym) {
    const char* error = dlerror();
    module_load_fatal("dlsym '" + func_name + "' not found in " + so_name +
                      ": " + (error ? error : "unknown error"));
  }

  auto* create_fn = reinterpret_cast<Module*(*)()>(sym);
  Module* instance = create_fn();
  if (!instance) {
    dlclose(raw);
    module_load_fatal("Factory '" + func_name +
                      "' returned nullptr in " + so_name);
  }

  // Intentional process-lifetime handle: the OS unloads it at process exit.
  return std::shared_ptr<Module>(instance);
}

}  // namespace open_lmm
