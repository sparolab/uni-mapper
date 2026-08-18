#include <open_lmm/server/runtime_client.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_move_constructible_v<open_lmm::RuntimeClient>);
  static_assert(!std::is_copy_constructible_v<open_lmm::RuntimeClient>);
  open_lmm::RuntimeClient client(1);
  return client.SessionIds().empty() ? 0 : 1;
}
