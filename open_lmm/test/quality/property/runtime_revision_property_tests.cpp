#include <runtime/state/runtime_state_store.hpp>

#include "property_generator.hpp"

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <memory>

namespace {

std::shared_ptr<const open_lmm::RuntimeState> State(uint64_t revision) {
  auto state = std::make_shared<open_lmm::RuntimeState>();
  state->revision = revision;
  return state;
}

}  // namespace

int main() {
  using open_lmm::test::property::Fail;
  using open_lmm::test::property::Generator;
  const uint64_t seed = open_lmm::test::property::Seed();
  const std::size_t cases = open_lmm::test::property::Cases(500);
  Generator generator(seed);
  auto authoritative = State(generator.Next() % 1000U);
  open_lmm::RuntimeStateStore store(authoritative);

  for (std::size_t index = 0; index < cases; ++index) {
    const uint64_t revision = authoritative->revision;
    const std::size_t operation = generator.Index(4);
    if (operation == 0) {
      auto candidate = State(revision + 1);
      const auto result = store.Commit(authoritative, candidate);
      if (!result || store.Snapshot().get() != candidate.get() ||
          store.Snapshot()->revision != revision + 1) {
        Fail("revision-valid-commit", seed, index,
             "valid candidate did not advance exactly once");
      }
      authoritative = std::move(candidate);
      continue;
    }

    const auto before = store.Snapshot();
    open_lmm::Result<void> result = open_lmm::Result<void>::Ok();
    if (operation == 1) {
      result = store.Commit(State(revision), State(revision + 1));
    } else if (operation == 2) {
      result = store.Commit(authoritative, State(revision));
    } else {
      result = store.Commit(authoritative, State(revision + 2));
    }
    if (result || store.Snapshot().get() != before.get() ||
        store.Snapshot()->revision != revision) {
      Fail("revision-rejected-commit", seed, index,
           "stale or non-monotonic candidate changed authority");
    }
  }
  std::cout << "runtime revision properties passed seed=" << seed
            << " cases=" << cases << '\n';
  return 0;
}
