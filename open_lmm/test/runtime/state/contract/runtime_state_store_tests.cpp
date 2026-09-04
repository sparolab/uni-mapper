#include <runtime/state/runtime_state_store.hpp>
#include "support/check.hpp"

#include <cstdlib>
#include <iostream>
#include <memory>

namespace {
using namespace open_lmm;

std::shared_ptr<const RuntimeState> State(uint64_t revision) {
  auto state = std::make_shared<RuntimeState>();
  state->revision = revision;
  return state;
}

void TestCandidateAndSideEffectBarrier() {
  const auto base = State(4);
  RuntimeStateStore store(base);
  bool side_effect_ran = false;
  const auto rejected = store.CommitWithBarrier(base, State(5), [&] {
    side_effect_ran = true;
    return Result<RuntimeCommitOutcome>::Failure(
        Error::IoError("fixture side-effect failure"));
  });
  Check(!rejected && side_effect_ran && store.Snapshot().get() == base.get(),
        "side-effect failure preserves canonical committed state");

  const auto stale = State(4);
  side_effect_ran = false;
  const auto conflict = store.CommitWithBarrier(stale, State(5), [&] {
    side_effect_ran = true;
    return Result<RuntimeCommitOutcome>::Ok({});
  });
  Check(!conflict && !side_effect_ran && store.Snapshot().get() == base.get(),
        "stale identity is rejected before side effects");

  Check(!store.Commit(base, State(6)),
        "candidate must advance the expected revision exactly once");
  Check(store.Commit(base, State(5)).IsOk() && store.Snapshot()->revision == 5,
        "validated candidate becomes the single committed authority");
}

void TestRecoveryAuthorityIsPersistent() {
  const auto base = State(10);
  RuntimeStateStore store(base);
  auto recovery = std::make_shared<Error>(Error::IoError("recovery fixture"));
  recovery->MarkFatalRuntime().WithRuntimeRevision(11);
  const auto committed = store.CommitWithBarrier(base, State(11), [recovery] {
    return Result<RuntimeCommitOutcome>::Ok({recovery});
  });
  const auto authority = store.AuthoritySnapshot();
  Check(committed && authority.state && authority.state->revision == 11 &&
            authority.recovery_required == recovery,
        "state and recovery health publish as one authority snapshot");

  bool side_effect_ran = false;
  const auto blocked = store.CommitWithBarrier(authority.state, State(12), [&] {
    side_effect_ran = true;
    return Result<RuntimeCommitOutcome>::Ok({});
  });
  Check(!blocked && !side_effect_ran &&
            store.AuthoritySnapshot().recovery_required == recovery,
        "recovery-required authority gates later mutation before side effects");

  auto mismatched = std::make_shared<Error>(Error::IoError("imported health"));
  mismatched->WithRuntimeRevision(99);
  store.Initialize(State(20), mismatched);
  const auto normalized = store.AuthoritySnapshot();
  Check(normalized.recovery_required &&
            normalized.recovery_required->severity ==
                Error::Severity::kFatalRuntime &&
            normalized.recovery_required->context.runtime_revision == 20,
        "imported recovery health is normalized to committed authority");
}

void TestMatchAndExplicitRecoveryLatchContracts() {
  const auto base = State(30);
  RuntimeStateStore store(base);
  Check(store.Matches(base),
        "the exact committed state is recognized as authoritative");
  Check(!store.Matches(State(30)) &&
            !store.Matches(std::shared_ptr<const RuntimeState>{}),
        "matching revision without pointer identity is not authoritative");

  auto recovery = std::make_shared<Error>(Error::IoError("latched recovery"));
  recovery->MarkFatalRuntime().WithRuntimeRevision(base->revision);
  store.LatchRecoveryRequired(recovery);
  const auto authority = store.AuthoritySnapshot();
  Check(authority.recovery_required == recovery,
        "explicit recovery latch becomes authoritative health");

  bool side_effect_ran = false;
  const auto blocked = store.CommitWithBarrier(base, State(31), [&] {
    side_effect_ran = true;
    return Result<RuntimeCommitOutcome>::Ok({});
  });
  Check(!blocked && !side_effect_ran,
        "explicit recovery latch gates mutation before side effects");

  RuntimeStateStore uninitialized(nullptr);
  uninitialized.LatchRecoveryRequired(recovery);
  Check(!uninitialized.AuthoritySnapshot().recovery_required,
        "recovery cannot be latched without committed authority");
}

}  // namespace

int main() {
  TestCandidateAndSideEffectBarrier();
  TestRecoveryAuthorityIsPersistent();
  TestMatchAndExplicitRecoveryLatchContracts();
  std::cout << "runtime state store contract tests passed\n";
  return 0;
}
