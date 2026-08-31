# Architecture Invariants

OpenLMM architecture is considered frozen at the current stabilization baseline.

New features, fixes, and refactors must preserve the following invariants.

## Runtime State

- `RuntimeStateStore` is the single canonical owner of committed runtime state.
- Pending/candidate work must never mutate committed state in place.
- State changes follow:
  candidate → validate → commit barrier → committed state.
- Failure before commit preserves the previous committed state.
- File side effects and runtime-state publication must preserve their defined commit ordering.
- Every opened or replaced runtime has a distinct epoch. Jobs, events, callbacks,
  and presentation work from a retired epoch must never publish into the active
  runtime.

## Cancellation / Command Semantics

- Cancellation observed before commit may prevent the commit.
- Once a valid committed receipt exists, committed success wins over a later cancellation request.
- A mutating failure/exception must be reconciled against authoritative query state.
- Controller-visible runtime revision must never silently diverge from authoritative runtime state.

## Recovery Health

- `committed + recovery_required` is not equivalent to rollback/failure.
- Committed files/state remain authoritative.
- Recovery-required health must remain authoritative until explicitly reconciled or replaced.
- Diagnostic logs must not be the sole owner of state that changes future runtime behavior.

## Presentation / Visualization

- A valid visible presentation must remain until its replacement is ready.
- Normal transitions use:
  visible → pending → ready → commit.
- Do not introduce `clear → rebuild` transitions when a valid previous presentation exists.
- Stale visualization generations must never replace newer presentation state.
- Presentation state is derived state and must not become a second owner of committed runtime state.

## Dependency Direction

The intended internal dependency direction is:

public contracts / foundation
    ↓
private runtime model
    ↓
runtime state
    ↓
runtime execution
    ↓
runtime composition / service
    ↓
RuntimeClient
    ↓
GUI / ROS / Batch adapters

Plugin composition direction:

plugin host / AlgorithmFactory
    ↓
domain algorithms + plugin interfaces

Rules:

- `runtime/state` must not depend on `runtime/execution`.
- `domain` must not depend on runtime internals.
- `domain` must not load plugins or depend on `plugins/host`.
- `RuntimeService` must not depend on adapters.
- Adapters remain leaf consumers of the public/runtime façade.
- Do not introduce duplicate composition owners.

## Public API / ABI

- Internal implementation types under `src/` must not leak into installed public headers.
- `RuntimeClient` PImpl boundaries must be preserved.
- Public API/ABI changes require explicit compatibility review.
- Plugin ABI v1 remains a same-toolchain compatibility model unless a separate SDK evolution project changes that policy.
- Do not silently strengthen compatibility guarantees beyond the documented release policy.

## Resource Ownership

- Long-lived/unbounded containers require an explicit retention or eviction policy.
- Expensive runtime work should participate in the appropriate resource-admission policy.
- A bounded output does not imply bounded peak memory.
- Do not introduce large hidden copies across runtime / visualization / adapter boundaries without measurement.

## Structures That Should Not Be Refactored Without Strong Evidence

Do not split or collapse these boundaries merely to reduce LOC:

- `RuntimeService` lifecycle / epoch / job / event / lease authority
- `StageCoordinator` transaction barrier
- `StageExecutor` runtime ownership façade
- `RuntimeReconfigurer`
- `RuntimeTransaction` vs `RuntimeStateStore`
- `PendingOutputSet` / `OutputRepository`
- `StageCommandPort` / `RuntimeQueryPort`
- visualization projector / worker / repository / presentation-state separation
- `RuntimeClient` / GUI PImpl boundaries
- plugin DSO lifetime ownership

A refactor of one of these requires evidence that it removes a real dependency,
ownership problem, correctness issue, or measurable bottleneck.

## Architecture Change Admission

An architecture-changing change is admitted only when its proposal:

1. names every affected invariant, canonical owner, dependency edge, and public
   API/ABI contract;
2. provides concrete evidence of a real ownership, dependency, correctness, or
   measured resource problem;
3. explains why the existing extension points cannot satisfy the requirement
   without introducing a parallel owner;
4. defines migration and rollback boundaries, including compatibility impact;
5. adds executable regression coverage for every affected invariant; and
6. passes the architecture/release policy gates and the required build, test,
   package-consumer, ROS, and applicable sanitizer checks.

If satisfying a request would break a frozen invariant, stop production
implementation and submit the architecture change as a separate proposal for
explicit review. LOC reduction or directory symmetry alone is not admission
evidence.

## Feature Development Rule

Before implementing a feature:

1. Identify which modules and architecture invariants it touches.
2. Prefer extending an existing owner/boundary rather than introducing a parallel owner.
3. Do not bypass transaction, lifecycle, presentation, or resource-admission boundaries.
4. Add regression tests for any invariant affected by the feature.
5. If the feature appears to require breaking an invariant, stop and propose the architecture change separately before implementation.
