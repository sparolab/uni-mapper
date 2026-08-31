# Architecture Freeze Baseline

## Decision

`59e003ebc4b7d44597ced4ddab3436adec310370` on `develop` is the OpenLMM
architecture stabilization baseline. The baseline is **FROZEN** for architecture
purposes: later features and fixes must extend the owners and ports below rather
than create parallel ownership or reverse the dependency direction.

This decision adopts the completed C1-C7 evidence in
`final_impl_result/FINAL_FREEZE_READINESS.md`. Production code at the audited
commit is the source of truth. Uncommitted working-tree changes are not part of
the freeze checkpoint.

## Audited C1-C7 Baseline

| Cleanup | Status | Baseline commit | Freeze evidence |
|---|---|---|---|
| C1 — post-commit reconciliation | DONE | `8afacd3` | Committed authority is reconciled before late cancellation. |
| C2 — recovery-required health | DONE | `78798fe` | Committed degraded health remains authoritative. |
| C3 — unpublished Open rollback | DONE | `dbd5fc1` | Failed/cancelled unpublished output is removed before lifecycle release. |
| C4 — MapUpdate admission parity | DONE | `af6513f` | Sequential and parallel paths share admitted per-agent lifetime. |
| C5 — runtime model/type placement | DONE | `e0bed77` | Runtime model, state, and execution ownership/direction are separated. |
| C6 — production port placement | DONE | `6832060` | Runtime composition is the sole production `MapServer` owner. |
| C7 — plugin loading ownership | DONE | `59e003e` | Host composition owns plugin inspection/loading; domain owns no loader policy. |

`FINAL_FREEZE_READINESS.md` records the independent review, exact clean-build,
package, ROS, sanitizer, deterministic regression, and forbidden-edge evidence
for this seven-commit baseline. No C1-C7 blocker remains.

## Canonical Ownership and Commit Boundaries

| Concern | Canonical owner / boundary | Frozen responsibility |
|---|---|---|
| Committed runtime state and recovery-required health | `RuntimeStateStore` | The only owner allowed to replace the externally visible committed state pointer and latch authoritative recovery health. |
| Candidate state | `RuntimeTransaction` | Builds and validates an isolated working state from an immutable base; it never mutates committed state in place. |
| State/file commit ordering | `StageCoordinator::CommitCandidate` + `RuntimeStateStore::CommitWithBarrier` | Coordinates candidate validation, file side effects, and state publication. Pre-commit failure preserves the old state; a committed cleanup fault publishes committed state plus recovery-required health. |
| Pending files | `PendingOutputSet` / `FileSetTransaction` | Own temporary files through commit or rollback. Destruction cannot silently become the owner of future runtime health. |
| Runtime lifecycle and epoch | `RuntimeService` | Owns Open/Replace/Close, the active runtime instance, epoch isolation, public job/event identity, operation leases, and subscription lifetime. |
| Command scheduling and cancellation reconciliation | `PipelineController` | Serializes mutating commands, owns the active cancellation capability and job journal, accepts committed receipts before late cancellation, and reconciles failure against the query port. |
| Stage runtime execution | `StageExecutor` | Owns execution components behind the production port without becoming a second lifecycle or committed-state owner. |
| Stage transaction barrier | `StageCoordinator` | Owns stage ordering, candidate commit, save commit, and reconfiguration dispatch. |
| Runtime reconfiguration | `RuntimeReconfigurer` | Derives and validates configuration candidates through the existing transaction path. |
| Production runtime port | runtime composition `MapServer` | The single concrete `StageRuntimePort`; it delegates runtime ownership to `StageExecutor`. |
| Algorithm/plugin composition | plugin host `AlgorithmFactory` | Selects, inspects, and loads algorithms/plugins, then passes typed lifetime-owning handles toward domain implementations. |
| Presentation | projector / worker / repository / presentation-state owners | Derived, generation-checked state. The last valid presentation remains visible until a ready replacement commits. |
| Public façade | `RuntimeClient` PImpl | Stable client entry point; adapters consume it without importing runtime internals. |

## Frozen Invariants

The repository `AGENTS.md` is the authoritative development rule. It freezes:

1. `RuntimeStateStore` as the single committed-state owner.
2. `candidate -> validate -> commit barrier -> committed state`; failure before
   commit preserves the previous committed state.
3. A valid committed receipt wins over cancellation observed after commit.
4. `committed + recovery_required` remains authoritative and is not rollback.
5. Each runtime replacement creates a new epoch; retired jobs, events, callbacks,
   or presentation work cannot publish into the active epoch.
6. Presentation uses `visible -> pending -> ready -> commit`; stale generations
   cannot replace newer visible state.
7. `runtime/state` must not depend on `runtime/execution`.
8. `domain` must not depend on runtime internals or plugin-host loading.
9. Runtime composition/service must not depend on adapters; adapters are leaves.
10. Installed public headers must not expose private implementation types, and
    `RuntimeClient` PImpl and the documented same-toolchain plugin ABI v1 policy
    remain intact.

## Dependency Direction

```text
public contracts / foundation
          -> private runtime model
          -> runtime state
          -> runtime execution
          -> runtime composition / service
          -> RuntimeClient
          -> GUI / ROS / Batch adapters

plugin host / AlgorithmFactory
          -> domain algorithms + plugin interfaces

storage transaction outcome
          -> runtime commit health
```

The direction is about ownership and compile-time dependency, not only folder
names. An adapter may call the public façade, but runtime service must not call an
adapter. Domain implementations may receive typed plugin owners, but may not
select or load a DSO.

## Forbidden-Edge Enforcement

`open_lmm/test/architecture_boundary_tests.cmake`, registered as
`open_lmm_architecture_boundary_tests`, is the executable architecture gate.
`.github/workflows/compiler-matrix.yml` runs it for pull requests, merge queues,
and pushes through both every full CTest matrix entry and the dedicated
`policy / architecture-boundary` job.

The gate protects the C5-C7 corrections directly:

- **C5:** zero state-to-execution includes, zero domain-to-runtime includes,
  model-before-state-before-execution target order, one runtime-model owner, and
  no legacy `stage_runner` files.
- **C6:** one production `map_server.cpp` owner in runtime composition, zero
  runtime-service-to-adapter includes, no batch dependency on runtime internals,
  and no legacy batch compatibility port.
- **C7:** zero domain includes/calls of plugin-host loaders or inspectors, zero
  domain CMake dependency on the plugin host, one plugin-host object owner, and
  plugin lifetime/selection behavioral tests.

The same gate also checks canonical runtime/config/storage/foundation/client/GUI
target owners, public-header privacy, `MapServer` façade ownership, presentation
generation paths, and resource-admission boundaries. Behavioral invariants remain
covered by the runtime transaction, controller, service, E2E, GUI presentation,
plugin ABI, and plugin selection suites; source checks are not treated as a
substitute for those tests.

Goal 01 strengthened this gate to scan all supported C/C++ header/source
extensions, reject private runtime includes and private runtime target links from
all current adapters, reject model/state/service reverse target dependencies,
count the production port owner across all CMake targets, and reject plugin-host
or direct dynamic-loader dependencies across the entire domain tree. The release
policy test now also verifies that the workflow invokes the policy script and
that the script invokes both policy suites.

These are executable regression guards, not a formal proof of every future
semantic edge. Generated sources, novel indirection, public ABI review, and
GitHub branch-protection configuration still require the architecture-change
admission review below.

## Supported Extension Points

- Public applications and adapters extend behavior through `RuntimeClient` and
  installed public contracts.
- Runtime commands and queries cross the internal `StageCommandPort` and
  `RuntimeQueryPort` boundary; new adapters do not reach through it.
- Algorithm selection extends through `AlgorithmProvider` / `AlgorithmFactory`.
- Plugin ABI v1 remains a same-toolchain extension point with existing metadata,
  capability, and destroy-before-`dlclose` rules; it is not a portable binary SDK
  promise.
- New configuration behavior extends the existing schema/domain/application
  pipeline and `RuntimeReconfigurer`.
- New visualization behavior extends the projector/worker/repository/
  presentation-state pipeline without owning committed runtime state.

## Architecture-Change Admission Rule

The authoritative admission checklist is recorded in `AGENTS.md`. A proposal
must name affected invariants, owners, dependency edges, and compatibility
contracts; provide concrete ownership/correctness/dependency or measured resource
evidence; explain why existing extension points are insufficient; define
migration and rollback; add executable invariant regression tests; and pass all
required policy, build, test, package, ROS, and applicable sanitizer gates.

If the proposal breaks a frozen invariant, implementation must stop until the
architecture change is reviewed separately and explicitly approved. Directory
symmetry or reduced LOC is not sufficient evidence.

## Freeze Checkpoint Candidate

- Commit: `59e003ebc4b7d44597ced4ddab3436adec310370`
- Candidate annotated tag: `architecture-freeze-v3.0.0`
- Tag message: `OpenLMM architecture freeze after C1-C7 stabilization`
- Required evidence before publishing the tag: the seven stable checks named in
  `RELEASE_POLICY.md`, an exact clean-checkout HEAD match, and no unresolved
  architecture or compatibility exception.

This document records the candidate; it does not create or publish the tag.

## Deferred Work and Known Limits

The freeze does not promote intentionally deferred work into an architecture
blocker. Hard-RSS guarantees, streaming PCD decode, plugin-private memory
accounting, public recovery-detail ABI, crash journaling/directory durability,
portable external plugin ABI, broad directory reshaping, and public DTO cleanup
remain separate post-freeze product decisions. Real large-dataset replay and RSS
measurement still require datasets not present in this workspace.

## Validation

Validation ran on 2026-08-20 UTC from base HEAD
`59e003ebc4b7d44597ced4ddab3436adec310370`, including the Goal 01 policy changes
and the pre-existing working-tree changes. The CI script required a nonexistent
configuration path and therefore configured a fresh build/install/package tree.
Uncommitted files remain excluded from the tag candidate itself.

| Gate | Command / evidence | Result |
|---|---|---|
| Clean GCC 12 GUI-on build | `scripts/ci/build_and_test.sh goal01-freeze-gcc12-gui-on /usr/bin/gcc-12 /usr/bin/g++-12 ON` | PASS |
| Full OpenLMM CTest | fresh build JUnit and CI log | **31/31 PASS** |
| Full ROS CTest | fresh installed-prefix build JUnit and CI log | **1/1 PASS** |
| Source-free package consumer | runtime, contracts-only, client-only, PluginSDK-only, and public-header self-containment fixtures | PASS |
| Architecture/release policy | `scripts/ci/check_architecture_policy.sh` | PASS |
| Patch hygiene | `git diff --check` | PASS |

The exact tag-target commit also retains the earlier clean 32/32 standalone
CTest, package/public-header, ROS installed-prefix, ASan+UBSan 13/13, and TSan
6/6 evidence recorded in `FINAL_FREEZE_READINESS.md`.

## Completion Statement

The Goal 01 architecture freeze baseline is complete. Freeze HEAD, invariant
owners, forbidden edges, supported extension points, architecture-change
admission, and the checkpoint candidate are explicit and all required current
validation gates pass. No production behavior, public API/ABI, plugin ABI, or
runtime architecture change is introduced by Goal 01.
