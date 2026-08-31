# Goal 02 — Test Architecture Reorganization Implementation Result

## Document Status

- Status: **IMPLEMENTED AND VALIDATED**
- Implementation base: `5a29664`
- Deterministic production seam commit: `f4154eb`
- Frozen architecture baseline: `59e003e`
- Goal 01 policy baseline: `73aa86a`
- Audit date: 2026-08-20 UTC
- Scope: test sources, test-only support, CTest/CMake registration, package and
  policy fixtures, ROS tests, CI and sanitizer selection
- Production redesign: **prohibited**

This document records both the authoritative Goal 02 specification and its
validated implementation result. `final_impl_result/10_test_architecture.md`
remains historical evidence from pre-C1-C7 HEAD `45afae7`; the evidence below
was collected from the current implementation instead of copying that older
protection verdict.

## Objective

Reorganize OpenLMM tests around canonical module ownership, test layer, workflow,
and frozen invariant. The implementation must improve what the suite proves; a
directory-only move is insufficient.

The implementation must:

1. preserve every meaningful current assertion before removing or consolidating
   a fixture;
2. assign exactly one primary module owner and one primary L1-L6 layer to every
   registered test;
3. distinguish static architecture policy from executable behavioral evidence;
4. add private logical-module compile contracts for the C5-C7 boundaries;
5. replace scheduler-timing assertions with deterministic synchronization; and
6. keep production ownership, public API/ABI, plugin ABI, and adapter dependency
   direction unchanged.

## Architecture Scope Guard

Goal 02 may change:

- `open_lmm/test/**`;
- `ros/test/**` and test registration in `ros/CMakeLists.txt`;
- test-only CMake helpers and fixtures;
- `scripts/ci/**` and `.github/workflows/compiler-matrix.yml` only as needed to
  select the reorganized suites; and
- architecture/release policy tests when needed to protect test ownership.

Goal 02 must not:

- move or split `RuntimeService`, `StageCoordinator`, `StageExecutor`,
  `RuntimeStateStore`, plugin-host, visualization, or adapter production owners;
- add a second state, lifecycle, transaction, presentation, or plugin owner;
- expose private runtime types through installed headers;
- strengthen the documented C++ or plugin ABI promise;
- add global production test flags; or
- redesign production code for test convenience.

If a deterministic test cannot be written through an existing observable
contract, one narrow injectable callback or wait seam may be proposed in a
separate production commit. That commit must identify the affected invariant,
have no default-runtime behavior change, and pass architecture admission review.

## Layer Model

Every registered test receives exactly one primary layer label.

| Layer | Definition | Permitted dependencies | Default lane |
|---|---|---|---|
| L1 Unit | One function/class or pure data rule; no DSO, filesystem workflow, runtime service, or scheduling dependency | direct owner and lightweight contracts | PR |
| L2 Module Contract | One canonical module owner exercised through its inward/outward contract; fakes allowed at outward ports | owner plus declared direct dependencies; no aggregate façade unless the façade is the owner | PR |
| L3 Integration | Two or more production modules, filesystem transaction, plugin DSO, installed component, GUI bridge, or ROS component integration | explicit integrated owners | PR where deterministic |
| L4 End-to-End | Public/runtime façade workflow from Open through artifacts/Close or external dataset replay | production composition and supported plugins | tiny PR / real dataset external-nightly |
| L5 Fault / Concurrency | Injected failure, cancellation, stale generation, recovery, reentrancy, or concurrent scheduling | same owners as protected contract plus deterministic synchronization | PR + sanitizer selection |
| L6 Soak / Performance | Repeated lifecycle, resource slope, peak memory, latency, large map, real GPU endurance | explicit resource/measurement owner | nightly; implemented by Goals 04/05 |

L6 is reserved and wired into the taxonomy in Goal 02. Goal 02 must not label a
short counter assertion as soak/performance evidence. Actual L6 harnesses and
thresholds belong to Goals 04 and 05.

## Ownership Metadata

CTest metadata is the machine-readable source of test ownership. The physical
tree mirrors production modules, while labels encode layer and cross-cutting
invariants.

Required labels:

```text
layer:L1 .. layer:L6             exactly one
module:<logical-module>          exactly one primary module
owner:<canonical-owner>          exactly one
invariant:<stable-id>            one or more when applicable
lane:pr | lane:nightly | lane:external | lane:gpu
sanitizer:asan-ubsan | sanitizer:tsan   when selected
```

Examples:

```text
layer:L2;module:runtime.state;owner:RuntimeStateStore;
invariant:committed-state-authority;lane:pr;sanitizer:asan-ubsan

layer:L5;module:runtime.control;owner:PipelineController;
invariant:committed-success-wins-late-cancel;lane:pr;sanitizer:tsan
```

CTest names remain stable during the initial move so sanitizer scripts and
external dashboards do not silently lose coverage. A rename is allowed only in
the split/consolidation phase and must be recorded in the migration table.

## Current Authoritative Inventory

### Registered core and ROS tests

The current GUI-on ROS/ament build registers 31 core CTests and one ROS CTest.
The package consumer is intentionally executed separately by CI.

| Current test | Current primary owner/module | Workflow / protected contract | Current layer | Disposition |
|---|---|---|---:|---|
| `open_lmm_safety_regression_tests` | **MIXED / UNASSIGNED** | storage rollback/recovery, config/input validation, pipeline failure, optimizer/remover/data-loader behavior | L1-L3 mixed | split by owner; preserve assertions |
| `open_lmm_algorithm_contract_tests` | domain algorithm base wrappers | Result/exception/cancel/context boundary across algorithms | L2 | keep, move to `domain/contract` |
| `open_lmm_descriptor_engine_tests` | `domain.descriptor` / descriptor engine | descriptor artifact, validation, identity, owner lifetime | L1/L2 | move to descriptor owner |
| `open_lmm_descriptor_database_tests` | `domain.loop_detection.database` | clone/merge/reuse, artifact compatibility, temporal gate | L1/L2 | move to loop-detection owner |
| `open_lmm_alignment_decision_policy_tests` | `domain.alignment` / decision policy | anchor/stored/interactive/automatic precedence | L1 | move unchanged |
| `open_lmm_alignment_contract_fixture_tests` | public/domain alignment contract | proposer and loop-constraint direction/diagnostics | L2 | rename file; it is a suite, not a fixture |
| `open_lmm_alignment_map_resolution_tests` | `domain.loop_detection.alignment` | map resolution, KISS injection, progress, cancellation | L1/L2 | move unchanged |
| `open_lmm_agent_id_tests` | `foundation.contracts` / `AgentId` | public AgentId/data/result value rules | L1 | move to foundation/contracts |
| `open_lmm_pipeline_controller_tests` | `runtime.control` / `PipelineController` | commands, receipts/query reconciliation, recovery, cancellation, events, feedback | L2-L5 mixed | split contract/integration/fault groups |
| `open_lmm_runtime_service_tests` | `runtime.service` / `RuntimeService` | Open/Replace/Close, epoch, jobs, subscriptions, recovery | L3/L5 mixed | split contract and fault/concurrency groups |
| `open_lmm_bounded_executor_tests` | **MIXED** `runtime.resources` + foundation executor | admission/bounds, cancellation, backpressure, heavy-phase RAII | L1/L2/L5 | split resource contract from concurrency cases |
| `open_lmm_runtime_transaction_tests` | `runtime.state` with one storage case | candidate/store/payload, stale base, file/state barrier, recovery | L2/L3/L5 | move storage case out; keep state owner |
| `open_lmm_config_transaction_tests` | `config.application` integration | config candidate with state/file barrier and concurrent commit | L3/L5 | keep integration owner; make overlap deterministic |
| `open_lmm_stage_executor_fixture_tests` | **MIXED** `runtime.execution.data_load/map_update` | candidate publication, resident ownership, admission/draining; no `StageExecutor` instance | L2/L5 | rename/split by executor owner |
| `open_lmm_map_update_executor_tests` | `runtime.execution` / `MapUpdateExecutor` | sequential/parallel admission, failure, cancellation, draining | L2/L5 | split contract and concurrency groups |
| `open_lmm_visualization_projector_tests` | `visualization.projection` / projector | projection, candidate rollback, generation, cache eviction | L2/L5 | move to visualization owner |
| `open_lmm_orchestration_executor_fixture_tests` | **MIXED** alignment/optimize executors | prefix replay, exclusion, artifact store, cancellation | L2/L3 | rename/split; not generic orchestration |
| `open_lmm_save_executor_tests` | `runtime.execution` / `SaveExecutor` | ready-only/fallback save, atomic file commit/recovery | L2/L3 | keep execution owner with storage integration label |
| `open_lmm_runtime_bootstrapper_tests` | `runtime.composition` / bootstrapper | config/plugin preflight, bounded read, manifest/output recovery | L2/L3/L5 | move to composition owner |
| `open_lmm_execution_spec_tests` | **MIXED** `runtime.model` + state artifacts | execution order/dependency/invalidation and artifact repository | L1/L2 | split state assertions from model suite |
| `open_lmm_controller_concurrency_tests` | `runtime.control` / `PipelineController` | late cancel, reentrancy, notification, lock ordering | L5 | retain as canonical control concurrency suite |
| `open_lmm_map_alignment_coordinator_tests` | `domain.loop_detection` / coordinator | interactive feedback/fallback/retry/timeout/stale response | L3/L5 | split integration/fault; remove sleep polling |
| `open_lmm_gui_plugin_tests` | **MIXED** GUI model/presentation/worker/plugin/runtime bridge | capability, event model, ready swap, supersession, RuntimeClient replacement | L1-L5 mixed | split into five owner/layer suites |
| `open_lmm_profiling_macro_tests` | `foundation.profiling` | profiling macro compile/runtime behavior | L1 | move to foundation |
| `open_lmm_logging_tests` | `foundation.logging` | logging façade behavior | L1 | move to foundation |
| `open_lmm_config_schema_tests` | `config.document/schema` | parse/schema/migration/canonicalization/redaction/limits | L1/L2 | move to config document/schema owner |
| `open_lmm_plugin_abi_tests` | **MIXED** plugin host/factory/built-ins | metadata rejection, DSO lifetime, factory transfer, built-in smoke | L2/L3 mixed | split loader, factory, built-in integration |
| `open_lmm_plugin_selection_tests` | `config.domain` / algorithm config | typed plugin selection, capability, numeric/extrinsic decoding | L1/L2 | move to config domain owner |
| `open_lmm_self_contained_e2e_tests` | **MIXED** full runtime workflow | normal pipeline, replay/reconfigure, recovery/presentation/resource faults | L3-L5 mixed | split normal L4 from L3/L5 scenarios |
| `open_lmm_architecture_boundary_tests` | `architecture.policy` | static owner/dependency/public-boundary contract | L2 static | keep as architecture policy helper |
| `open_lmm_release_policy_tests` | `release.policy` | release version/SONAME/workflow contract | L2 static | keep as release policy helper |
| `open_lmm_ros_goal_admission_tests` | `adapters.ros` / `GoalAdmissionGate` | single admission and terminal release under 32 threads | L1/L5 | split pure contract/concurrency labels; add real adapter integration separately |

### Package, fixture, tool, and CI assets

| Asset | Classification | Disposition |
|---|---|---|
| `package_consumer_tests.cmake` and `package_consumer/**` | L2/L3 installed API/package contract | keep; move under `package/`; add header-to-minimum-component manifest |
| `plugin_fixture.cpp` + `plugin_fixture_interface.hpp` | plugin ABI DSO fixture | move under `plugins/fixtures` |
| `gui_plugin_fixture.cpp` | GUI capability DSO fixture | move under `adapters/gui/fixtures` |
| `test_runtime_port.hpp` | reusable runtime command/query fake | move to `support/runtime/recording_runtime_port.hpp` |
| `replay_verify.cpp` | L4 external real-dataset verifier | keep under `tools/replay`; Goal 03 owns registration/data policy |
| `artifact_compare.cpp` | replay artifact tolerance tool | keep under `tools/replay`; Goal 03 owns use |
| `runtime_service_verify.cpp` | unregistered, unreferenced polling verifier | remove after confirming no external caller; registered service/E2E coverage supersedes it |
| `scripts/ci/build_and_test.sh` | clean build, core/ROS/package aggregator | keep; select/report new labels |
| `scripts/ci/build_sanitizer_tests.sh` | selected sanitizer suite | convert test selection from name regex to labels; preserve explicit build targets |
| `scripts/ci/check_architecture_policy.sh` | source policy entry point | keep |
| `scripts/ci/tsan.supp` | TSan environment policy | keep with rationale |
| `.github/workflows/compiler-matrix.yml` | required compiler/sanitizer/policy topology | keep stable job names |

Exact generated fixture modes are also part of the inventory:

- GUI DSO fixtures: `valid`, `old_capability`, `empty_capability`,
  `null_capability`;
- algorithm plugin fixtures: `valid`, `wrong_abi`, `null_factory`,
  `missing_destroy`, `no_entry`;
- built-in plugin smoke arguments when enabled: ScanContext, SOLiD, HMM-MOS,
  Dufomap, OTD, FreeDOM, and ERASOR; and
- package consumers: full, contracts-only, client-only, PluginSDK-only, header
  self-containment, relocation, upgrade ownership, and manifest/SONAME checks.

## Current Defects the Reorganization Must Fix

1. `open_lmm/test/CMakeLists.txt` is a single flat registration file with no
   module/layer ownership and only one `package` label.
2. Most runtime/config/plugin tests link `open_lmm_map_server`, which can mask a
   missing direct dependency behind the aggregate façade.
3. There is no private C5-C7 logical-module compile fixture. Current policy
   checks source strings and duplicate translation-unit ownership, but does not
   prove module-minimal compilation/linking.
4. Large mixed-owner suites obscure which invariant fails. The largest examples
   are `safety_regression`, `pipeline_controller`, `runtime_service`,
   `gui_plugin`, and `self_contained_e2e`.
5. Static policy checks sometimes protect vocabulary rather than behavior.
   Structural dependency rules belong there; commit/cancel/recovery/presentation
   behavior does not.
6. At least 11 fixed sleeps exist in registered tests, plus multiple unbounded
   `yield()` polling loops. Several sleeps are used to prove that another thread
   has not returned, which is scheduler-dependent.
7. Batch has only static dependency and installed `--help` smoke coverage. ROS
   has only the admission primitive. Real Iridescence/GPU execution is absent.
8. `runtime_service_verify` has no caller or CTest registration.
9. Sanitizer membership is encoded by target-name regex rather than test
   ownership metadata and omits relevant alignment/config/visualization suites.
10. No real L6 test exists; short cache/reservation assertions must not be
    represented as soak/performance evidence.

## Target Test Tree

The physical tree is module-owner-first. Layer is metadata and, where a module
contains multiple layers, a final directory component.

```text
open_lmm/test/
├── CMakeLists.txt                         # thin dispatcher only
├── cmake/
│   ├── OpenLmmTest.cmake                  # registration + metadata checks
│   └── TestManifest.cmake                 # complete owner/layer registry
├── support/
│   ├── check.hpp
│   ├── temp_workspace.hpp
│   ├── synchronization.hpp
│   ├── runtime/
│   │   ├── recording_runtime_port.hpp
│   │   └── runtime_state_fixture.hpp
│   └── plugins/plugin_fixture_interface.hpp
├── foundation/
│   ├── contracts/unit/
│   ├── logging/unit/
│   └── profiling/unit/
├── config/
│   ├── document/unit/
│   ├── schema/contract/
│   ├── domain/contract/
│   └── application/{integration,fault_concurrency}/
├── domain/
│   ├── support/{unit,contract}/
│   ├── data_loader/{unit,contract}/
│   ├── descriptor/{unit,contract}/
│   ├── alignment/{unit,contract}/
│   ├── loop_detection/{unit,contract,integration,fault_concurrency}/
│   ├── optimization/{unit,contract}/
│   └── dynamic_removal/{unit,contract}/
├── runtime/
│   ├── model/{unit,compile_contract}/
│   ├── state/{contract,fault_concurrency,compile_contract}/
│   ├── resources/{contract,fault_concurrency,compile_contract}/
│   ├── execution/{contract,integration,fault_concurrency,compile_contract}/
│   ├── control/{contract,integration,fault_concurrency,compile_contract}/
│   ├── service/{contract,integration,fault_concurrency,compile_contract}/
│   ├── composition/{contract,integration,compile_contract}/
│   └── client/{contract,integration,compile_contract}/
├── storage/{contract,fault_concurrency,compile_contract}/
├── visualization/{contract,fault_concurrency,compile_contract}/
├── plugins/
│   ├── host/{contract,integration,compile_contract}/
│   └── fixtures/
├── adapters/
│   ├── batch/{contract,integration}/
│   └── gui/{model,presentation,plugin_contract,runtime_bridge,snapshot_worker,fixtures}/
├── workflows/
│   ├── integration/
│   ├── e2e/
│   └── fault_concurrency/
├── architecture/{policy,module_compile}/
├── release/policy/
├── package/{orchestrator,consumers,manifests}/
└── tools/replay/

ros/test/
├── support/
├── contract/goal_admission_tests.cpp
├── integration/runtime_adapter_graph_tests.cpp
└── fault_concurrency/runtime_adapter_cancel_tests.cpp
```

No empty L6 directory is created merely for symmetry. Goal 04/05 will add it
when a real harness exists.

## Test Registration Contract

Add a test-only CMake helper with a required schema:

```cmake
openlmm_add_test(
  NAME open_lmm_runtime_transaction_tests
  LAYER L2
  MODULE runtime.state
  OWNER RuntimeStateStore
  SOURCES runtime/state/contract/runtime_transaction_tests.cpp
  LINK_TARGETS open_lmm_runtime_state_test_support
  INVARIANTS committed-state-authority candidate-commit-rollback
  LANES pr
  SANITIZERS asan-ubsan)
```

The helper must:

- reject a missing/unknown layer, module, or owner;
- set CTest labels, timeout, working directory, and build-tree library path;
- register the target in a generated manifest;
- reject production `.cpp` files as test sources;
- preserve the current target/CTest name unless an explicit migration entry
  exists; and
- allow environment-dependent `external` and `gpu` tests to be built without
  running in the default PR lane.

Add a policy CTest that reads the generated manifest and fails if:

- a registered test has zero or multiple primary layers/owners;
- a test target is absent from the manifest;
- a sanitizer-selected test lacks the corresponding built target;
- a CTest name is registered twice; or
- an L1/L2 module test links `open_lmm_map_server` without the aggregate façade
  being its declared owner.

## Module-to-Minimum-Contract Matrix

| Module | Canonical owner | Required minimum suite after Goal 02 |
|---|---|---|
| runtime/model | execution spec owner | node/stage order, dependency/invalidation, model-only compile contract |
| runtime/state | `RuntimeStateStore`, `RuntimeTransaction`, payload/artifact owners | immutable candidate, stale base, revision, authority+recovery latch, payload lifetime, state-only compile contract |
| runtime/resources | `ResourceGovernor`, bounded executor | queue/task/memory bound, reservation RAII, cancellation/failure release, resource-only compile contract |
| runtime/execution | `StageExecutor`, `StageCoordinator`, individual stage executors | command→candidate, stage ordering, file/state barrier, both MapUpdate modes, save/reconfigure recovery, execution compile contract |
| runtime/control | `PipelineController` | receipt/query reconciliation, late cancellation, serialization, event journal, feedback, callback reentrancy, control compile contract |
| runtime/service | `RuntimeService` | Open/Replace/Close, epoch/job/event mapping, leases, subscriptions, bounded retention, recovery gating, service compile contract |
| runtime/composition | `MapServer`, `RuntimeBootstrapper` | production-port delegation, bootstrap/preflight/output ownership, composition compile contract |
| runtime/client | `RuntimeClient` PImpl | forwarding and move/lifetime behavior through public API plus source-free compile/link contract |
| storage | `PendingOutputSet`, `FileSetTransaction`, `OutputRepository` | preflight, atomic install, rollback, committed cleanup recovery, alias protection, storage compile contract |
| visualization | projector/worker/repository/presentation owners | last-valid retention, candidate rollback, stale generation, cache eviction/metrics, visualization compile contract |
| plugin host | `AlgorithmFactory` and ABI loader | metadata/error normalization, destroy-before-dlclose, factory owner transfer, host-only compile contract |
| domain algorithms | each domain library | base wrapper contract plus owner-specific behavior, cancellation and result/error context |
| config | document/schema/domain/application owners | parse/canonicalize/migrate, typed decode, candidate validation, transactional apply, per-layer compile contracts |
| GUI adapter | model/presentation/worker/plugin-host/runtime-bridge owners | event projection, ready-then-swap, stale rejection, deterministic worker cancellation, capability, RuntimeClient bridge |
| Batch adapter | `open_lmm_batch` | usage/invalid input exit codes and installed tiny Open→RunAll→Wait subprocess |
| ROS adapter | runtime adapter and goal gate | admission primitive plus real component/action/status/events/cancel integration |
| package/API | installed targets and manifests | minimal-component header compile, source-free consumers, relocation, SONAME/license/upgrade ownership |
| architecture | architecture/release policy owners | forbidden edges, canonical owners, manifest completeness, logical-module compile contracts |

## Invariant-to-Test Matrix

Stable invariant IDs are introduced for metadata. Static policy is supporting
evidence unless the invariant is itself structural.

| ID | Frozen invariant | Primary executable owner after Goal 02 | Static/supporting gate |
|---|---|---|---|
| INV-01 | one committed runtime-state owner | runtime/state contract | canonical owner architecture gate |
| INV-02 | pending work does not mutate committed state | runtime/state + execution contracts | none |
| INV-03 | candidate → validate → commit/rollback | state, storage, StageCoordinator contracts | owner-direction gate |
| INV-04 | pre-commit failure preserves last state/files | storage/execution/config fault suites | none |
| INV-05 | committed success wins late cancellation | control L5 suite | none |
| INV-06 | mutating failure reconciles authoritative query revision | control L2/L5 suite | none |
| INV-07 | committed recovery-required health is authoritative | state/execution/service L3/L5 suites | none |
| INV-08 | retired runtime epoch cannot publish jobs/events/callbacks | service and GUI worker L5 suites | epoch field ownership gate |
| INV-09 | last valid presentation remains until ready replacement | visualization and GUI presentation contracts | eager-clear forbidden pattern only as support |
| INV-10 | stale presentation generation cannot replace newer state | visualization/GUI worker L5 suite | generation owner gate |
| INV-11 | runtime model → state → execution dependency direction | module compile contracts | forbidden include/link gate |
| INV-12 | domain does not depend on runtime/plugin host | domain and plugin compile contracts | whole-tree forbidden dependency gate |
| INV-13 | runtime service does not depend on adapters; adapters are leaves | Batch/GUI/ROS compile/integration contracts | adapter forbidden dependency gate |
| INV-14 | public/private API boundary and RuntimeClient PImpl | package/client compile contracts | public allowlist/light-header gate |
| INV-15 | plugin DSO instance dies before `dlclose` | isolated plugin-host ABI contract | ABI metadata gate |
| INV-16 | long-lived resources are bounded/admitted | resources/execution/visualization contracts | owner/policy gate |
| INV-17 | shutdown and callback lifetime are deterministic | control/service/GUI L5 suites | none |
| INV-18 | diagnostics do not become behavioral authority | control/service snapshot/event tests | retention-policy gate |

## C5-C7 Logical-Module Compile Fixtures

Goal 02 must add executable compilation evidence in addition to source scans.

### A. Boundary consumer translation units

Create one compile-only consumer per logical owner. Each consumer includes the
owner's representative boundary headers and receives only the declared direct
dependency include/link set. It must not link the aggregate
`open_lmm_map_server` except for composition/E2E consumers.

Required consumers:

```text
runtime_model_contract_compile
runtime_state_contract_compile
runtime_resources_contract_compile
storage_contract_compile
runtime_execution_contract_compile
runtime_control_contract_compile
runtime_service_contract_compile
runtime_composition_contract_compile
runtime_client_contract_compile
config_document_contract_compile
config_schema_contract_compile
config_domain_contract_compile
config_application_contract_compile
domain_no_runtime_contract_compile
plugin_host_direction_contract_compile
visualization_contract_compile
adapter_public_leaf_contract_compile
```

### B. Canonical production-owner build proof

The clean CI build must explicitly request every canonical object/library target
for model, state, resources, storage, execution, control, service, composition,
client, config layers, visualization, plugin host, and domain libraries. The
generated compile database policy must require every production translation unit
to appear exactly once under its declared owner; zero occurrences is a failure,
not an allowed case.

### C. C5-C7 negative boundaries

- C5: model consumer cannot see state/execution; state consumer cannot see
  execution; no legacy `stage_runner` compatibility header.
- C6: service consumer cannot see adapters; adapter leaf consumer uses installed
  `RuntimeClient`; only composition owns `MapServer`.
- C7: domain consumers receive no plugin-host include/link path; plugin-host
  consumer may depend toward typed domain interfaces; direct `dlopen`/`dlsym`
  remains host-owned.

Compilation fixtures must never compile production `.cpp` files inside a test
target. Canonical production targets remain the only translation-unit owners.

## Static Policy vs Behavioral Evidence

Static tests remain authoritative for:

- forbidden include/link direction;
- canonical CMake target/source ownership;
- public header/export allowlists;
- lightweight/PImpl header dependency limits;
- release version, SONAME, workflow job names, and pinned action SHAs; and
- absence of removed legacy paths.

Executable tests are authoritative for:

- state/file commit and rollback;
- cancellation/receipt/query reconciliation;
- recovery-required state and future mutation gating;
- sequential/parallel resource admission;
- runtime replacement/epoch isolation;
- plugin create/destroy/lifetime behavior;
- presentation retention and stale-generation rejection;
- Batch/ROS adapter workflows; and
- installed consumer execution.

A source-string requirement such as `AcquireHeavyMemoryPhase`, `RuntimeClient`,
or absence of `ClearVisualizationLayers` must never be cited as sole proof of the
corresponding runtime behavior.

## Deterministic Synchronization Policy

Add `test/support/synchronization.hpp` with test-only primitives built from
`std::latch`, `std::barrier`, `std::condition_variable`, promise/future, and a
deadline used only as a failure watchdog.

Rules:

1. no fixed sleep may establish that another operation is blocked or has made
   progress;
2. no unbounded spin/yield loop is permitted;
3. a negative assertion requires an `entered` acknowledgement before checking
   that completion has not occurred;
4. release is controlled by a separate latch/barrier;
5. real timeout semantics may use elapsed time, but scheduler ordering must be
   controlled independently; and
6. external process/replay polling may retain deadline polling because it is an
   L4 runner, not a race oracle.

Mandatory conversions:

- `map_update_executor_tests`: heavy-phase overlap and sibling-drain sleeps;
- `runtime_service_tests`: subscription reset, Open/Close transition, and
  unpublished-output Close waits;
- `gui_plugin_tests`: completed worker publication and queued-event polling;
- `map_alignment_coordinator_tests`: 1 ms snapshot/attempt polling and proposer
  spin;
- `pipeline_controller_tests`: unbounded feedback/event spin loops; and
- `config_transaction_tests`: start concurrent candidates behind a barrier so
  the conflict is a deterministic commit-barrier race.

## Split and Consolidation Rules

### Mixed suites

- Split `safety_regression_tests.cpp` into storage file-set, domain support,
  data-loader, optimizer, remover, config-input, and pipeline integration suites.
- Split `gui_plugin_tests.cpp` into GUI plugin capability, model/config editor,
  presentation/repository, snapshot-worker concurrency, and RuntimeClient bridge.
- Split `self_contained_e2e_tests.cpp` into normal full-pipeline L4,
  reconfiguration/replay L3/L4, and recovery/presentation/resource L5 suites.
- Split `plugin_abi_tests.cpp` into loader ABI/lifetime L2, factory transfer L2,
  and built-in plugin integration L3.
- Split controller/service files by layer only when the owner stays the same;
  do not create artificial production abstractions to match test files.

### Misnamed/misowned suites

- Rename `stage_executor_fixture_tests.cpp` to DataLoad/MapUpdate executor
  contracts; it does not instantiate `StageExecutor`.
- Rename `orchestration_executor_fixture_tests.cpp` to alignment/optimize
  executor contracts.
- Move `RuntimeTransaction::TestOutputRepositoryRollbackAndCommit` to storage.
- Move ArtifactRepository assertions out of `execution_spec_tests` to state.
- Remove the unused `RuntimeClient` include from `runtime_service_tests`; client
  behavior belongs to client/package/GUI bridge suites.

### Duplicates

Consolidate only after assertion-by-assertion comparison:

- controller snapshot-outside-lock cases duplicated between
  `pipeline_controller_tests` and `controller_concurrency_tests`; and
- feedback/snapshot lock-inversion cases duplicated in the same two suites.

The L5 concurrency version remains canonical. Removal requires proof that error
context, ordering, and callback assertions are all preserved.

### Intentional duplication

Keep production-independent golden manifests:

- public header allowlist;
- exported target allowlist; and
- release/package version expectations.

Generating these from production would eliminate their regression value.

## New Behavioral Contracts Required

Goal 02 adds focused tests only for frozen behavior already promised by current
architecture:

1. isolated `RuntimeStateStore` authority/recovery contract;
2. direct StageCoordinator file/state/recovery barrier contract;
3. direct StageExecutor command/query façade contract;
4. RuntimeClient forwarding/move/Close contract through public types;
5. isolated plugin-host loader/factory error and lifetime contracts;
6. header-to-minimum-installed-component compile matrix;
7. Batch installed subprocess success/failure contract using a tiny fixture;
8. one deterministic ROS component/action/status/events/cancel integration
   contract, with Goal 13 extending the matrix later;
9. GUI worker publication and epoch cancellation through latches; and
10. visualization cache eviction/accounting contract without claiming hard RSS.

Missing product capabilities, real dataset baselines, real GPU execution,
portable plugin ABI, crash durability, and hard-memory guarantees are not
silently added by Goal 02. They remain owned by Goals 03-05, 13, and 15.

## Plugin and Package Contract Details

Split plugin negative fixtures so loader behavior is independently observable.
Add cases for:

- null/invalid plugin kind/name metadata;
- exact capability mismatch and null/empty capability semantics;
- create callback throw and successful-null return;
- entry callback failure where representable without undefined ABI behavior; and
- factory normalization for null/throw/unknown algorithm types.

Do not test a throwing `destroy` across the C ABI if the public contract requires
`noexcept`; that is plugin undefined behavior, not a host compatibility promise.

For package headers, replace the current broad link set with a checked manifest:

```text
header -> minimum component(s) -> language -> run/compile-only
```

`contracts` headers compile with contracts only; `RuntimeClient` headers with
client; PluginSDK C header with PluginSDK; GUI public headers with gui. A header
may receive a broader component only when the manifest records the reason.

## Adapter Test Tiers

### Headless PR tier

- GUI model, queue, repository, presentation, worker, plugin capability;
- Batch usage and tiny installed subprocess;
- ROS admission plus one real rclcpp graph integration;
- no real window/GPU requirement.

### GPU/self-hosted tier

Define but do not fake:

- actual Iridescence plugin load;
- drawable upload/remove;
- ready-then-swap old-drawable retention;
- stale upload rejection;
- multi-agent visibility; and
- shutdown/resource release.

This tier uses label `lane:gpu` and runs only when the driver/environment
capability check succeeds. Goal 13 owns making it a required environment.

### External dataset tier

`replay_verify` and `artifact_compare` remain `lane:external`; Goal 03 defines
dataset manifests and CI/nightly policy. They are not silently counted as green
when data is absent.

## Implementation Phases

### Phase 0 — Baseline and manifest

1. Capture current test list, target list, runtime, and sanitizer membership.
2. Add `OpenLmmTest.cmake` and `TestManifest.cmake` without moving files.
3. Register all existing CTests through the helper with owner/layer/invariant
   metadata.
4. Preserve current test names and commands.
5. Validate that the 31 core + 1 ROS suite remains behaviorally identical.

Checkpoint: full CTest and sanitizer selection count match the pre-change
baseline.

### Phase 1 — Test support and deterministic synchronization

1. Add focused check/temp/synchronization/runtime-port support headers.
2. Convert every fixed-sleep race oracle and unbounded wait listed above.
3. Add per-test CTest timeout as a final watchdog.
4. Do not centralize domain-specific fakes merely to reduce LOC.

Checkpoint: targeted L5 suite repeated at least 100 times; selected TSan PASS.

### Phase 2 — Physical owner-aligned moves

1. Create the target tree and thin per-module CMake files.
2. Use `git mv` so history remains reviewable.
3. Move cohesive files first without changing assertions or target names.
4. Move fixtures/support/tools/package assets to explicit owners.
5. Remove no test in this phase.

Checkpoint: generated manifest and CTest list are unchanged except paths.

### Phase 3 — Mixed-suite split and duplicate cleanup

1. Split safety, GUI, E2E, plugin ABI, controller, and service suites by the
   rules above.
2. Record old target → new target mapping in the manifest.
3. Compare assertion inventory before deleting duplicate cases.
4. Remove `runtime_service_verify` only after repository and release-tooling
   caller search remains empty.

Checkpoint: invariant matrix has no ownerless row and no removed assertion gap.

### Phase 4 — Compile and missing module contracts

1. Add all C5-C7 compile consumers and canonical target build proof.
2. Tighten compile database ownership from `0 or 1` to exactly `1` for every
   production translation unit.
3. Add focused state, coordinator, executor, client, plugin-host, package,
   Batch, ROS, GUI worker, and visualization contracts.
4. Keep structural policy and behavioral evidence separate.

Checkpoint: deliberately injected forbidden edge fails the relevant fixture in
a temporary negative-test copy; production tree remains unchanged.

### Phase 5 — CI and sanitizer lanes

1. Select tests by labels rather than broad name regex.
2. Keep the seven required workflow check names stable.
3. Expand ASan/UBSan to owner-focused domain/config/visualization contracts that
   allocate/parse/load.
4. Expand TSan to alignment feedback, config commit, executor drain, GUI worker,
   controller, and service L5 suites.
5. Preserve package consumer as a separate source-free installed-prefix gate.
6. Publish manifest, CTest JUnit, and per-layer counts as CI artifacts.

Checkpoint: required compiler matrix, policy, package, ROS, ASan/UBSan, and TSan
all PASS.

### Phase 6 — Final audit

1. Re-run module→test and invariant→test completeness checks.
2. Verify no test target compiles a production `.cpp`.
3. Verify no L1/L2 target overlinks the aggregate façade without declaration.
4. Verify no timing-based race oracle remains.
5. Record actual result, deviations, and validation evidence in this document,
   changing status from specification to implementation result only then.

## Validation Commands

Use unique clean directories; do not reuse the Goal 01 build.

```bash
scripts/ci/check_architecture_policy.sh

scripts/ci/build_and_test.sh \
  goal02-gcc12-gui-on /usr/bin/gcc-12 /usr/bin/g++-12 ON

scripts/ci/build_sanitizer_tests.sh \
  goal02-clang15-asan-ubsan ASAN_UBSAN \
  /usr/bin/clang-15 /usr/bin/clang++-15

scripts/ci/build_sanitizer_tests.sh \
  goal02-gcc12-tsan TSAN /usr/bin/gcc-12 /usr/bin/g++-12

ctest --test-dir <build>/open_lmm -N
ctest --test-dir <build>/open_lmm --output-on-failure -L 'layer:L1'
ctest --test-dir <build>/open_lmm --output-on-failure -L 'layer:L2'
ctest --test-dir <build>/open_lmm --output-on-failure -L 'layer:L3'
ctest --test-dir <build>/open_lmm --output-on-failure -L 'layer:L4'
ctest --test-dir <build>/open_lmm --output-on-failure -L 'layer:L5'

git diff --check
```

Required evidence:

- fresh GCC GUI-on build;
- full core CTest;
- full ROS CTest including real adapter integration;
- source-free package consumer and minimal header/component matrix;
- module compile contract suite;
- architecture/release policy;
- selected ASan+UBSan and TSan suites;
- 100x deterministic repetition of newly converted concurrency cases; and
- explicit `NOT_AVAILABLE` rather than inferred PASS for real dataset/GPU/L6
  lanes not present in the environment.

## Implemented State and Evidence

The Goal 02 implementation was completed on 2026-08-20 UTC. The test tree is
module-owner-first, while layer remains explicit CTest metadata. The top-level
`test/CMakeLists.txt` is now a thin dispatcher over owner-area registration
files. The generated manifest contains 59 core tests and rejects raw
`add_test`, duplicate names/targets, missing owner metadata, production source
compilation by test targets, and undeclared L1/L2 aggregate-façade links.

Implemented coverage includes:

- 17 isolated C5-C7 logical-module compile consumers and an exact-one
  production translation-unit compile-owner gate;
- focused RuntimeStateStore, OutputRepository, StageCoordinator,
  StageExecutor, RuntimeClient, plugin-loader, AlgorithmFactory, GUI
  capability, worker, presentation, and resource contracts;
- owner/layer splits for the former safety, controller, service, executor,
  visualization, GUI, plugin ABI, and self-contained workflow suites, with the
  complete old-to-new disposition in `test_migration_manifest.tsv`;
- 12 negative plugin DSO fixture modes, built-in plugin integration, and a
  26-header minimum installed-component manifest;
- installed Batch help, valid tiny workflow, and invalid-input subprocesses;
- a real in-process ROS 2 component/action/status/events/cancel integration;
- label-selected sanitizer suites and CI output for the generated manifest,
  JUnit, layer counts, and unavailable environment lanes; and
- deterministic latch/barrier/condition-variable test support. Registered
  tests contain no fixed sleep or unbounded spin/yield race oracle.

The only production changes used by this work are isolated in `f4154eb`: an
optional bounded-executor wait notification, an optional visualization-worker
completion notification, and build-tree-only runtime-client link resolution.
Both callbacks default to empty, are private implementation seams, and do not
alter default runtime behavior, public API/ABI, plugin ABI, or ownership.

Validation evidence:

| Evidence | Result |
|---|---|
| fresh GCC 12 GUI-on build, core, ROS, install/package/Batch | PASS |
| core CTest after final registration split | 59/59 PASS |
| ROS CTest with real adapter graph | 2/2 PASS |
| ASan+UBSan, fresh Clang 15 build | 52/52 PASS |
| TSan, fresh GCC 12 build | 17/17 PASS |
| converted concurrency cases repeated 100 times | 800/800 PASS |
| isolated compile-contract suite | 17/17 owners built; policy PASS |
| temporary C5 negative edge (`runtime/model` -> `runtime/state`) | compile failed as required; restored fixture rebuilt PASS |
| architecture/release/manifest/module policy CTests | 4/4 PASS |
| `scripts/ci/check_architecture_policy.sh` | PASS |
| `git diff --check` | PASS |
| real dataset replay | NOT_AVAILABLE (Goal 03) |
| real GPU/Iridescence lane | NOT_AVAILABLE (Goal 13) |
| L6 soak/performance | NOT_AVAILABLE (Goals 04/05) |

The final core layer distribution is L1=4, L2=22, L3=21, L4=1, L5=11,
L6=0. A zero L6 count is intentional and is reported as `NOT_AVAILABLE`, not
as passing evidence.

## Completion Conditions

Goal 02 is complete only when all statements below are proven by current-state
evidence:

- every test, fixture, tool, manifest, and CI script in scope has an explicit
  disposition and owner;
- every registered test has exactly one module owner and one L1-L6 layer;
- the physical tree reflects final production module ownership;
- runtime state/execution/control/service, storage, visualization, plugins,
  adapters, package, and architecture each have the minimum contract suite
  specified above;
- C5-C7 logical-module compile fixtures build with declared direct dependencies;
- production translation units have exactly one canonical compile owner;
- critical semantic branches are protected by executable tests, not only source
  strings;
- duplicate/obsolete fixtures are removed only after assertion-equivalent
  coverage is demonstrated;
- no fixed-sleep or unbounded-spin race oracle remains;
- core, ROS, package, policy, ASan+UBSan, and TSan required suites pass; and
- no production behavior, public API/ABI, plugin ABI, or architecture invariant
  changed merely to reorganize tests.

The current-state evidence above satisfies these conditions. Future changes
must continue to pass the manifest, compile-owner, forbidden-edge, behavioral,
package, ROS, and sanitizer gates; unavailable external/GPU/L6 lanes must remain
explicit rather than being inferred green.
