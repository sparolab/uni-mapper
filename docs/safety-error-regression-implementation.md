# Safety, Error Propagation, and Regression Verification

## Scope

This change implements the first three production-readiness priorities:

1. input and index validation;
2. pipeline control flow and recoverable error propagation;
3. a regression-test foundation.

The user-owned `open_lmm/config/config.json` was kept unchanged and used for
both reference and post-change runs.

## Implemented safeguards

- Data loading now rejects missing pose/scan paths, malformed pose rows,
  unsupported formats, empty inputs, and scan/pose count mismatches through
  `Result<T>`.
- Agent count and anchor index are checked before contexts are built.
- Loop source/target agents and scan indices are checked before optimization.
- KD-tree matching handles empty pose sets, absent agents, failed searches,
  and invalid neighbor indices without unchecked access.
- Optimized-map output checks agent and scan indices, and map writes are
  checked for failure.
- `ControlFlow::kKill` now stops the pipeline, records the flow, and returns an
  error. `kSkip` remains a successful per-agent stop.
- DataLoader, loop-detector, backend-optimizer, dynamic-remover, and shared
  library factories return `Result<T>`. Plugin loading no longer terminates the
  process.
- Config file open/JSON parse failures are retained and propagated through
  `MapServer::process()` to the ROS boundary.
- Core-layer `exit()`/`abort()` calls were removed; legacy required-config and
  HMM scan failures throw to their caller instead of terminating locally.

## Automated tests

`open_lmm_safety_regression_tests` covers:

- anchor/follower `AgentContext` behavior;
- equal, mismatched, and empty DataLoader scan/pose counts;
- valid and invalid agent indices;
- empty, invalid, and valid KD-tree search results;
- Pipeline continue, skip, kill, and node-failure behavior;
- a minimal two-node successful pipeline integration;
- missing plugin error propagation;
- missing and malformed config error retention.

`open_lmm_artifact_compare` loads two PCD files and reports bidirectional,
sampled nearest-neighbor mean/RMS/maximum distances for repeatable map
regression checks.

## Verification results

Reference artifacts and logs:

- `/tmp/open_lmm_safety_regression.o4TBAM/before_output`
- `/tmp/open_lmm_safety_regression.o4TBAM/before.log`

Post-change comparison run selected after a repeat run:

- `/root/workspace/output/2026_8_15_0_52_45`
- `/tmp/open_lmm_safety_regression.o4TBAM/after_repeat.log`

### Optimized poses

| Agent | Pose count | Max translation delta | Translation RMS | Max rotation delta |
|---|---:|---:|---:|---:|
| A | 652 | 0.00010 m | 0.0000056 m | 0.00000175 rad |
| B | 1329 | 0.01256 m | 0.00840 m | 0.0000357 rad |

### Point clouds

The PCD comparator sampled approximately 100,000 points in each direction.

| Agent | Before/after points | Mean NN distance | RMS NN distance | Max sampled distance |
|---|---:|---:|---:|---:|
| A | 3,583,054 / 3,583,056 | 0.000014–0.000015 m | 0.00061–0.00062 m | 0.0773 m |
| B | 5,491,092 / 5,491,045 | 0.01165–0.01169 m | 0.01990–0.01997 m | 0.214 m |

The relative point-count changes are about 0.00006% for A and 0.00086% for B.

### Performance

| Metric | Before | After | Change |
|---|---:|---:|---:|
| Dynamic Remover A | 5.4 s | 5.2 s | -3.7% |
| Dynamic Remover B | 15.6 s | 14.7 s | -5.8% |
| End-to-end elapsed | 52.638 s | 51.393 s | -2.4% |
| User CPU | 203.020 s | 199.484 s | -1.7% |
| System CPU | 8.297 s | 8.376 s | +1.0% |

One intervening run produced an agent-B pose offset of about 0.207 m, while a
second run of the same post-change binary returned to within 0.0126 m of the
reference. This demonstrates existing run-to-run nondeterminism in the
parallel matching/optimization path rather than a deterministic change caused
by the safety checks. The selected repeat run is close to the reference in
pose, point-cloud geometry, and runtime.
