# Goal 07 — Python Evaluation / Experiment Toolkit 구현 명세 및 결과

- 문서 상태: **로컬 구현 완료 / 검증 완료**
- 작성일: 2026-08-26 UTC
- 구현·검증일: 2026-08-26 UTC
- 기준 브랜치: `develop`
- 구현 기준 HEAD: `bafed911604d65cb76eac1692ce0009a4e6dd117`
- 결과 commit: 이 문서와 Goal 07 구현을 함께 담은 Git commit(Git history가 canonical SHA)
- architecture freeze: `59e003ebc4b7d44597ced4ddab3436adec310370`
- 상위 요구사항:
  `docs/pose_freeze_goals/open_lmm_post_freeze_goals/07_python_evaluation_experiment_toolkit_goal.md`
- 선행 결과:
  - `docs/post_freeze_results/03_real_dataset_e2e_replay.md`
  - `docs/post_freeze_results/05_benchmark_resource_observability.md`
  - `docs/post_freeze_results/06_python_binding_v1.md`

## 1. 결론

Goal 07은 현재 HEAD에서 구현을 시작할 수 있다. 구현은 새 native binding이나 runtime
owner가 아니라 Goal 06의 `open_lmm.Runtime`을 사용하는 **pure-Python leaf
orchestration layer**로 만든다.

핵심 범위는 다음과 같다.

- locked dataset/config를 사용하는 fresh-process experiment runner
- immutable plan/trial/result DTO와 canonical JSON manifest
- repeated trials, deterministic seed tracking, Cartesian parameter sweep
- stage callback latency, artifact, alignment, trajectory, map summary metric
- JSON/CSV/record export와 optional pandas 변환 경계
- Goal 03 replay runner/comparator를 재사용하는 subprocess adapter
- Goal 05 benchmark workflow/bundle을 재사용하는 subprocess/report adapter
- algorithm variant를 config patch와 expected plugin identity로 표현하는 adapter

Goal 07이 생성하는 metric은 QA/연구용 derived observation이다. runtime state의 authority,
Goal 03 replay 판정, Goal 05 performance baseline을 대체하지 않는다. production algorithm
hot path에 Python callback을 넣지 않으며 private runtime/domain/plugin-host type을 노출하지
않는다.

상위 Goal task와 이 명세의 대응은 다음과 같다.

| Goal 07 task | 구현 명세 |
|---|---|
| dataset runner | §§8–10 |
| experiment manifest | §7 |
| latency/resource/alignment/trajectory/map metric | §11 |
| repeated trial/seed | §9 |
| parameter sweep | §9 |
| JSON/CSV/pandas-friendly export | §12 |
| Goal 03 replay integration | §13 |
| algorithm benchmark adapter | §14 |
| reproducible report metadata | §§7, 12, 19 |

## 2. 현재 기준점과 착수 증거

### 2.1 현재 HEAD 검증

2026-08-26에 exact HEAD `bafed911604d65cb76eac1692ce0009a4e6dd117`의 clean
worktree에서 다음을 확인했다.

| 항목 | 결과 |
|---|---|
| full CTest | **88/88 PASS** |
| architecture/release policy | PASS |
| `git diff --check` | PASS |
| Python Binding v1 | local implementation complete |

Goal 06 완료 증거에는 GCC 12 Python-ON 88/88, Python-OFF 81/81, Clang 15 Python
7/7, ASan+UBSan Python 7/7, source-free wheel E2E 3/3이 기록돼 있다.

### 2.2 재사용할 canonical owner

| Owner/asset | 현재 제공 기능 | Goal 07 사용 방식 |
|---|---|---|
| `open_lmm.Runtime` | Open, RunAll/RunStage, Wait, Snapshot, Visualization, event, Close | 일반 experiment의 유일한 runtime entry |
| immutable Python DTO | event/snapshot/artifact/config/visualization value | post-run metric 입력 |
| Goal 03 replay contract | locked case, report, baseline, runner, comparator | replay adapter가 CLI와 report를 그대로 소비 |
| Goal 05 benchmark contract | P01–P10 runner, raw/bundle/pair schema, approved catalog | benchmark adapter가 workflow를 실행하거나 report를 import |
| `performance_baseline.json` | reviewed compiler/machine/scenario key와 threshold | Python이 재계산하지 않고 comparison을 보존 |
| Python wheel | CPython 3.10 local package와 NumPy | experiment package 배포 기반 |
| `openlmm_add_test` | owner/layer/invariant metadata | 새 Python test 등록 |

Goal 03의 owner-controlled internal replay와 Goal 05의 headless immutable-image
calibration은 완료됐다. 과거 `03_05_hosted_completion_plan.md`의 일부 문구는 당시의
운영 계획 기록이며, 현재 구현 상태 판단은 각 Goal의 최신 결과 문서와 production code를
우선한다.

### 2.3 현재 공백

- `open_lmm.experiments` package와 public API가 없다.
- dataset/config/software identity를 하나로 묶는 experiment manifest가 없다.
- repeated trial과 parameter grid를 deterministic하게 materialize하는 owner가 없다.
- Python runtime event/snapshot/visualization을 stable metric record로 변환하는 계약이 없다.
- JSON/CSV/pandas-friendly result export와 no-overwrite evidence layout이 없다.
- Goal 03/05 tool은 test build asset이며 Python wheel에 설치되지 않는다.
- Goal 06 event에는 native timestamp가 없다. Python callback 도착 시각은 측정할 수 있지만
  Goal 05의 canonical stage latency와 동일하다고 주장할 수 없다.
- source-free wheel은 Git commit을 스스로 증명할 metadata가 없다. strict experiment는
  explicit source commit 또는 wheel artifact digest가 필요하다.

이 공백은 existing extension point로 해결할 수 있으며 architecture change proposal은
필요하지 않다.

## 3. Architecture와 ownership

### 3.1 의존 방향

```text
open_lmm.experiments public API / CLI
              │
              ├── generic experiment runner
              │       └── open_lmm.Runtime
              │               └── native RuntimeClient façade
              │
              ├── replay adapter
              │       └── Goal 03 replay runner/comparator/report
              │
              └── benchmark adapter
                      └── Goal 05 benchmark script/bundle/baseline
```

`open_lmm.experiments`는 `open_lmm`의 공개 Python object만 import한다. `_native`를 direct
import하거나 C++ private header/target을 추가하지 않는다. Goal 07에는 production C++
source 변경이 필요하지 않다.

### 3.2 owner map

| Owner | 소유하는 것 | 소유하지 않는 것 |
|---|---|---|
| `ExperimentPlan` | 요청한 dataset/config/workflow/trial/metric 정책의 immutable 값 | runtime state, live job |
| `TrialPlanner` | deterministic trial 순서, parameter set, seed, run ID | algorithm scheduling |
| `ConfigMaterializer` | trial 전용 config copy와 declared JSON replacement | committed runtime config |
| `ExperimentWorker` | 한 trial의 `Runtime` lifetime과 evidence directory | cross-trial runtime cache |
| `MetricCollector` | bounded callback counters/timestamps와 post-run scalar metric | event authority, unbounded event history |
| `EvidenceWriter` | canonical manifest/result/hash와 no-overwrite publication | baseline 승인 |
| `ReplayAdapter` | Goal 03 tool invocation과 result reference | replay schema/comparison semantics |
| `BenchmarkAdapter` | Goal 05 invocation/import와 metric projection | benchmark sampling/baseline threshold |

Python result/report는 immutable historical evidence다. 다음 runtime operation의 input
authority로 암묵적으로 재사용하지 않는다.

### 3.3 invariant mapping

| Invariant | Goal 07 보존 규칙 |
|---|---|
| INV-01~04 state/commit | command 결과는 `Job.wait()`와 fresh `snapshot()`에서 얻고 Python cache를 authority로 사용하지 않는다. |
| INV-05 late cancel | Python runner가 terminal success를 cancel failure로 재해석하지 않는다. |
| INV-06~07 reconciliation/health | structured `OpenLMMError`와 authoritative snapshot을 trial result에 그대로 기록한다. |
| INV-08 epoch isolation | trial마다 fresh worker process/runtime를 기본으로 하며 Job/event를 다음 trial로 넘기지 않는다. |
| INV-09~10 presentation | visualization은 post-run derived snapshot으로만 읽고 clear/rebuild를 삽입하지 않는다. |
| INV-13 adapter leaf | pure Python package는 public `open_lmm` API와 explicit external tools만 소비한다. |
| INV-14 API/PImpl | `_native`, RuntimeService, MapServer, private C++ type을 import/bind하지 않는다. |
| INV-16 resource | bounded event retention, serial sweep default, point metric opt-in, scalar result만 보존한다. |
| INV-17 lifetime | worker timeout/close/subscription/process-exit contract를 deterministic test로 검증한다. |
| INV-18 diagnostics | metric/report/log는 관측 evidence이며 runtime behavior authority가 아니다. |

## 4. Scope와 non-goal

### 4.1 포함

- local/CI dataset experiment automation
- config copy와 closed JSON-pointer replacement
- RunAll 또는 ordered public Stage workflow
- repeat/seed/Cartesian grid planning
- built-in post-run metrics
- structured failure result
- canonical result JSON, long-form CSV, record iterator
- Goal 03/05 tool adapter
- source-free Goal 06 wheel에서 generic experiment 실행

### 4.2 제외

- Python algorithm/plugin을 production hot path에서 호출
- RuntimeService/controller/executor/state store direct binding
- arbitrary execution node graph와 interactive alignment feedback
- Bayesian optimization, distributed scheduler, remote queue
- notebook object/pickle을 canonical manifest로 사용
- pandas를 mandatory runtime dependency로 추가
- Goal 03 comparator 또는 Goal 05 sampler/baseline을 Python으로 재구현
- auto baseline update, auto threshold widening, auto best-model promotion
- native performance optimization 또는 point-cloud representation 변경
- replay/benchmark tool의 공식 설치 artifact 결정 — Goal 09

## 5. Package와 file layout

```text
open_lmm/src/adapters/python/
├── package/open_lmm/
│   └── experiments/
│       ├── __init__.py
│       ├── _api.py
│       ├── _models.py
│       ├── _canonical.py
│       ├── _manifest.py
│       ├── _config.py
│       ├── _planner.py
│       ├── _metrics.py
│       ├── _runner.py
│       ├── _worker.py
│       ├── _cli.py
│       ├── _export.py
│       ├── replay.py
│       ├── benchmark.py
│       └── schema/
│           ├── experiment_plan.schema.json
│           ├── experiment_result.schema.json
│           └── experiment_trial.schema.json
├── experiments_public_api_v1.txt
└── examples/
    ├── experiment_single.py
    ├── experiment_repeated.py
    ├── experiment_sweep.py
    ├── experiment_replay.py
    └── experiment_benchmark.py

open_lmm/test/adapters/python/experiments/
├── test_public_api.py
├── test_manifest.py
├── test_config_materializer.py
├── test_planner.py
├── test_metrics.py
├── test_export.py
├── test_runner.py
├── test_process_lifetime.py
├── test_replay_adapter.py
└── test_benchmark_adapter.py
```

현재 CMake package staging은 nested `.py`를 복사한다. schema JSON을 wheel/install에
포함하도록 package file glob과 install pattern에 `*.json`을 명시적으로 추가한다.
새 native module이나 C++ library target은 만들지 않는다.

## 6. Python public API v1

### 6.1 module identity

```text
import: open_lmm.experiments
EXPERIMENT_API_VERSION: 1
distribution version: open_lmm.__version__과 동일
native extension change: none
```

`open_lmm` top-level 46-symbol Goal 06 golden manifest는 바꾸지 않는다. Goal 07 public
surface는 `open_lmm.experiments`의 별도 golden manifest로 고정한다.

### 6.2 core API

```python
from os import PathLike
from typing import Iterable, Mapping

class Experiment:
    def __init__(
        self,
        plan: ExperimentPlan,
        *,
        evidence_root: str | PathLike[str],
    ) -> None: ...

    @classmethod
    def from_manifest(
        cls,
        manifest: str | PathLike[str],
        *,
        dataset_root: str | PathLike[str],
        config_root: str | PathLike[str] | None = None,
        evidence_root: str | PathLike[str],
    ) -> "Experiment": ...

    def run(self) -> ExperimentResult: ...

class DatasetSpec:
    id: str
    root: Path
    lock_kind: InputLockKind
    lock_manifest: Path
    lock_sha256: str

class ConfigSpec:
    directory: Path
    files: tuple[LockedFile, ...]
    dataset_bindings: tuple[ConfigTarget, ...]

class ConfigTarget:
    file: str
    pointer: str

class ConfigPatch:
    file: str
    pointer: str
    value: JsonValue

class ParameterAxis:
    name: str
    target: ConfigTarget
    values: tuple[JsonValue, ...]

class AlgorithmVariant:
    id: str
    slot: AlgorithmSlot
    patches: tuple[ConfigPatch, ...]
    expected_plugin_ids: tuple[str, ...]

class ExperimentPlan:
    id: str
    dataset: DatasetSpec
    config: ConfigSpec
    software: SoftwareIdentity
    workflow: WorkflowSpec
    execution: ExecutionPolicy
    metrics: MetricPolicy
    fixed_patches: tuple[ConfigPatch, ...]
    parameter_axes: tuple[ParameterAxis, ...]
    algorithm_variants: tuple[AlgorithmVariant, ...]

class ExperimentResult:
    manifest_sha256: str
    trials: tuple[TrialResult, ...]
    metrics: tuple[MetricRecord, ...]
    summaries: tuple[MetricSummary, ...]
    result: ExperimentStatus

    def records(self) -> tuple[dict[str, JsonValue], ...]: ...
    def write_json(self, path: str | PathLike[str]) -> None: ...
    def write_csv(self, path: str | PathLike[str]) -> None: ...
```

실제 class는 `@dataclass(frozen=True, slots=True)` 또는 동등한 immutable value object로
구현한다. mutable default, open file handle, native object, NumPy point array를 result
DTO에 보존하지 않는다.

`LockedFile`, `SoftwareIdentity`, `WorkflowSpec`, `ExecutionPolicy`, `MetricPolicy`도
immutable public DTO다. JSON value는 `None | bool | int | finite float | str | tuple |
immutable mapping`만 허용하고 validator 진입 시 canonical owned value로 정규화한다.

### 6.3 권장 UX

재현 가능한 기본 경로는 explicit manifest다.

```python
from open_lmm.experiments import Experiment

result = Experiment.from_manifest(
    "experiments/indoor_01.json",
    dataset_root="/datasets/indoor_01",
    evidence_root="results/indoor_01-run-001",
).run()

print(result.summaries)
result.write_csv("results/indoor_01-run-001/metrics.csv")
```

상위 Goal의 짧은 UX는 explicit catalog가 dataset ID를 locked spec으로 해석할 때만
허용한다.

```python
catalog = DatasetCatalog.load("datasets/catalog.json")
result = Experiment(
    ExperimentPlan.simple(
        dataset=catalog.require("indoor_01"),
        config="configs/indoor_01",
    ),
    evidence_root="results/indoor_01-run-001",
).run()
```

검증되지 않은 bare ID를 현재 working directory나 환경변수에서 암묵적으로 검색하지
않는다.

### 6.4 public export lock

`experiments_public_api_v1.txt`는 다음 qualified surface를 고정한다.

```text
open_lmm.experiments
  EXPERIMENT_API_VERSION
  AlgorithmSlot
  AlgorithmVariant
  ConfigPatch
  ConfigSpec
  ConfigTarget
  DatasetCatalog
  DatasetSpec
  ExecutionMode
  ExecutionPolicy
  Experiment
  ExperimentPlan
  ExperimentResult
  ExperimentStatus
  FailurePolicy
  InputLockKind
  JsonValue
  LockedFile
  MetricAvailability
  MetricPolicy
  MetricRecord
  MetricSource
  MetricSummary
  ParameterAxis
  SoftwareIdentity
  TrialResult
  WorkflowKind
  WorkflowSpec

open_lmm.experiments.replay
  ReplayAdapter
  ReplayRequest
  ReplayResult
  ReplayStatus
  ReplayToolchain

open_lmm.experiments.benchmark
  BenchmarkAdapter
  BenchmarkRequest
  BenchmarkResult
  BenchmarkStatus
  BenchmarkToolchain
```

private worker/protocol/config/serializer helper는 export하지 않는다. `DatasetCatalog`는
explicit catalog file만 읽고 global registry나 singleton을 만들지 않는다.

`Experiment.from_manifest(..., config_root=None)`은 manifest에 선언한 config-relative
path를 manifest parent에서 해석한다. 다른 config tree를 사용할 때만 explicit
`config_root`를 요구하며 current working directory를 fallback으로 사용하지 않는다.

### 6.5 status 의미

| Status | 의미 |
|---|---|
| `succeeded` | 모든 required trial과 evidence publication 성공 |
| `partial` | continue policy에서 성공과 실패 trial이 함께 존재 |
| `failed` | required trial 실패 또는 stop policy 중단 |
| `timed_out` | 하나 이상의 required worker가 timeout으로 종료 |
| `invalid` | manifest/protocol/schema/evidence 검증 실패 |

`partial`, `timed_out`, `invalid`는 CI/replay/benchmark PASS로 취급하지 않는다.

### 6.6 CLI contract

`pyproject.toml`에 다음 console entry point를 추가한다.

```text
open-lmm-experiment = open_lmm.experiments._cli:main
```

v1 CLI:

```bash
open-lmm-experiment validate \
  --manifest experiments/indoor_01.json

open-lmm-experiment run \
  --manifest experiments/indoor_01.json \
  --dataset-root /datasets/indoor_01 \
  --config-root configs/indoor_01 \
  --evidence-root results/indoor_01-run-001
```

CLI는 manifest value를 임의 override하는 `--set`을 제공하지 않는다. parameter 변경은
review 가능한 manifest를 새로 만들거나 manifest가 선언한 axis로 수행한다. 정상 종료는
0, trial/runtime failure는 1, usage/schema error는 2, unavailable external tool/data는 77을
사용하며 strict CI는 77을 PASS로 처리하지 않는다.

## 7. Experiment manifest와 identity

### 7.1 plan schema v1

```json
{
  "schema_version": 1,
  "experiment_id": "indoor-01-scan-context-v1",
  "dataset": {
    "id": "indoor-01",
    "lock_kind": "replay-case-v1",
    "lock_manifest_sha256": "sha256:<64hex>"
  },
  "config": {
    "files": [
      {"path": "config.json", "sha256": "sha256:<64hex>"}
    ],
    "dataset_bindings": [
      {"file": "config.json", "pointer": "/directory/root_dir_path"}
    ]
  },
  "software": {
    "open_lmm_version": "3.0.0",
    "runtime_api_version": 1,
    "experiment_api_version": 1,
    "source": {
      "kind": "git",
      "commit": "<40hex>",
      "dirty": false
    }
  },
  "workflow": {"kind": "run-all", "stages": []},
  "execution": {
    "mode": "fresh-process",
    "repetitions": 3,
    "seeds": [101, 102, 103],
    "timeout_seconds": 900,
    "failure_policy": "continue",
    "strict_reproducibility": true,
    "max_trials": 64
  },
  "fixed_patches": [],
  "parameter_axes": [],
  "algorithm_variants": [],
  "metrics": {
    "include_visualization": true,
    "include_points": false,
    "preview_voxel_size_m": null,
    "hash_output_files": true
  }
}
```

schema는 unknown key, duplicate logical name, invalid SHA-256, negative limit,
non-finite number, absolute manifest-relative path를 거부한다.

### 7.2 locator와 identity 분리

`dataset_root`, source config directory, evidence root, executable path는 machine locator다.
이 값을 semantic experiment ID hash에 넣지 않는다. identity에는 stable dataset/config
content digest, software identity, workflow, parameter, seed만 넣는다.

source-free wheel에서 Git repository를 추측하지 않는다. strict run은 다음 중 하나를
요구한다.

- clean Git commit 40-hex와 `dirty=false`
- distribution/wheel artifact SHA-256과 version
- immutable container digest가 포함된 externally supplied software identity

dirty run은 `contract`/interactive evidence로 허용할 수 있지만 baseline/replay 승격
가능 결과로 표시하지 않는다.

### 7.3 canonical JSON과 run ID

- UTF-8, sorted object key, compact separator, trailing newline
- NaN, Infinity, negative zero normalization 금지 또는 fail
- JSON object key는 string만 허용
- SHA-256 표기는 `sha256:<64 lowercase hex>`
- trial ID는 semantic plan digest, trial ordinal, seed, canonical parameter set으로 계산
- Python `hash()`나 dictionary insertion order를 ID에 사용하지 않음

timestamp, PID, hostname, absolute path는 diagnostic field일 수 있으나 identity hash에서
제외한다.

### 7.4 reproducible environment metadata

result manifest에는 allowlisted metadata를 기록한다.

```text
Python implementation/version
OpenLMM distribution/runtime/experiment API version
NumPy version
OS/kernel/architecture
CPU model/count/affinity, memory class
container digest when supplied
OMP/OpenBLAS thread count when explicitly set
worker PID/exit status and process isolation mode
```

full environment dump, username, home/workspace absolute path, URL credential는 기록하지
않는다. 누락된 machine/container 정보는 추정하지 않고 unavailable reason을 남긴다.

## 8. Dataset와 config materialization

### 8.1 input lock

`InputLockKind` v1:

| Kind | 검증 owner | 용도 |
|---|---|---|
| `replay-case-v1` | Goal 03 runner/input-lock contract | locked replay dataset |
| `sha256-index-v1` | Goal 07 streaming file-index validator | 일반 연구 dataset |

`sha256-index-v1`은 sorted relative path, byte size, SHA-256의 closed index다. symlink,
path traversal, device/socket, missing/extra declared file를 거부한다. large file은 chunked
hash를 사용하며 전체 payload를 메모리에 올리지 않는다.

strict mode는 trial마다 input lock을 재검증한다. parent에서 한 번만 검증하는 fast mode는
`input_reverified=false`, `reproducible=false`로 표시하고 baseline/replay comparison에
사용할 수 없다.

### 8.2 config snapshot

각 trial은 source config를 수정하지 않고 fresh trial directory에 materialize한다.

1. declared config file digest 검증
2. symlink와 config root escape 거부
3. exact file tree copy
4. dataset binding replacement
5. fixed patch 적용
6. algorithm variant patch 적용
7. parameter-axis patch 적용
8. materialized file digest와 aggregate config digest 기록
9. `Runtime.open()` validation/commit에 전달

v1 patch operation은 JSON Pointer의 **existing-value replace**만 허용한다. add/remove,
array append, JSON merge patch, arbitrary Python callback은 제외한다. 같은 file/pointer를
두 patch가 수정하면 명시적 precedence 없이 fail한다.

dataset root와 output root는 identity가 아니다. dataset root는 declared binding에만
주입하고 output root는 `Runtime.open(..., output_root=...)`로 전달한다.

## 9. Trial, repeat, seed, parameter sweep

### 9.1 trial planning

trial 순서는 다음 stable product로 생성한다.

```text
algorithm_variants declaration order
  × parameter axes lexical name order / values declaration order
  × repetition ordinal
```

parameter/variant ID는 unique non-empty slug여야 한다. materialize 전에 전체 trial 수를
계산하고 `max_trials`를 넘으면 어떤 output도 만들지 않고 실패한다.

### 9.2 seed 의미

- trial seed는 Python orchestration과 사용자가 명시한 config seed patch를 추적한다.
- `random.seed`와 NumPy generator는 worker 내부 post-processing에만 적용한다.
- native algorithm이 seed option을 제공하지 않으면 seed를 기록했다는 이유로 native
  determinism을 주장하지 않는다.
- native algorithm seed는 반드시 `ParameterAxis`/`ConfigPatch`로 config에 주입하고
  materialized config digest에 포함한다.
- master seed에서 seed를 파생할 때 SHA-256 기반 deterministic derivation을 사용한다.

### 9.3 execution isolation

기본 `ExecutionMode`는 `fresh-process`다.

- parent는 한 번에 한 worker만 실행한다.
- worker마다 새 `Runtime`, config, output, metric state를 만든다.
- result는 pickle/stdout이 아니라 closed JSON file로 전달한다.
- timeout은 parent `wait(timeout)`로 감시하고 grace 후 terminate/kill한다.
- killed trial은 `timed_out`이며 success statistics에서 제외하되 failure count에 남긴다.
- partial output/result는 authoritative success로 publish하지 않는다.

`in-process` mode는 interactive debugging 전용이며 `reproducible=false`다. parallel process
sweep, remote/distributed execution은 v1에서 제외한다.

## 10. Runtime workflow contract

### 10.1 normal path

worker 한 개의 순서:

```text
input/config preflight
  → Runtime.open
  → event subscription
  → RunAll 또는 ordered RunStage/Wait
  → authoritative Runtime.snapshot
  → requested Visualization queries
  → subscription close/drain
  → Runtime.close
  → post-run file/metric extraction
  → trial result publish
```

stage별 workflow는 public `Stage`만 허용한다. node/execution graph를 Python에 새로
노출하지 않는다.

### 10.2 failure path

- `OpenLMMError`의 code, severity, full context를 구조화해 보존한다.
- 실패 직후 가능한 경우 snapshot을 재조회해 revision/artifact/health evidence를 남긴다.
- failed command 이후 추가 mutating command를 자동 실행하지 않는다.
- subscription drain과 close를 best effort로 수행하고 각각의 결과를 별도 기록한다.
- user exception과 worker protocol exception을 native runtime error와 구분한다.
- trial failure를 빈 metric 또는 0으로 바꾸지 않는다.

`failure_policy=continue`는 다음 independent trial을 실행한다는 뜻이며 실패를 PASS로
바꾸는 옵션이 아니다.

### 10.3 event collection bound

callback은 다음만 수행한다.

- `time.monotonic_ns()` timestamp
- event type/stage/sequence counter 갱신
- stage start/terminal timestamp 갱신
- last terminal/error의 소유 immutable copy
- bounded recent-event ring 갱신

file I/O, NumPy work, config mutation, runtime lifecycle command를 callback에서 수행하지
않는다. event count는 scalar로 유지하고 retained sample은 기본 256개로 제한한다.
overflow는 drop count로 진단하며 correctness replay가 full event sequence를 필요로 하면
Goal 03 runner를 사용한다.

## 11. Metric contract

### 11.1 common record

```text
MetricRecord
  name
  value                 # finite int/float/bool/string 또는 null
  unit
  source                # python-callback | snapshot | visualization |
                        # output-file | replay-v1 | benchmark-v1
  scope                 # experiment | trial | stage | agent | artifact
  subject               # optional stage/agent/artifact key
  availability          # available | not_available | invalid
  reason
```

metric name과 unit은 versioned vocabulary다. 같은 이름에 다른 단위를 쓰지 않는다.
unavailable metric을 0으로 기록하지 않는다.

### 11.2 built-in metric

| Group | Metric 예 | Source/주의 |
|---|---|---|
| workflow | total wall, command wall, per-stage callback latency | Python monotonic callback arrival; Goal 05 baseline과 별개 |
| runtime | runtime/config revision, status, agent count | authoritative final snapshot |
| artifact | type/state count, ready/stale/failed count, output-relative path | snapshot artifact inventory |
| alignment | edge count/type, inter-agent pair count, affected agent set | visualization edge/pose identity |
| trajectory | pose count, path length, translation AABB extent | visualization pose matrices |
| map | source/displayed point count, bounds extent/volume, completeness flags | visualization metadata |
| point opt-in | centroid/finite count/intensity range | read-only NumPy array; full array 결과 보존 금지 |
| output | file count, logical bytes, optional content SHA-256 | post-close streaming scan |
| process | worker max RSS/HWM, exit code | fresh worker diagnostic; Goal 05 owner metric 아님 |

Python stage latency는 callback enqueue/delivery overhead를 포함한다. report field에
`source=python-callback`을 고정하고 performance regression gate로 자동 승격하지 않는다.
여러 agent visualization에 같은 edge가 보이면
`(from_agent, from_index, to_agent, to_index, type)` canonical key로 deduplicate한다.
process HWM은 Linux `RUSAGE_SELF`/`/proc` 단위를 bytes로 명시적으로 변환하고 platform이
다르면 unavailable reason을 남긴다.

### 11.3 advanced accuracy

ATE/RPE나 map-to-ground-truth distance는 ground-truth frame, association, alignment policy가
명시돼야 한다. v1 core는 정책이 없는 값은 계산하지 않는다.

- Goal 03 baseline 비교가 있으면 canonical replay comparator 결과를 import한다.
- 일반 ground-truth evaluator는 별도 versioned adapter로 추가할 수 있다.
- quaternion sign, frame direction, timestamp association을 추정하지 않는다.

### 11.4 resource/copy policy

- `include_points=false`가 기본값이다.
- point metric을 요청하면 `preview_voxel_size_m`과 complete/incomplete 상태를 함께 기록한다.
- NumPy reduction은 기존 read-only array 위에서 수행하고 full `.tolist()`/duplicate array를
  만들지 않는다.
- `TrialResult`에는 scalar metric만 남기며 array lifetime을 연장하지 않는다.
- process RSS와 Goal 05 owner reservation을 하나의 `memory_bytes`로 합치지 않는다.

## 12. Result, aggregation, export

### 12.1 trial result schema

각 trial은 다음을 갖는다.

- trial ID, ordinal, seed, parameter/variant values
- plan/config/dataset/software digest
- status: `succeeded | failed | cancelled | timed_out | invalid`
- runtime revision before/after와 final snapshot summary
- structured error와 close result
- metric record 목록
- output artifact relative path/size/hash
- worker exit/protocol diagnostic
- started/finished timestamp는 diagnostic only

### 12.2 experiment aggregate

numeric metric summary:

```text
available_count, missing_count, failure_count,
median, nearest-rank p95, MAD, min, max
```

실패 trial을 통계에서 조용히 제거하지 않는다. summary는 성공 metric sample을 계산하되
전체 trial status count와 selection predicate를 함께 기록한다. 평균만 제공하지 않는다.

Goal 05 bundle을 import하면 그 bundle의 summary/comparison을 보존하고 Python에서 새
threshold를 계산하지 않는다.

### 12.3 output layout

```text
<evidence-root>/
├── plan.json
├── manifest.json
├── trials/
│   └── 0001-<short-run-id>/
│       ├── config/
│       ├── runtime-output/
│       ├── trial-result.json
│       └── worker.log
├── experiment-result.json
├── metrics.csv
└── evidence.sha256
```

evidence root는 존재하지 않아야 한다. writer는 같은 directory의 temporary file을 fsync한
뒤 no-overwrite publish하고, 기존 result/baseline을 덮어쓰지 않는다. directory fsync와
crash recovery journal은 Goal 07 범위 밖이며 그 보장을 암묵적으로 주장하지 않는다.

### 12.4 CSV와 pandas

CSV는 long form이다.

```text
experiment_id,trial_id,trial_index,seed,status,
parameter_json,variant_id,metric,unit,value,source,scope,subject,
availability,reason
```

composite 값은 canonical JSON string으로 넣는다. `ExperimentResult.records()`는 같은
column을 가진 Python dict sequence를 반환해 사용자가
`pandas.DataFrame(result.records())`를 호출할 수 있게 한다. pandas는 mandatory dependency로
추가하지 않는다. optional convenience `to_pandas()`를 추가한다면 import 시점이 아니라
호출 시점에만 pandas를 요구한다.

## 13. Goal 03 replay integration

### 13.1 adapter API

```python
class ReplayToolchain:
    runner: Path
    comparator: Path

class ReplayRequest:
    case_manifest: Path
    data_root: Path
    config_root: Path
    baseline: Path
    evidence_root: Path
    software: SoftwareIdentity
    container_digest: str

class ReplayAdapter:
    def run(self, request: ReplayRequest) -> ReplayResult: ...
```

### 13.2 delegation rule

- `open_lmm_replay_runner`가 input lock, workflow, report를 소유한다.
- `open_lmm_replay_compare`가 exact/tolerance/range 판정을 소유한다.
- Python adapter는 argument vector, exit status, report/diff path/hash만 관리한다.
- `shell=True`, PATH의 우연한 executable, baseline rewrite를 사용하지 않는다.
- runner/comparator를 explicit absolute path로 받고 executable/file identity를 기록한다.
- return code 77 `NOT_AVAILABLE`은 strict/CI에서 PASS가 아니다.
- baseline SHA-256을 실행 전후 비교해 mutation을 거부한다.

Goal 03 tool은 현재 test build asset이다. Goal 07 wheel에 임의로 복제/설치하지 않는다.
source-free generic experiment는 wheel만으로 동작하지만 replay integration은 explicit
developer toolchain이 필요하다. 공식 Tools/Python artifact 결합은 Goal 09가 결정한다.

### 13.3 imported metric

replay report의 case/dataset/config digest, step result/revision, health, visualization metric,
comparator diff를 `source=replay-v1` record로 projection한다. original report와 baseline의
content hash를 반드시 남기며 field 의미를 새로 해석하지 않는다.

## 14. Goal 05 benchmark와 algorithm adapter

### 14.1 canonical benchmark adapter

```python
class BenchmarkToolchain:
    script: Path
    build_root: Path
    baseline: Path | None

class BenchmarkRequest:
    profile: str
    fixture: str
    scenario: str
    repetitions: int
    warmups: int
    evidence_root: Path
    container_digest: str
    sanitizer: str

class BenchmarkAdapter:
    def run(self, request: BenchmarkRequest) -> BenchmarkResult: ...
    def load(self, bundle: Path) -> BenchmarkResult: ...
```

adapter는 `scripts/benchmark/run_benchmarks.sh`를 explicit `bash` argv로 호출하고 raw/bundle/
pair schema version, result, comparison, baseline identity를 확인한다. `pass`, `fail`,
`uncalibrated`, `baseline_mismatch`, `not_available`을 서로 다른 상태로 보존한다.

Goal 05의 private owner runner를 Python package에 link/import하지 않는다. approved
`performance_baseline.json`을 Python이 수정하거나 tolerance를 재계산하지 않는다.

### 14.2 algorithm variant adapter

`AlgorithmVariant`는 algorithm 실행 callback이 아니라 config patch bundle이다.

```text
slot = data_loader | loop_detector | backend_optimizer | dynamic_remover
id = stable variant ID
patches = closed JSON replacements
expected_plugin_ids = manifest evidence
benchmark_mapping = optional Goal 05 scenario set
```

일반 variant 비교는 fresh-process Experiment로 실행한다. Goal 05 canonical benchmark와
호환되는 fixed fixture/scenario만 `BenchmarkAdapter`로 실행한다. arbitrary algorithm config를
Goal 05 baseline key에 강제로 끼워 넣지 않고 key mismatch/uncalibrated로 남긴다.
Goal 06 snapshot에는 selected plugin ID가 없으므로 일반 experiment의
`expected_plugin_ids`는 materialized config provenance다. runtime-confirmed plugin
identity라고 주장하지 않으며, canonical verification이 필요하면 plugin ID를 포함하는
Goal 03/05 report를 사용한다.

### 14.3 subprocess safety

- argv list만 사용하고 shell interpolation 금지
- explicit executable/script/build root 존재·권한 검사
- output root must-not-exist
- bounded log file와 maximum byte retention
- timeout 후 process group 종료와 timed-out evidence
- parent environment는 allowlist로 구성하고 secret value를 report에 기록하지 않음
- current working directory와 absolute executable path는 diagnostic이며 identity는 digest로 기록

## 15. 구현 작업 패키지

### P00 — baseline과 API lock

- 이 문서 승인
- `experiments_public_api_v1.txt` 작성
- plan/trial/result schema v1 작성
- exact HEAD/full CTest/policy baseline 기록

완료 증거: golden API와 schema contract test.

### P01 — package skeleton

- `open_lmm.experiments` package와 `EXPERIMENT_API_VERSION=1`
- `open-lmm-experiment validate/run` console entry point
- nested schema package/install/wheel staging
- top-level Goal 06 API manifest 무변경 확인

완료 증거: build-tree와 source-free wheel import PASS.

### P02 — canonical manifest와 input identity

- canonical JSON/SHA-256/finite JSON 구현
- DatasetSpec/ConfigSpec/SoftwareIdentity validation
- sha256 index streaming validator와 strict dirty/source policy

완료 증거: ordering/path/symlink/tamper/dirty negative test.

### P03 — config materializer와 trial planner

- config copy와 JSON Pointer replace
- fixed/variant/axis conflict detection
- deterministic Cartesian product/run ID/seed derivation/max-trial guard

완료 증거: source config unchanged, compiler/process-independent golden plan.

### P04 — worker process와 runtime workflow

- parent/worker closed JSON protocol
- fresh Runtime Open→Run/Wait→Snapshot→Close
- structured error/timeout/close result
- serial default와 no-overwrite evidence root

완료 증거: repeated trial의 process/runtime identity 격리와 failure continuation PASS.

### P05 — metric collector

- bounded callback accumulator
- snapshot/artifact/alignment/trajectory/map/output/process metric
- point metric opt-in, no array retention
- unavailable reason과 summary statistics

완료 증거: known tiny fixture value parity와 bounded retention/RSS check.

### P06 — export와 result bundle

- trial/experiment closed JSON
- long-form CSV, records API, evidence hash manifest
- no-overwrite/partial publish failure test

완료 증거: locale/key-order-independent hash와 round-trip PASS.

### P07 — replay adapter

- explicit Goal 03 toolchain/request/result
- runner/comparator execution, baseline read-only check
- report metric projection와 NOT_AVAILABLE/failure distinction

완료 증거: existing synthetic replay fixture와 tiny fixture report contract PASS.

### P08 — benchmark와 algorithm adapter

- Goal 05 bundle loader와 workflow adapter
- comparison status projection
- AlgorithmVariant/config mapping
- baseline mismatch/uncalibrated 보존

완료 증거: existing small benchmark bundle/schema fixture와 adapter contract PASS.

### P09 — test/CI/package policy

- CTest metadata 등록
- architecture gate에 `_native`/private import와 shell invocation 금지 추가
- GCC/Clang Python test, ASan native regression, full CTest, package/wheel 검증

완료 증거: policy/full/wheel/source-free test PASS.

### P10 — docs/examples/final evidence

- single/repeat/sweep/replay/benchmark example
- supported/unsupported matrix와 known limitation
- result 문서에 exact commands/count/hash/sample output 기록

완료 증거: fresh wheel venv examples와 Goal 07 completion checklist.

## 16. Test specification

### 16.1 CTest portfolio

| Test | Layer | Module | Owner | Invariants | 핵심 검증 |
|---|---|---|---|---|---|
| `open_lmm_python_experiment_api_tests` | L2 | experiments.api | ExperimentAPI | INV-13,14 | golden exports, Goal 06 top-level 무변경 |
| `open_lmm_python_experiment_cli_tests` | L2 | experiments.cli | ExperimentCLI | INV-13,14,18 | argv/exit status/no implicit override |
| `open_lmm_python_experiment_manifest_tests` | L2 | experiments.manifest | ExperimentManifest | INV-02,14,18 | schema/hash/path/input lock |
| `open_lmm_python_experiment_planner_tests` | L2 | experiments.planner | TrialPlanner | INV-08,16,18 | stable product, seed, max trial |
| `open_lmm_python_experiment_config_tests` | L3 | experiments.config | ConfigMaterializer | INV-01,02,03,04 | copy/patch/conflict/source immutability |
| `open_lmm_python_experiment_metric_tests` | L3 | experiments.metrics | MetricCollector | INV-09,10,16,18 | bounds/trajectory/availability/bounded events |
| `open_lmm_python_experiment_export_tests` | L2 | experiments.export | EvidenceWriter | INV-14,16,18 | JSON/CSV/no-overwrite/hash |
| `open_lmm_python_experiment_runner_tests` | L4 | experiments.workflow | ExperimentWorker | INV-01~08,17 | fresh process Open→Run→Snapshot→Close |
| `open_lmm_python_experiment_lifetime_tests` | L5 | experiments.lifetime | ExperimentWorker | INV-08,17 | timeout/cancel/subscription/process exit |
| `open_lmm_python_experiment_replay_tests` | L4 | experiments.replay | ReplayAdapter | INV-02,03,04,14 | Goal 03 delegate/baseline read-only/status |
| `open_lmm_python_experiment_benchmark_tests` | L3 | experiments.benchmark | BenchmarkAdapter | INV-13,16,18 | bundle projection/status/key mismatch |

모든 test는 `openlmm_add_test`로 owner/layer/invariant/lane을 등록한다. sleep/polling을
correctness oracle로 사용하지 않고 Event, process exit, pipe/result-file protocol,
subprocess timeout을 사용한다.

### 16.2 필수 behavior cases

#### Manifest/config

- key order와 locale이 달라도 같은 semantic digest
- NaN/Infinity/absolute relative-path/unknown key 거부
- dataset/config hash tamper fail closed
- symlink escape, path traversal, missing/extra file 거부
- JSON Pointer missing target/type mismatch/duplicate target 거부
- source config byte-identical 보존
- dirty/software identity strict-mode 거부

#### Planner/runner

- Cartesian order와 run ID golden
- repeat/seed/variant identity separation
- max trial guard가 output 생성 전에 실패
- trial마다 distinct PID/runtime/output/config
- normal RunAll과 ordered stages
- failure continue/stop policy
- native structured error/context 보존
- timeout/worker crash/invalid result protocol
- callback unsubscribe와 Runtime close 후 process exit

#### Metric/export

- stage sequence mismatch면 latency unavailable
- artifact/edge/pose/map bounds known fixture parity
- read-only point array reduction과 full array retention 0건
- event retention cap과 dropped count
- failure/unavailable을 zero로 변환하지 않음
- median/p95/MAD boundary와 failure count
- JSON round-trip, CSV stable columns, no overwrite

#### Replay/benchmark

- explicit executable path만 허용
- shell metacharacter가 argv data로 유지
- replay baseline before/after hash 동일
- comparator fail과 NOT_AVAILABLE이 PASS가 아님
- benchmark pass/fail/uncalibrated/baseline_mismatch/not_available 보존
- Goal 05 source bundle/baseline hash projection
- unknown schema/version fail closed

## 17. Validation lanes와 명령 계약

### 17.1 local order

```text
1. Python public/schema/unit tests
2. generic tiny fixture repeated/sweep L4
3. Goal 03 synthetic replay adapter L4
4. Goal 05 bundle/adapter contract
5. architecture/release policy
6. Python-enabled GCC 12 full CTest
7. Python-enabled Clang 15 test
8. applicable ASan+UBSan native/Python bridge regression
9. local wheel build and source-free experiment example
10. git diff --check
```

### 17.2 representative commands

```bash
cmake -S open_lmm -B /tmp/openlmm-goal07-gcc12 \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DOPEN_LMM_BUILD_PYTHON=ON
cmake --build /tmp/openlmm-goal07-gcc12 -j2

ctest --test-dir /tmp/openlmm-goal07-gcc12 --output-on-failure \
  -R '^open_lmm_python_experiment_.*_tests$'
ctest --test-dir /tmp/openlmm-goal07-gcc12 --output-on-failure -j2

bash scripts/ci/check_architecture_policy.sh
git diff --check
```

실제 구현 결과에는 build directory, compiler version, test count, replay/benchmark fixture
hash, wheel SHA-256, example evidence hash를 기록한다.

### 17.3 CI policy

- pure Python contract와 tiny generic experiment는 `pr` lane
- real internal replay는 Goal 03 trusted workflow의 secret/immutable input 정책을 그대로 사용
- canonical medium benchmark는 Goal 05 nightly workflow를 그대로 사용
- Goal 07이 replay/benchmark workflow의 branch-protection 이름이나 baseline을 대신 소유하지 않음
- adapter integration failure artifact에는 plan/result/diff/log을 제한된 크기로 업로드

## 18. Compatibility와 versioning

- `EXPERIMENT_API_VERSION=1`은 Python experiment surface version이다.
- Goal 06 `API_VERSION=1`, plugin ABI v1, replay schema v1, benchmark schema v1과 서로
  독립적으로 versioning한다.
- `open_lmm.experiments` public exports는 separate golden manifest를 사용한다.
- public signature, enum 의미, manifest/result schema의 breaking change는 compatibility
  review와 new major schema가 필요하다.
- metric 추가는 additive지만 같은 name/unit/meaning 변경은 breaking이다.
- replay/benchmark original report는 adapter result에 content hash와 schema version으로
  참조하고 lossless source artifact를 보존한다.
- pandas DataFrame column contract는 CSV/records column과 함께 versioning한다.

## 19. Resource, security, privacy

- dataset/config/evidence path는 user-controlled input이며 root escape를 거부한다.
- manifest/report에 environment 전체를 dump하지 않고 allowlisted metadata만 기록한다.
- URL credential, token, secret env, private dataset absolute path를 result에 포함하지 않는다.
- log는 size cap과 truncation flag를 가지며 stdout/stderr를 unbounded memory로 capture하지 않는다.
- event history와 trial result는 bounded; large points/raw files를 JSON에 embed하지 않는다.
- sweep는 serial default와 max-trial guard를 사용한다.
- subprocess는 explicit argv/process group을 사용하고 shell string을 실행하지 않는다.
- result의 artifact path는 evidence root 상대경로다.
- snapshot `external_path`가 runtime output root 밖을 가리키면 절대 경로를 export하지 않고
  stable redacted locator와 `outside_output_root` reason만 기록한다.
- internal dataset license/redistribution status는 report에 보존하며 Python export가 권리를
  확대하지 않는다.

## 20. Risk와 rollback

| Risk | 방지/검증 | Rollback boundary |
|---|---|---|
| experiment state가 runtime authority가 됨 | 매 command 후 snapshot 재조회, derived-only DTO | `_runner.py` 제거 |
| config source mutation | fresh copy, replace-only, digest before/after | `_config.py` 단독 rollback |
| cross-trial cache/epoch leak | fresh process default | worker/process layer rollback |
| event/RSS memory growth | bounded ring/scalar aggregation/no arrays | metric collector rollback |
| replay semantics duplication | Goal 03 CLI delegation와 report hash | `replay.py` 제거 |
| benchmark threshold drift | Goal 05 bundle/baseline read-only | `benchmark.py` 제거 |
| sweep explosion | preflight product/max-trial guard | planner rollback |
| wheel dependency 증가 | stdlib + existing NumPy, pandas optional | experiment package만 제외 |
| secrets/path leakage | allowlist/redaction/relative artifact path | export metadata 축소 |

Goal 07 rollback이 native binding, runtime core, public C++ header, plugin ABI를 변경하지 않아야
한다.

## 21. Completion conditions

다음이 모두 충족돼야 Goal 07 구현 완료로 판정한다.

- [x] `open_lmm.experiments` API와 golden manifest
- [x] versioned closed plan/trial/result schema
- [x] dataset/config/software input lock과 strict reproducibility 판정
- [x] fresh-process Open/Run/Wait/Snapshot/Artifacts/Close batch runner
- [x] repeat, seed tracking, deterministic Cartesian sweep, max-trial guard
- [x] stage/artifact/alignment/trajectory/map/resource metric contract
- [x] bounded event/point/output memory policy 검증
- [x] JSON/CSV/records export와 no-overwrite evidence bundle
- [x] Goal 03 replay runner/comparator integration
- [x] Goal 05 benchmark bundle/baseline integration
- [x] algorithm variant config adapter와 example
- [x] normal/failure/timeout/process-exit deterministic tests
- [x] GCC 12/Clang 15 Python validation과 applicable sanitizer regression
- [x] source-free local wheel에서 generic experiment/repeat/sweep example PASS
- [x] existing full CTest/package/policy regression PASS
- [x] exact HEAD, commands, test count, artifact/schema hash 기록

Goal 07 완료는 experiment automation이 가능하다는 뜻이다. public portable wheel, official
Tools artifact, release signing/promotion은 Goal 09가 담당한다.

## 22. 구현 결과

### 22.1 최종 판정

**COMPLETE — pure-Python local implementation.**

Goal 07의 P00–P10과 §21 완료조건을 모두 구현·검증했다. 구현은
`open_lmm.experiments`라는 Python leaf에 한정됐으며 native binding, runtime core,
public C++ header, plugin ABI, replay comparator, benchmark threshold owner를 변경하지 않았다.

구현은 `develop`의 exact base HEAD
`bafed911604d65cb76eac1692ce0009a4e6dd117`에서 시작했다. 위 SHA는 구현 기준점이지 결과
commit이 아니다. 이 문서 자체가 결과 commit에 포함되므로 자기 참조 SHA는 문서에
고정하지 않으며, 결과 commit의 canonical SHA는 Git history에서 확인한다.

### 22.2 구현 파일과 owner

| Owner | 구현 |
|---|---|
| public API/model | `open_lmm/experiments/{__init__,_api,_models}.py`, `EXPERIMENT_API_VERSION=1`, 별도 golden export |
| canonical identity/manifest | `_canonical.py`, `_manifest.py`, plan schema |
| input/config | `_config.py`, exact file-set/digest 검증, replace-only JSON Pointer materialization |
| trial planning | `_planner.py`, variant × lexical axis × repetition product, deterministic seed/ID, pre-output max guard |
| worker/lifetime | `_runner.py`, `_worker.py`, `_subprocess.py`, fresh process, closed protocol, process-group timeout/reap, bounded log drain |
| metrics/export | `_metrics.py`, `_export.py`, bounded event ring, scalar/summary metric, JSON/CSV/records/evidence SHA bundle |
| Goal 03 delegation | `replay.py`, explicit runner/comparator, baseline read-only hash, canonical status 보존 |
| Goal 05 delegation | `benchmark.py`, existing workflow/bundle loader, comparison/baseline status 보존 |
| UX/package | `_cli.py`, `open-lmm-experiment`, 5개 example, README, wheel JSON schema 설치 |
| policy/test | 11개 Goal 07 CTest, source-free wheel test, architecture/release policy gate |

production package에는 새 C++ target이나 private runtime import가 없다. architecture policy는
experiment package 전체에서 direct `_native` import, private runtime/plugin-host 경로,
`shell=True`, `os.system()`을 금지한다. subprocess는 explicit argv와 allowlisted environment만
사용한다.

### 22.3 구현된 계약

- `sha256-index-v1`은 manifest 자신을 제외한 sorted exact file set, size, SHA-256을 매
  strict trial마다 재검증한다. missing/extra/tamper/symlink/special file은 fail closed다.
- config는 declared file set을 검증한 뒤 trial별 새 directory로 복사하며 dataset binding,
  fixed patch, algorithm variant, lexical parameter axis 순서로 existing value만 교체한다.
- trial은 fresh Python process가 기본이고 PID, runtime, config, output을 공유하지 않는다.
  `in-process`는 명시적 debug mode이며 `reproducible=false`다.
- stage callback은 bounded ring과 scalar counter만 유지한다. point metric은 opt-in read-only
  reduction이며 full point array를 result에 보존하지 않는다. stdout/stderr도 bounded
  threaded drain을 사용한다.
- 결과는 `plan.json`, `manifest.json`, per-trial result/log/config, aggregate JSON, long-form
  CSV, `evidence.sha256`으로 publish하며 기존 evidence root를 덮어쓰지 않는다.
- ReplayAdapter와 BenchmarkAdapter는 source report를 closed top-level contract로 읽고
  canonical status와 hashes를 projection한다. comparator/threshold/baseline 계산은 재구현하지
  않는다.

normal worker 순서는 구현 전 명세의 subscription/open 순서를 한 번 조정했다. Goal 06 public
`Runtime`은 open 전 subscription을 허용하지 않으므로 실제 순서는
`input preflight → Runtime.open → subscribe_events → run/wait → snapshot/visualization →
subscription.close → Runtime.close`다. public façade 밖으로 우회하지 않으며 lifecycle
authority도 추가하지 않는다.

### 22.4 Test와 validation 결과

| Lane | 최종 결과 | 비고 |
|---|---:|---|
| GCC 12 Goal 07 CTest | **11/11 PASS** | API/CLI/manifest/planner/config/metric/export/runner/lifetime/replay/benchmark |
| GCC 12 full CTest | **99/99 PASS** | `/root/workspace/build/open_lmm`, serial authoritative run |
| Clang 15 Goal 07 + policy | **13/13 PASS** | `/tmp/openlmm-goal06-clang15-build`; Goal 07 11 + architecture/release 2 |
| ASan+UBSan applicable manifest | **71/71 PASS** | `/tmp/openlmm-goal06-asan-build`; CI와 같은 `detect_leaks=0`, halt-on-error |
| source-free wheel | **1/1 CTest, 4/4 unittest PASS** | import/runtime/generic experiment + repeat + sweep, `/tmp`에서 `-I`, `PYTHONPATH`/`LD_LIBRARY_PATH` unset |
| architecture/release policy script | **PASS** | direct native/private/shell 경계 포함 |
| `git diff --check` | **PASS** | 최종 commit candidate |

Clang 구성은 `OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF`라 Goal 05 benchmark runner target이
없다. 이 구성에서는 adapter가 canonical script failure를 명시적 `fail`로 보존하는 경로를
검증했고, runner가 모두 존재하는 GCC 구성에서는 실제 Goal 05 `contract/small-v1/open`
bundle을 실행해 `uncalibrated`와 metric/hash를 검증했다. Goal 03 adapter 역시 현재 build의
실제 `open_lmm_replay_compare`와 existing fixture를 실행했다.

ASan Python process를 임의로 직접 실행하면 일반 CPython보다 sanitizer runtime이 먼저
load돼야 하고 container에서 LeakSanitizer ptrace가 제한된다. 따라서 최종 sanitizer 판정은
저장소의 canonical sanitizer manifest/CI 조건을 사용했으며 71/71이 통과했다. Goal 07의
pure-Python 11개는 GCC·Clang 양쪽에서 별도로 전부 통과했다.

최종 대표 명령은 다음과 같다.

```bash
ctest --test-dir /root/workspace/build/open_lmm --output-on-failure -j1
ctest --test-dir /tmp/openlmm-goal06-clang15-build --output-on-failure \
  -R '^(open_lmm_python_experiment_.*_tests|open_lmm_architecture_boundary_tests|open_lmm_release_policy_tests)$'
ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  ctest --test-dir /tmp/openlmm-goal06-asan-build --output-on-failure \
    -L 'sanitizer:asan-ubsan'
ctest --test-dir /root/workspace/build/open_lmm --output-on-failure \
  -R '^open_lmm_python_wheel_tests$'
bash scripts/ci/check_architecture_policy.sh
git diff --check
```

### 22.5 Package, schema, evidence identity

local wheel은 restricted-network 환경에서 기존 reviewed dependency source cache를 사용해
CPython 3.10/GCC 12, final parallel level 1로 다시 만들고 fresh venv에 설치했다.

| Artifact | SHA-256 |
|---|---|
| `open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl` | `73377b698c3486ca104d3ff30adb8f551cc43e19ece294a5af4b1f84cdfc7ee6` |
| `experiments_public_api_v1.txt` | `d1f8edf4f3a1c386adf23380364f70b870694293a6a15be3f26271e05c7e4ded` |
| `experiment_plan.schema.json` | `dc62a8a17b6720d9c409db3fa3bb65cb8f12b1cdaa9f260b41ea9888b31c1fef` |
| `experiment_trial.schema.json` | `004d81604e9f824941a06d1c6a32372cd3073c67879d4cbbfff212813e71ba1f` |
| `experiment_result.schema.json` | `7735a726e44b1ae1da59ac6bcd9f14e76ae71a4939c464ab371ac440f5de7881` |

GCC tiny 2-repeat sample은 두 trial 모두 `succeeded`, seed `101/102`, distinct worker PID,
strict input revalidation을 기록했다. 시간/PID가 포함되므로 이 hash를 regression golden으로
사용하지 않고 이번 실행의 provenance로만 기록한다.

| Sample artifact | SHA-256 |
|---|---|
| `plan.json` | `b65c083308032cd2e398c3db28a17f6addd3ec4c5c87b0b61901c71198b95057` |
| `manifest.json` | `a9694b9b681af15bd9473513d73e4f2f33228f51550a672606756542d83b7277` |
| `experiment-result.json` | `03677f2b3bd8f35c48bb86f4d4968a3c12d7516e00c7b13b002bb10faa7c1345` |
| `evidence.sha256` | `6e204d395b36199d62666099e481de7b56921578ecf274c0e1265f0eb2153dfe` |

### 22.6 남은 제한과 후속 owner

이 제한은 Goal 07 미완료가 아니라 의도한 v1 범위다.

- local serial CPython 3.10/Ubuntu 22.04 wheel만 지원한다. portable/public wheel,
  Tools artifact, signing/promotion은 Goal 09다.
- distributed/parallel sweep, remote executor, Python algorithm plugin, Python hot-path callback은
  지원하지 않는다.
- generic runner는 `sha256-index-v1`만 실행한다. `replay-case-v1`은 반드시 ReplayAdapter로
  Goal 03 owner에 위임한다.
- ground-truth ATE/RPE와 dataset별 accuracy 정책은 dataset owner가 정의하기 전까지 자동
  계산하지 않는다.
- pandas는 optional lazy conversion이며 core package dependency가 아니다.
- Goal 05 baseline 승인/promotion이나 threshold 재계산은 BenchmarkAdapter 범위가 아니다.

## 23. 원래 구현 착수 판정(보존)

**GO — pure-Python local implementation.**

현재 `RuntimeClient` 기반 Python API, Goal 03 replay contract, Goal 05 benchmark contract가
필요한 extension point를 제공한다. 구현은 public façade와 external report boundary에서
끝나므로 frozen architecture를 변경하지 않는다.

구현 중 다음이 필요해지는 순간에는 이 GO 판정을 중지한다.

- private runtime/domain/plugin-host import/link
- Python이 소유하는 두 번째 runtime/config/resource authority
- algorithm hot path의 Python callback
- replay/baseline/benchmark comparator 재구현 또는 자동 갱신
- installed replay/benchmark Tools ABI의 새 public promise

그 경우 production 구현에 섞지 않고 architecture/release proposal을 별도로 제출한다.
