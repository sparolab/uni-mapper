# Goal 04 개발 명세서: Soak / Fault / Stress Infrastructure

- 상태: **HEADLESS PASS — CALIBRATION / EXTERNAL / REAL-DRIVER DEFERRED**
- 작성일: 2026-08-20 UTC
- 구현 기준 HEAD: `f7055d675d462f59ac13773badb7740751c25b5d` (`develop`)
- 동결 기준: `59e003ebc4b7d44597ced4ddab3436adec310370`
- 상위 목표: `docs/pose_freeze_goals/open_lmm_post_freeze_goals/04_soak_fault_stress_goal.md`

> 작성 시 worktree에는 Goal 03 replay 구현 변경이 존재한다. 이 문서는 그 변경을
> 소유하거나 재분류하지 않으며, Goal 04 구현 커밋은 기존 변경과 분리해야 한다.

## 1. 목적과 완료 정의

Goal 04는 단발성 correctness test를 늘리는 작업이 아니다. 동일 runtime lifecycle과
fault timing을 반복했을 때 다음 세 종류의 누적 결함이 없음을 재현 가능한 증거로
만드는 작업이다.

1. committed state, epoch, presentation, recovery authority가 반복 경로에서 깨지는 결함
2. job/event/subscription/task/reservation/cache/file owner가 반복 후 남는 결함
3. close/cancel/replacement/callback 경계에서만 드러나는 race, deadlock, UAF

완료 시 다음이 모두 존재해야 한다.

- deterministic fault scenario를 조합하는 재사용 가능한 harness
- PR용 fast stress와 nightly soak의 분리된 명령 및 CTest/CI lane
- 최소 100회 fast lifecycle과 1,000회 nightly lifecycle 실행 경로
- RSS/thread/fd와 owner metric의 시계열 및 slope 판정
- ASan/UBSan 및 TSan 반복 subset
- headless GUI와 real-driver GPU tier의 명시적 분리
- seed, commit, compiler, sanitizer, machine metadata를 포함한 JSON report
- 현재 자동화할 수 없는 경우와 그 이유를 기록한 known-limitations 목록

실제 1,000회 headless 결과는 생성했다. RSS hard threshold는 아직 clean two-compiler
5-run calibration 전이므로 확정하지 않으며 external/real-driver tier와 분리한다.

## 2. 아키텍처 범위

### 2.1 허용 범위

- `open_lmm/test/soak/**`
- `open_lmm/test/support/soak/**`
- 기존 owner별 `fault_concurrency` fixture의 재사용 가능한 부분 추출
- `open_lmm/test/cmake/registrations/**`와 test manifest metadata
- `scripts/soak/**`, `scripts/ci/run_soak_tests.sh`
- `.github/workflows/nightly-soak.yml`
- 필요한 경우 private production owner의 read-only diagnostic snapshot 한 개

### 2.2 금지 범위

- `RuntimeStateStore`, `RuntimeService`, `StageCoordinator`, `StageExecutor`의 소유권 분할
- 별도 lifecycle/state/presentation/resource registry 도입
- installed `RuntimeClient` API 또는 plugin ABI 변경
- fault를 만들기 위한 process-global boolean, environment-driven production branch,
  singleton hook 추가
- `sleep_for`, `sleep_until`, 반복 `yield()`로 race window를 추측하는 테스트
- Goal 05의 benchmark/성능 최적화, streaming/sidecar/generic pointcloud redesign 선행
- sanitizer 오류나 slope 실패를 suppression 또는 threshold 자동 확장으로 숨기는 처리

### 2.3 production seam admission

우선순위는 다음과 같다.

1. 공개 `RuntimeClient`/public contract로 end-to-end lifecycle을 구동한다.
2. private owner test는 이미 존재하는 constructor injection, port, callback, broker,
   cancellation token과 transaction seam을 사용한다.
3. 관측이 불가능한 owner metric은 해당 private owner에 read-only diagnostic snapshot을
   한 개만 추가한다.

새 diagnostic snapshot은 기존 mutex 아래에서 기존 state의 크기/카운터만 복사해야
하며 상태를 변경하거나 lifetime을 연장하면 안 된다. `RuntimeService` 전체 구조를
test가 직접 탐색하는 friend probe, private field offset 접근, installed DTO 추가는
허용하지 않는다. 필요한 최소 후보는 아래 값이다.

```text
RuntimeServiceDiagnostics (private/internal, not installed)
  lifecycle_state
  active_epoch
  public_job_count
  terminal_job_count
  recent_event_count
  subscriber_count
  callbacks_in_flight
  operation_lease_count (기존에 계산 가능할 때만)
```

visualization cache는 `VisualizationProjector::PointCacheEntryCount/Bytes`, resource는
`ResourceGovernor::ReservedMemoryBytes`와 `BoundedExecutor::Snapshot`을 owner별
harness에서 직접 관찰한다. 하나의 새 전역 telemetry owner로 합치지 않는다.

## 3. 현재 fault seam과 관측 가능성

| Owner | 현재 재사용 가능한 seam/증거 | Goal 04에서 추가할 반복 증거 |
|---|---|---|
| `RuntimeService` | `PortFactory`, blocking/interactive/recovery port, Open latch, callback phase gate, Close mode | epoch 교체, terminal job cap, subscriber drain, close 중 open/job/feedback/callback 반복 |
| `PipelineController` | `RuntimePortFixture`, receipt/config receipt 변조, post-commit failure/throw, late-cancel gates | Run/Cancel/Run 장기 반복, event cap/sequence, callback reentrancy 반복 |
| `RuntimeTransaction` / `RuntimeStateStore` | candidate commit, stale base, recovery-required, shared reservation identity | success/failure 교차 반복 후 revision/authority/reservation 불변 |
| `ConfigTransaction` | expected-revision 경쟁, file/state barrier, cleanup recovery fault | apply success/stale/pre-commit failure/post-commit recovery 순환 |
| `OutputRepository` / file transaction | pending file set, backup/temporary cleanup fault | output/tmp/backup/recovery manifest 개수의 반복 안정성 |
| `ResourceGovernor` | per-class reservation counters, admission failure count, heavy phase RAII | cancellation/failure 후 zero, replacement overlap bound, executor idle snapshot |
| `BoundedExecutor` | wait notification, queued cancel, task handle, `Snapshot()` | queue saturation/cancel/destruction 반복과 worker/queue 안정성 |
| `VisualizationProjector` | candidate publish/rollback, stale generation, cache entry/byte metrics | supersession churn, eviction, rollback, epoch replacement 후 cache bound |
| alignment feedback | broker wait/response seam, proposer `PhaseGate` | close/cancel/root replacement 중 feedback 대기와 stale response 반복 |
| plugin host/factory | fixture DSO modes, loader/factory split, cancellation context | load/create/destroy failure와 cancel 반복, DSO handle/fd slope |
| GUI bridge/presentation | point-provider phase gates, generation/epoch presentation tests | headless supersession/host replacement 반복; real driver는 별도 tier |

현재 `RuntimeSnapshot`은 authoritative runtime revision, config revision, active job,
artifact, recent event를 제공하지만 registry의 실제 크기와 resource counter 전체를
공개하지 않는다. 따라서 black-box snapshot만으로 “누수가 없다”고 결론 내리지
않고 process metric과 owner-local metric을 함께 기록한다.

## 4. Target test/harness 구조

```text
open_lmm/test/
├── support/soak/
│   ├── deterministic_scenario.hpp       # phase와 action 계약
│   ├── lifecycle_fixture.hpp            # isolated temp/config/output owner
│   ├── process_metrics_linux.cpp         # /proc self sampler
│   ├── resource_samples.hpp              # sample/report DTO
│   ├── slope_analysis.cpp                # warmup 제외 + robust slope
│   └── watchdog.hpp                      # diagnostic-only deadline
├── soak/
│   ├── contract/
│   │   ├── metrics_sampler_tests.cpp
│   │   └── slope_analysis_tests.cpp
│   ├── runtime_lifecycle_soak.cpp
│   ├── transaction_fault_soak.cpp
│   ├── visualization_soak.cpp
│   ├── plugin_soak.cpp
│   └── gui_headless_soak.cpp
└── fixtures/soak/
    ├── configs/
    ├── plugins/
    └── expected/

scripts/
├── soak/run_soak.sh
├── soak/compare_soak_report.sh
└── ci/run_soak_tests.sh
```

Harness는 scenario 실행, sampling, slope 계산, JSON 작성만 소유한다. Runtime state나
expected revision을 자체 캐시해 authoritative owner처럼 행동하면 안 된다. 각 action
뒤 예상값은 public receipt와 새 `Snapshot()`에서 다시 계산한다.

## 5. Deterministic scenario 계약

각 scenario는 다음 순서를 구현한다.

```text
Arrange → Arm deterministic phase → Start operation → Await entered signal
→ Inject action/fault → Release phase → Await terminal owner signal
→ Query authority → Sample resources → Verify cleanup
```

`WaitFor` timeout은 race를 만드는 수단이 아니라 deadlock watchdog이다. 정상 진행은
반드시 condition variable, promise/future, broker wait callback 또는 event subscription의
명시적 신호로 판정한다. watchdog expiry report에는 scenario, iteration, phase,
runtime/config revision, job state, 마지막 event sequence를 남긴다.

모든 scenario는 정수 seed를 받지만 wall clock으로 동작 순서를 무작위화하지 않는다.
seed는 미리 정의된 action permutation과 agent/config 선택에만 사용하며 동일 binary,
fixture, seed는 동일 schedule을 생성해야 한다.

## 6. Required scenario matrix

### S01 — Open/Close 반복

- fresh output namespace로 Open하고 Snapshot revision 1을 확인한다.
- idle Close, 중복 Close, reopen을 반복한다.
- 실패한/cancelled Open candidate의 output이 Close 반환 전에 제거되는지 확인한다.
- 종료 checkpoint: runtime closed, threads/fds/output dirs baseline 복귀,
  reservations 0, active job/subscriber/callback 0.

### S02 — Run/Cancel/Run

- pre-commit gate에서 cancel: revision과 committed artifact 불변.
- commit 후 receipt 반환 gate에서 late cancel: committed success가 승리.
- 같은 runtime에서 즉시 다음 Run을 성공시킨다.
- queued, active node, feedback wait 각각에서 cancel을 반복한다.
- event sequence는 단조 증가하며 job terminal event는 정확히 하나다.

### S03 — Root replace / epoch isolation

- idle replacement 성공, stale expected revision 거부, busy replacement 거부를 순환한다.
- 이전 epoch job/feedback handle은 새 epoch에서 항상 거부한다.
- replacement가 ready 되기 전 기존 runtime이 계속 authoritative여야 한다.
- failed replacement candidate output/config는 publish되지 않아야 한다.
- job/event retention은 정의된 cap 안에 있고 replacement/Close 후 비워져야 한다.

### S04 — Config apply

- 성공 apply, stale apply, cancel-before-commit, post-commit failure/reconciliation,
  `committed + recovery_required`를 정해진 순서로 반복한다.
- file publication과 runtime revision ordering, affected artifact invalidation을 확인한다.
- recovery-required 상태에서는 unsafe mutation이 거부되고 query/Close/healthy
  replacement만 허용되는지 확인한다.

### S05 — Subscribe/unsubscribe

- subscribe/reset, self-reset, callback 중 reset, callback throw, service보다 오래 사는
  subscription을 반복한다.
- Reset 반환 뒤 새 callback이 시작되지 않고 in-flight callback은 drain되어야 한다.
- slow callback이 runtime mutex를 점유하거나 Close를 영구 차단하면 실패다.
- subscriber ID 증가 자체는 leak로 판정하지 않되 live registry는 baseline으로
  돌아와야 한다.

### S06 — Visualization supersession

- visible → pending → ready → commit 순서로 generation을 반복한다.
- old generation completion, candidate failure/cancel, epoch replacement를 주입한다.
- 마지막 valid presentation은 replacement ready 전까지 유지되어야 한다.
- stale generation은 publish되지 않고 cache entry는 16 이하로 유지된다.
- cache byte는 live entries의 byte 합과 같고 clear/replacement 후 owner-defined
  baseline으로 돌아와야 한다.

### S07 — Plugin failure/cancel

- inspect failure, ABI/kind/capability mismatch, null create, create throw,
  algorithm cancel, successful create/destroy를 반복한다.
- plugin object는 DSO보다 먼저 파괴되고 failed candidate는 factory/runtime에
  설치되지 않아야 한다.
- iteration 전후 fd count와 loaded DSO evidence가 안정적이어야 한다.
- built-in plugin과 synthetic fault DSO를 분리해 어느 owner가 실패했는지 기록한다.

### S08 — File commit/recovery fault

- stage/config/root file commit의 pre-install failure, commit conflict,
  post-install backup cleanup failure를 반복한다.
- commit 전 failure는 이전 파일/state를 보존한다.
- post-commit cleanup failure는 새 파일/state와 recovery manifest를 보존한다.
- output 아래 `.tmp`, backup, recovery manifest, final artifact 수를 매 iteration
  기록하고 각 fault 의미에 맞는 exact cardinality를 검증한다.

### S09 — Shutdown while active

다음 phase 각각에서 `Close(kCancelAndWait)`를 실행한다.

- queued/running stage
- alignment feedback wait
- event callback in flight
- Open candidate build
- visualization candidate build
- resource admission wait

Close는 cancellation을 publish한 뒤 owner thread/callback/lease를 drain하고 반환해야
한다. Close 반환 후 callback, file commit, presentation publish, reservation 증가는
허용하지 않는다. `kRejectIfRunning`은 state를 변경하지 않고 즉시 거부해야 한다.

## 7. Iteration profiles

| Profile | 반복 | 대상 | 기본 lane | timeout |
|---|---:|---|---|---:|
| `contract` | 1–5 | sampler, slope math, report schema | PR L2 | 30초 |
| `fast` | scenario별 100 | synthetic port/작은 filesystem/headless | PR L6 | 5분 |
| `sanitizer` | scenario별 20–50 | ownership/race 위험 subset | sanitizer L5/L6 | 15분 |
| `nightly` | scenario별 1,000 | 전체 headless scenario | nightly L6 | 90분 |
| `external` | 100–1,000 | Goal 03 locked real dataset | external/nightly L6 | 120분 |
| `gpu` | 100 | Iridescence real driver context | gpu/nightly L6 | 60분 |

100회 fast는 전체 scenario를 한 process에서 무조건 연결하지 않는다. 한 process가
모든 owner를 공유하면 leak origin을 알 수 없으므로 scenario family별 executable 또는
subprocess로 격리한다. nightly aggregator가 family report를 하나로 합친다.

## 8. Metric 정의와 sampling

### 8.1 Process metric

Linux runner는 `/proc/self`에서 다음을 읽는다.

- RSS: `VmRSS` 현재값, `VmHWM` process peak
- threads: `/proc/self/task` entry 수
- file descriptors: `/proc/self/fd` entry 수
- CPU time: `clock_gettime(CLOCK_PROCESS_CPUTIME_ID)` 또는 `getrusage`
- wall latency: `steady_clock`

sampler가 여는 `/proc` fd가 자신의 count를 오염시키지 않도록 측정 순서를 고정하고
sampler unit test에서 반복 count가 안정적인지 검증한다. non-Linux는 metric별
`available:false`를 기록할 수 있지만 required Linux CI에서 unavailable은 실패다.

### 8.2 Owner metric

| Metric | Source owner | steady checkpoint expectation |
|---|---|---|
| public/live job registry | `RuntimeService` internal diagnostic | active 0, retained terminal ≤ 256; Close 후 0 |
| recent event registry | `RuntimeService`/Snapshot | ≤ 256; replacement current epoch는 empty |
| subscriptions/callbacks | subscriber registry diagnostic | iteration 종료 0 |
| executor queue/active/waiters | `BoundedExecutor::Snapshot` | 모두 0, worker count 고정 |
| reservations total/by class | `ResourceGovernor` | scenario 정의값; Close/destruction 후 0 |
| visualization cache entries/bytes | `VisualizationProjector` | entries ≤ 16, bytes 합 일치 |
| output/final/tmp/backup/recovery files | isolated output tree | scenario별 exact cardinality |
| cancellation/stage latency | action signal부터 terminal signal | p50/p95/max 기록, watchdog 이내 |

RuntimeService diagnostic가 승인되지 않으면 job/subscriber registry에 대해서는
“직접 관측 불가”를 report에 명시한다. RSS 안정만으로 내부 registry가 bounded라고
대체 결론을 내리지 않는다.

### 8.3 Sampling points

각 iteration은 최소 다음 sample을 남긴다.

```text
before_open, after_open, fault_armed, operation_terminal,
after_close, after_owner_destruction
```

RSS allocator warmup과 DSO lazy loading을 lifecycle leak로 오인하지 않도록 최초
`max(10, iterations / 10)`회는 warmup으로 표시한다. warmup sample은 report에
보존하지만 slope 합격 판정에서는 제외한다.

## 9. Slope 판정 정책

### 9.1 Exact/bounded gate

다음은 calibration 없이 즉시 required gate로 사용한다.

- thread/fd/output-dir live count: warmup 이후 동일 checkpoint에서 증가 slope 0,
  최종값은 baseline allowance 이내
- active task/queued task/waiting submitter/subscriber/callback: iteration 종료 0
- reservation: owner가 없어지는 checkpoint에서 0
- event/job/cache entry: 구현 cap 이하
- revision/event ordering, artifact authority, recovery semantics: exact
- watchdog expiry, sanitizer report, crash, deadlock: 한 번이라도 발생하면 실패

fd와 thread의 baseline allowance는 harness가 항상 보유하는 worker/log fd만
명시적으로 이름 붙여 계산한다. 숫자 `+N`만 하드코딩하지 않는다.

### 9.2 RSS gate

RSS는 allocator와 DSO warmup 때문에 단일 시작/끝 차이만으로 판정하지 않는다.

1. warmup 이후 같은 checkpoint sample에 Theil–Sen bytes/iteration slope를 계산한다.
2. 마지막 25% median과 그 직전 25% median의 차이를 함께 기록한다.
3. 최소 5개의 clean nightly run과 두 compiler에서 calibration한다.
4. calibration 전 RSS 결과는 required evidence이지만 threshold 초과는 warning이다.
5. calibration 후 threshold는 versioned policy JSON에서만 변경하며 CI가 자동으로
   넓힐 수 없다.

초기 leak-alert 후보는 다음과 같고, baseline data 없이 required 기준으로 확정하지
않는다.

```text
abs(Theil–Sen slope) <= 64 KiB / iteration
last-quarter median - previous-quarter median <= max(16 MiB, 2% of plateau RSS)
```

두 조건을 모두 만족해야 안정으로 판정한다. 양의 slope가 반복 재현되면 Goal 05
최적화가 아니라 해당 owner의 retention/lifetime 결함으로 분류한다.

### 9.3 Latency

cancellation/stage latency는 Goal 04에서 hang/endurance 진단용이다. absolute watchdog
초과는 실패지만 성능 regression threshold는 Goal 05가 소유한다. p50/p95/max는
기록하되 calibration 전 PR 성능 gate로 사용하지 않는다.

## 10. Report 계약

`soak_report.schema.json` v1은 closed schema로 unknown field를 거부한다.

```json
{
  "schema_version": 1,
  "run_id": "runtime-nightly-2026-08-20",
  "profile": "nightly",
  "scenario": "root_replace_epoch",
  "iterations": 1000,
  "warmup_iterations": 100,
  "seed": 104729,
  "git": {"commit": "<40 hex>", "dirty": false},
  "build": {
    "compiler": "gcc-12.3",
    "sanitizer": "none",
    "build_type": "RelWithDebInfo"
  },
  "machine": {
    "os": "linux",
    "kernel": "<version>",
    "cpu_count": 16,
    "memory_bytes": 34359738368,
    "container_digest": "sha256:<64 hex>"
  },
  "samples": [],
  "slopes": {},
  "latencies": {},
  "failures": [],
  "result": "pass"
}
```

absolute temp path, callback address, process ID, wall-clock event ordering은 비교
baseline에 포함하지 않는다. failure report에는 재현 가능한 scenario/seed/iteration/
phase와 마지막 authoritative snapshot을 포함한다. 전체 point cloud나 core dump는
기본 report에 embed하지 않고 제한된 별도 CI artifact로 보존한다.

## 11. CTest와 명령

제안 CTest:

```text
open_lmm_soak_metrics_contract_tests
  layer:L2;module:workflows.soak;owner:SoakMetrics;lane:pr

open_lmm_runtime_lifecycle_stress_tests
  layer:L6;module:workflows.soak;owner:RuntimeService;lane:pr

open_lmm_transaction_fault_stress_tests
  layer:L6;module:workflows.soak;owner:FileSetTransaction;lane:pr

open_lmm_runtime_concurrency_stress_tests
  layer:L6;module:workflows.soak;owner:RuntimeService;lane:pr

open_lmm_visualization_stress_tests
  layer:L6;module:workflows.soak;owner:VisualizationProjector;lane:pr

open_lmm_runtime_nightly_soak_tests
  layer:L6;module:workflows.soak;owner:RuntimeWorkflow;lane:nightly

open_lmm_gui_driver_soak_tests
  layer:L6;module:adapters.gui;owner:IridescenceGui;lane:gpu
```

명령 계약:

```bash
scripts/soak/run_soak.sh \
  --build <build-dir> \
  --profile fast \
  --scenario all-headless \
  --iterations 100 \
  --seed 104729 \
  --report <new-report.json>
```

- report와 artifact directory는 새 경로여야 하며 기존 evidence를 덮어쓰지 않는다.
- `--iterations`, `--seed`, `--scenario`는 report에 그대로 기록한다.
- iteration 0, unknown scenario, dirty required-CI run, metric unavailable은 fail closed다.
- local GPU/external data 부재는 exit 77을 허용할 수 있다. configured nightly/GPU
  runner에서 required asset/driver 부재는 성공 skip으로 바꾸지 않는다.

Nightly test는 기본 PR configure에 등록해 전체 CTest 시간을 늘리지 않는다.
`OPEN_LMM_ENABLE_NIGHTLY_SOAK=ON`에서만 등록하고 전용 workflow가 이 option을 켠다.
fast 100회는 PR lane에 등록하되 wall-time이 5분을 넘으면 scenario를 삭제하지 말고
여러 process/job으로 분할한다.

## 12. Sanitizer 전략

### ASan/UBSan

- Open/Close와 root replacement candidate cleanup
- plugin load/create/destroy failure
- visualization candidate/cache supersession
- file transaction rollback/recovery
- callback reset/Close lifetime

각 scenario 20–50회 반복한다. leak sanitizer가 container restriction으로 비활성인
현재 정책은 명시적으로 유지하되 RSS/owner metric이 leak 증거를 대신 완전히
증명한다고 서술하지 않는다.

### TSan

- pre/late cancel barrier
- root replacement 대 Submit/Snapshot/Close
- subscribe/reset 대 dispatch/Close
- feedback response 대 cancel/Close
- bounded executor queue saturation/cancel/destruction
- stale visualization generation publish

TSan suppressions는 third-party의 확인된 noise에만 허용한다. OpenLMM frame이 포함된
새 race는 suppression하지 않는다. sanitizer test 선택은 기존 manifest의
`sanitizer:` label을 계속 사용한다.

## 13. Headless와 real-driver tier

| Tier | 포함 | 제외/조건 |
|---|---|---|
| headless PR/nightly | GUI model, runtime bridge, worker, presentation generation, point provider fault | OpenGL context와 실제 upload 제외 |
| software-driver optional | offscreen context가 공식 지원될 때만 | hardware 결과로 표현하지 않음 |
| real-driver GPU | Iridescence 생성/파괴, upload, supersession, close 반복 | 전용 runner, driver/GPU metadata 필수 |

GPU estimated/upload bytes와 frame performance threshold는 Goal 05 소유다. Goal 04는
context/worker/resource lifetime, stale presentation 차단, shutdown 안전성만 판정한다.

## 14. CI topology

### Required PR

- metric/slope contract unit test
- fast 100회 synthetic lifecycle family
- deterministic fault subset
- 기존 ASan/UBSan·TSan labels에 위험 scenario 추가
- JSON schema validation과 `git diff --check`

기존 required job 이름을 변경하지 않는다. fast suite가 기존 compiler matrix full
CTest에 포함될 경우 총 시간과 evidence XML을 확인한다.

### Nightly

`.github/workflows/nightly-soak.yml`:

- `schedule` + `workflow_dispatch`
- clean container image와 pinned compiler
- `OPEN_LMM_ENABLE_NIGHTLY_SOAK=ON`
- GCC 12 / Clang 15 compiler jobs 병렬 실행, `fail-fast: false`; 각 compiler job 안의
  scenario family는 owner별 process로 격리해 순차 실행하고 family JSON/log를 보존
- report/CTest XML/log를 항상 업로드
- 최소 30일 retention, 실패 seed와 iteration을 summary에 표시
- RSS slope warning과 exact owner failure를 구분해 표시

### External/GPU

Goal 03 locked dataset 또는 real GPU runner가 준비됐을 때만 별도 job을 활성화한다.
asset provenance나 driver metadata가 없는 실행을 representative evidence로 승격하지
않는다.

## 15. 구현 단계

### Phase A — metric/report foundation

1. `/proc` sampler, process metric DTO, report schema 구현
2. Theil–Sen/quarter median 계산과 synthetic slope unit test 구현
3. isolated temp/output owner와 no-overwrite report writer 구현
4. deterministic synchronization helper 재사용 규칙 검사

완료 조건: 실제 runtime 없이 sampler와 slope의 positive/zero/noisy/missing-data
fixture가 deterministic하게 통과한다.

### Phase B — fast lifecycle stress

1. S01, S02, S03, S05, S09 harness 구현
2. RuntimeService internal diagnostic 필요성을 executable gap test로 확인
3. 필요 시 read-only diagnostic snapshot을 별도 작은 production commit으로 추가
4. fast 100회 CTest와 report 생성

완료 조건: 동일 seed 3회 실행에서 authority 결과와 exact owner metric이 동일하다.

### Phase C — owner fault stress

1. S04 config/file recovery
2. S06 visualization supersession/cache
3. S07 plugin failure/cancel/DSO lifetime
4. ResourceGovernor/BoundedExecutor 반복과 sanitizer subset

완료 조건: fault family마다 success/failure/recovery의 state와 cleanup cardinality가
명시적으로 검증된다.

### Phase D — nightly and calibration

1. 1,000회 workflow와 nightly CI 연결
2. GCC/Clang clean runner에서 최소 5회 baseline 수집
3. RSS alert threshold 검토/고정
4. known untestable cases와 machine variance 기록

완료 조건: repeatable command, versioned report, slope result, CI artifact URL이
존재하고 baseline 갱신이 자동 승인되지 않는다.

### Phase E — optional GPU/external

1. real-driver lifecycle harness
2. Goal 03 dataset replay의 반복 mode
3. asset/driver 부재 정책과 failure artifact 연결

이 phase는 hardware/data availability 때문에 Goal 04 headless 완료와 별도 상태로
기록할 수 있지만, known untestable 목록에서 숨기면 안 된다.

## 16. 검증 체크리스트

- [x] 구현된 scenario가 explicit phase signal을 사용하고 고정 sleep이 없다.
- [x] fast 100회와 nightly 1,000회 명령이 같은 7개 headless family 계약을 사용한다.
- [x] Open/Close, Run/Cancel/Run, root replace, config apply가 반복된다.
- [x] subscribe/reset, self-reset, callback throw, in-flight reset/Close와 service보다
  오래 사는 subscription이 반복된다.
- [x] visualization stale generation, rollback, cancellation, cache cap/clear가 반복된다.
- [x] plugin failure/cancel과 DSO destruction ordering이 반복 검증된다. 로드된
  synthetic plugin algorithm이 deterministic safe point에서 host cancellation token을
  관찰하며 object destroy 뒤 DSO mapping이 0으로 복귀한다.
- [x] file rollback 및 committed recovery-required가 구분된다.
- [x] running/feedback/callback/open/visualization candidate/resource-admission wait 중
  shutdown이 모두 포함된다. Close의 실제 `kClosing` 진입은 private read-only lifecycle
  diagnostic wait로 동기화하고, lease/cancellation drain 뒤 closed 상태를 확인한다.
- [x] RSS/thread/fd 및 구현된 runtime owner metric sample이 report에 존재한다.
- [x] reservation/cache/output cardinality가 owner checkpoint와 연결된다.
- [x] fast lifecycle의 ASan/UBSan와 TSan subset이 manifest label로 등록된다.
- [x] headless와 real-driver 결과가 별도 tier로 기록된다. (real-driver는
  `NOT_AVAILABLE`)
- [x] full PR CTest, package/public boundary, architecture policy가 유지된다.
- [x] installed public API/ABI와 plugin ABI가 바뀌지 않는다. RuntimeService의 새
  read-only diagnostics는 private `src/` header에만 존재한다.
- [x] report/baseline이 자동 덮어쓰기 또는 자동 threshold 확대를 하지 않는다.

## 17. Known untestable / deferred cases

구현 결과 문서는 최소 다음을 판정해 기록해야 한다.

- process crash/power loss 중 file durability는 journal/fsync 정책이 없어 Goal 04의
  in-process fault seam으로 완전 검증할 수 없다.
- process-wide allocation failure의 엄격한 동작은 보장되지 않는다.
- plugin-private allocator/GPU driver memory는 `ResourceGovernor` counter에 포함되지
  않는다. process RSS/GPU runner metric과 별도 표기한다.
- allocator가 RSS를 OS에 반환하지 않는 현상과 실제 unreachable leak는 RSS 하나로
  구분할 수 없다. sanitizer, owner counter, slope를 함께 사용한다.
- external dataset은 Goal 03 provenance/redistribution 조건을 충족한 bundle이 있어야
  required/nightly evidence가 된다.
- real-driver GUI/GPU는 현재 일반 CI 환경에서 증명할 수 없다.
- hard RSS budget과 stage 성능 regression threshold는 Goal 05가 소유한다.

## 18. 최종 산출물

구현 완료 시 이 문서를 implementation result로 갱신하고 다음 evidence를 연결한다.

```text
docs/post_freeze_results/04_soak_fault_stress.md
docs/post_freeze_results/soak_baselines/*.json
open_lmm/test/soak/**
open_lmm/test/support/soak/**
scripts/soak/**
scripts/ci/run_soak_tests.sh
.github/workflows/nightly-soak.yml
```

결론은 `PASS`, `FAIL`, `NOT_AVAILABLE`을 scenario/tier별로 분리한다. headless fast
stress가 통과했다는 이유로 real dataset, real GPU, process-crash durability까지
검증됐다고 표현하지 않는다.

## 19. 구현 진행 증거 — 2026-08-20 UTC

현재 구현된 headless 범위는 다음과 같다.

- Linux `/proc/self` RSS/HWM/thread/fd와 `getrusage` CPU sampler
- deterministic Theil–Sen slope, quarter-median delta, warmup 제외
- closed JSON Schema v1, closed bundle schema 및 중첩 scalar/type까지 검사하는 runtime
  validator
- `O_EXCL` 기반 no-overwrite JSON evidence writer
- 같은 `RuntimeService`를 반복 사용하는 lifecycle stress와 별도 격리된 runtime
  concurrency stress
- Open/Run/subscribe/reset/Close, retired epoch handle 거부, pre-commit cancel, 즉시 retry,
  commit 후 receipt 반환 gate의 late cancel committed-success 승리, 취소된 active job의
  Close drain, 중복 Close
- idle/stale/busy root replacement, feedback/Open/callback 중 Close, self/in-flight reset,
  callback exception과 subscription-outlives-service, 실패한 replacement와 취소된 Open의
  unpublished candidate output rollback
- config success/stale/cancel/committed-recovery-required, affected artifact invalidation,
  file rollback/backup/manifest
- visualization supersession/rollback/cancel/cache cap, plugin ABI/create/algorithm
  cancellation/DSO lifetime, built-in ScanContext metadata/unload와 synthetic fault DSO의
  분리 검증, resource admission/queued cancel/RAII release
- visualization candidate build 및 resource admission wait 중 실제 RuntimeService Close와
  operation lease/cancellation drain
- `ResourceGovernor` reservation과 `BoundedExecutor` queue/active/waiter idle 검증
- fast/1,000회 nightly 공통 7-family runner, bundle aggregator, family log 보존,
  opt-in nightly CTest와 예약 workflow

로컬 검증 결과:

| 범위 | 결과 | 비고 |
|---|---|---|
| metric/report contract | `PASS` | closed nested field 및 no-overwrite 포함 |
| full default CTest | `PASS` | 로컬 dirty worktree build, 72/72 |
| all-headless fast 100회 | `PASS` | 7 family, 1,202 sample/checkpoint |
| CI evidence wrapper fast 100회 | `PASS` | closed bundle + family JSON/log + JUnit |
| runtime concurrency 1,000회 × 3 | `PASS` | lifecycle diagnostic phase gate 적용 후 |
| opt-in nightly 1,000회 CTest | `PASS` | 최신 구현 7/7, 94.58초 local dirty-worktree build; output-directory 보완 뒤 transaction 1,000회 재통과 |
| GCC 12 ASan/UBSan | `PASS` | 별도 build dir, manifest 선택 64/64; 최신 L6 7/7은 sanitizer profile 50회 |
| Clang 15 TSan | `PASS` | 별도 build dir, manifest 선택 22/22; 최신 L6 5/5는 sanitizer profile 50회, Linux ASLR off |
| architecture/release policy | `PASS` | direct spdlog/fmt 및 frozen boundary 포함 |
| RSS hard threshold | `NOT_AVAILABLE` | clean compiler 5-run calibration 전 warning evidence만 기록 |
| external/GPU tier | `NOT_AVAILABLE` | dataset provenance/real driver runner 필요 |

이 결과는 현재 dirty worktree에서 수행했으므로 release evidence나 clean compiler
baseline으로 승격하지 않는다. `.github/workflows/nightly-soak.yml`은 clean checkout,
container digest, 30일 artifact retention을 요구하며 실제 원격 실행 URL은 아직 없다.
현재 로컬 nightly RSS 결과는 threshold를 강제하지 않는 evidence다. 최종 fast evidence의
대표 lifecycle run은 Theil–Sen 약 `11.9 KiB/iteration`, quarter-median delta
`264 KiB`였으며,
이는 초기 alert 후보보다 작지만 clean two-compiler 5-run calibration을 대체하지 않는다.

TSan 기본 실행은 이 컨테이너에서 테스트 본문 진입 전 모든 binary가
`unexpected memory mapping`으로 종료됐다. `setarch -R`로 TSan shadow-memory와 ASLR
mapping 충돌을 제거한 뒤 21개는 한 번에 통과했고, 기본 60초 제한을 넘긴
`open_lmm_runtime_service_tests`도 180초 제한에서 34.56초로 통과했다. CI sanitizer
script는 지원되는 Linux runner에서 같은 ASLR 우회와 TSan 전용 180초 제한을
자동 적용한다. 이 환경 실패를 race suppression으로 처리하지 않았다.

Clang sanitizer build는 `visualization_stress.cpp`가 GCC 확장으로 허용되던 C++17
structured-binding 직접 lambda capture를 사용한 사실도 검출했다. 캡처 전 명시적
값 복사로 수정했으며 GCC/Clang 양쪽 sanitizer build가 같은 소스를 컴파일한다.

### 19.1 구현된 report family

| Scenario | fast/nightly | 핵심 exact gate |
|---|---|---|
| `all-headless` lifecycle | 100 / 1,000 | revision, late/pre-cancel 의미, jobs/events, 중복 Close idle |
| `runtime-concurrency` | 100 / 1,000 | root epoch, failed/cancelled candidate rollback, callback barrier, feedback/Open/visualization/resource wait/Close |
| `transaction-fault` | 100 / 1,000 | final/tmp/backup/manifest/output-directory cardinality |
| `visualization-supersession` | 100 / 1,000 | stale reject, cache entries ≤ 16, clear=0 |
| `plugin-failure-lifetime` | 100 / 1,000 | built-in inspect, algorithm cancel, failed DSO mapping=0, destroy-before-unload |
| `resource-admission` | 100 / 1,000 | reservation=0, queue/active/waiter=0 |
| `config-apply-recovery` | 100 / 1,000 | file/state revision, health/manifest, mutation gate |

Thread/fd 판정은 warmup 뒤 Theil–Sen slope가 정확히 0이고 마지막 checkpoint가
warmup baseline으로 복귀해야 한다. 짧게 관측되는 중간 thread sample은 JSON에
보존하되 누수로 오판하지 않는다. RSS는 slope/quarter median을 항상 기록하지만
calibration 전 hard gate로 쓰지 않는다.

`--profile gpu --scenario gpu-real-driver`는 headless runner로 대체 실행하지 않고 현재
명시적으로 exit 77을 반환한다. 향후 `open_lmm_gui_driver_soak_tests`가 구성된 전용
runner에서만 같은 no-overwrite report 계약으로 실행된다.

## 20. Goal 04 완료 감사

원본 Goal 04의 필수 범위를 구현 및 실행 증거에 다시 대조한 결과는 다음과 같다.

| 요구사항 | 판정 | 실행/구현 증거 |
|---|---|---|
| 9개 required scenario | `PASS` | 7개 owner-isolated family가 S01–S09를 모두 포함하고 fast 100회 및 nightly 1,000회 통과 |
| RSS/thread/fd slope | `PASS` | 모든 family report에 warmup 제외 Theil–Sen/quarter median 존재; 최종 lifecycle thread/fd slope 0 |
| job/event/reservation/cache/output/latency metric | `PASS` | closed owner schema, `RuntimeServiceDiagnostics`, governor/executor/projector/file cardinality, stage/cancel/late-cancel/Close latency |
| output dir count | `PASS` | `output_directory_count`를 closed schema에 추가; recovery 1, cleanup 0을 100회 및 1,000회 exact 검증 |
| deterministic harness/no sleep race | `PASS` | event/phase gate/future/broker 신호 사용; soak source의 sleep/yield 정적 검색 0 |
| fast/nightly 분리와 반복 명령 | `PASS` | `run_soak.sh`, `run_soak_tests.sh`, opt-in nightly CTest/workflow |
| sanitizer 반복 subset | `PASS` | ASan/UBSan L6 7개와 TSan L6 5개가 각각 50회; 전체 manifest도 64/64, 22/22 |
| headless/real-driver 분리 | `PASS` | headless required lane 구현; GPU real-driver 요청은 대체 실행 없이 exit 77 |
| slope 결과/known untestable | `PASS` | report에 slope 보존; hard RSS calibration, external dataset, GPU, crash durability는 별도 한계로 명시 |

따라서 Goal 04의 headless endurance infrastructure 완료 조건은 `PASS`다. clean
two-compiler 5-run RSS calibration, external dataset, real-driver GPU는 원본 완료 조건이
요구하는 known-untestable/deferred tier로 유지하며 headless 결과로 대체 주장하지 않는다.
