# Goal 05 개발 명세서: Benchmark + Resource Observability

- 상태: **COMPLETE — HEADLESS IMMUTABLE-IMAGE CALIBRATION APPROVED**
- 작성일: 2026-08-20 UTC (완료 증거 갱신: 2026-08-25 UTC)
- 구현·calibration 기준 source: `b168b3db89096ca7010afc15707abb223bcd0c71`
- 동결 기준: `59e003ebc4b7d44597ced4ddab3436adec310370`
- 상위 목표: `docs/pose_freeze_goals/open_lmm_post_freeze_goals/05_benchmark_resource_observability_goal.md`

## 0. 완료 판정

Goal 05의 필수 headless 범위는 완료됐다. 동일한 clean source와 `medium-v1` fixture를
사용해 GCC 12와 Clang 15에서 각각 독립 컨테이너 5회, 각 run마다 warmup 2회와
measured 10회를 수행했다. P01–P10의 public/owner bundle과 P04/P05 paired semantic
parity를 모두 검증한 뒤 16-entry canonical catalog를 수동 승인했다.

| 항목 | 승인 결과 |
|---|---|
| Source | `b168b3db89096ca7010afc15707abb223bcd0c71`, clean worktree |
| Base immutable image | `hwan0806/open-lmm@sha256:aabcc53791995ce4ddf9606f9710cbffa730bb555646f01b61844c7c6724eb6c` |
| GCC 12 승인 run | `2,3,4,5,6`; 각 17 bundles, 170 measured/owner reports, parity 10/10 |
| Clang 15 image | base digest에서 `docker/open_lmm.benchmark-clang15.Dockerfile`로 파생한 local digest `sha256:b546cb17dd9ea8a7f2d7c93e4bd86abafff83d5653d0f9085f01fb404b8e0fc6` |
| Clang 15 승인 run | `1,2,3,5,7`; 각 17 bundles, 170 measured/owner reports, parity 10/10 |
| Machine contract | Intel Core Ultra 9 285K, report CPU count 24, enforced affinity `0-7`, memory class `16-31GiB` |
| Canonical baseline | `docs/post_freeze_results/performance_baseline.json`, SHA-256 `6b8c07ca844c49567a35900621004dabbf15a266c70e6dc676ef6feb360dbca2` |
| Review record | `goal05-immutable-calibration-review-b168b3d.json`, SHA-256 `86915f37c301ba01e6c6009c033da4f0747f55b85de25f6b720e6363a386c379` |
| Independent comparison | selected source reports를 canonical catalog로 재-aggregate, `16/16 PASS`; validation set hash `af749c3643bf32da6d813ff73d646ae06e12beb2b32a95b3316bc12a1d33aee1` |
| Sanitizers | fresh ASan/UBSan `71/71`, TSan `22/22` PASS |
| Optional tiers | `large-external-v1`, real-driver GPU는 `NOT_AVAILABLE`; 필수 headless 완료를 막지 않음 |

Baseline key의 `cpu_count=24`는 `sysconf(_SC_NPROCESSORS_ONLN)` 값이고, 실제 캘리브레이션
CPU 계약은 raw report의 `cpu_affinity=0-7` 및 실행 명령의 Docker cpuset으로 추가 고정했다.
현 v1 key가 affinity를 직접 포함하지 않는 한계는 문서화하되, 이미 수집한 key/schema를
소급 변경하지 않는다.

## 1. 목적과 완료 정의

Goal 05의 목적은 성능 개선 자체가 아니라 현재 runtime의 시간·메모리·I/O·resource
ownership을 재현 가능한 숫자로 만드는 것이다. 측정 전에 streaming PCD decode,
sidecar, generic point-cloud representation, cache 구조 변경을 시작하지 않는다.

완료 시 다음이 모두 존재해야 한다.

1. Open, DataLoad, Alignment, sequential/parallel MapUpdate, Save fallback,
   final-map visualization cold/warm, full pipeline을 실행하는 대표 명령
2. small/medium/large fixture의 provenance 및 사용 lane 정책
3. wall/CPU time, sampled peak RSS, process high-water RSS, disk I/O 시계열
4. governor class별 reservation과 process RSS를 섞지 않은 owner metric
5. retained/output bytes와 target-window transient peak를 구분한 결과
6. points/decoded bytes, visualization cache hit/miss/bytes, CPU/GPU transfer estimate
7. cancellation latency와 stage latency의 raw sample 및 요약 통계
8. machine/build/container/fixture metadata가 포함된 closed JSON report
9. 반복 측정으로 승인된 versioned baseline과 수동 threshold 갱신 절차
10. PR benchmark와 nightly/external/GPU benchmark의 분리

`post_freeze_results/performance_baseline.json`은 측정 전 임의 숫자로 만들지 않는다.
최소 calibration 조건을 만족한 결과만 수동 review를 거쳐 생성한다.

## 2. 현재 코드 기준 관측 가능성

### 2.1 재사용 가능한 기반

| 기반 | 현재 제공하는 값 | Goal 05 사용 방식 |
|---|---|---|
| Goal 04 process sampler | `VmRSS`, `VmHWM`, thread/fd, process CPU time, machine metadata | 공통 support로 재사용하고 benchmark 전용 I/O/window sampler를 추가한다. |
| Goal 04 report writer | closed validation, `O_EXCL` no-overwrite, dirty/commit metadata | 동일 fail-closed/no-overwrite 원칙을 유지하되 별도 benchmark schema를 사용한다. |
| Goal 03 replay contract | locked input hash, case manifest, external provenance | large/external fixture의 입력 잠금과 full-pipeline correctness preflight에 사용한다. |
| `ResourceGovernor` | total/class별 reservation, admission failure, executor snapshot | resident/transient/heavy-map owner metric으로 직접 기록한다. |
| `RuntimeServiceDiagnostics` | lifecycle/epoch/job/event/subscriber/callback/lease | benchmark 종료 후 owner가 idle인지 검증한다. 성능 숫자의 대체값으로 쓰지 않는다. |
| `VisualizationProjector` | cache entry count/bytes | cache byte owner metric으로 기록하고 hit/miss/eviction 진단을 보강한다. |
| execution events | stage start/completion/failure, progress, cancellation terminal event | public façade stage latency와 cancellation latency의 authoritative boundary로 사용한다. |
| Tracy/timing log | node zone, selected domain algorithm timer, `[PROFILE]` node log | 개발자 상세 profile 보조 수단으로만 사용하고 JSON baseline source로 parse하지 않는다. |

### 2.2 착수 시 gap과 현재 해소 상태

| ID | 착수 시 gap | 현재 상태 |
|---|---|---|
| B5-01 | target stage RSS peak window 부재 | **해소:** background sampler와 stage event window 구현 |
| B5-02 | `/proc/self/io` logical/physical counter 부재 | **해소:** benchmark process sampler에 구현 |
| B5-03 | governor class/failure/executor 통합 snapshot 부재 | **해소:** private `ResourceGovernorDiagnostics` 구현 |
| B5-04 | projector hit/miss/eviction 관측 부재 | **해소:** private projector diagnostics 구현 |
| B5-05 | public façade에서 owner metric을 노출할 수 없음 | **해소:** public workflow와 private owner runner를 분리하고 pair key로 결합 |
| B5-06 | exact GPU resident bytes 관측 불가 | **optional 유지:** CPU staging/upload estimate와 `not_available` reason을 기록; real-driver tier와 혼동하지 않음 |
| B5-07 | 승인 baseline과 threshold 부재 | **해소:** immutable medium-v1 5-run을 검토해 16-entry canonical catalog 승인 |

## 3. 아키텍처 범위와 guard

### 3.1 허용 범위

- `open_lmm/test/benchmark/**`
- `open_lmm/test/support/benchmark/**`
- Goal 03 fixture/input-lock support와 Goal 04 process/machine support 재사용
- `open_lmm/test/cmake/registrations/benchmark.cmake`
- `scripts/benchmark/**`, `scripts/ci/run_benchmark_tests.sh`
- `.github/workflows/nightly-benchmark.yml`
- private owner의 read-only diagnostics snapshot
- benchmark report/baseline JSON schema와 validator

### 3.2 금지 범위

- installed `RuntimeClient`, public DTO, plugin ABI에 benchmark field 추가
- benchmark를 위해 `RuntimeStateStore`와 별도 runtime state/telemetry owner 도입
- production global test flag, environment-driven algorithm branch, hidden singleton hook
- 측정 전에 DataLoad/MapUpdate/Save/visualization data flow를 streaming으로 redesign
- output point cap, voxel policy, plugin algorithm, parallelism default 변경
- benchmark가 correctness commit barrier나 cancellation semantics를 우회하는 직접 호출
- noisy 결과에 맞춘 threshold 자동 확대 또는 baseline 자동 rewrite
- Tracy/log text를 안정적인 machine-readable API로 간주

### 3.3 invariant 영향

| Invariant | 보존 방법 |
|---|---|
| committed state single owner | benchmark expected state는 receipt와 `Snapshot()`에서 매번 재조회한다. |
| candidate/commit barrier | public runner는 `RuntimeClient` command만 사용한다. private runner도 canonical executor/transaction을 사용한다. |
| late cancellation | cancel 시각과 terminal event를 측정하되 committed-success 우선 의미를 변경하지 않는다. |
| epoch isolation | scenario마다 fresh process/runtime를 사용하고 old handle을 재사용하지 않는다. |
| presentation continuity | warm/cold visualization 측정이 clear/rebuild를 삽입하지 않는다. |
| dependency direction | public runner는 `open_lmm_client`만 링크하고 owner runner만 private canonical target을 소비한다. |
| resource ownership | process RSS와 governor estimate를 다른 field/단위로 기록한다. |
| API/ABI | 새 diagnostics는 `src/` private header에만 둔다. |

## 4. Benchmark topology

```text
fixture generator / locked external bundle
                   │
                   ├── public workflow runner ── RuntimeClient
                   │       │
                   │       ├── execution-event stage window
                   │       ├── process RSS/CPU/I/O sampler
                   │       └── output/correctness digest
                   │
                   └── private owner runner
                           │
                           ├── ResourceGovernor diagnostics
                           ├── DataLoad/MapUpdate/Save canonical owner
                           └── VisualizationProjector diagnostics

raw run reports ── validator ── aggregator ── baseline comparator
```

두 runner의 역할은 다르다.

- public workflow runner는 제품 사용 경로의 wall/CPU/RSS/I/O/stage 결과를 측정한다.
- private owner runner는 reservation/cache/decoded/retained byte ownership을 측정한다.
- private runner 숫자를 public workflow latency로 대체하지 않는다.
- 동일 fixture/config/hash와 scenario ID로 두 report를 join한다.

### 4.1 target 구조

```text
open_lmm/test/
├── support/benchmark/
│   ├── benchmark_options.hpp/.cpp
│   ├── benchmark_report.hpp/.cpp
│   ├── benchmark_bundle.hpp/.cpp
│   ├── benchmark_pair.hpp/.cpp
│   ├── benchmark_statistics.hpp/.cpp
│   ├── process_window_sampler.hpp/.cpp
│   ├── fixture_manifest.hpp/.cpp
│   └── stage_event_recorder.hpp/.cpp
├── benchmark/
│   ├── contract/
│   │   ├── report_contract_tests.cpp
│   │   ├── statistics_tests.cpp
│   │   ├── process_sampler_tests.cpp
│   │   └── fixture_policy_tests.cpp
│   ├── workflow/benchmark_runner.cpp
│   ├── workflow/orchestrator_tests.cmake
│   ├── owners/resource_owner_runner.cpp
│   ├── fixtures/generate_fixture.cpp
│   ├── schema/performance_report.schema.json
│   ├── schema/performance_bundle.schema.json
│   ├── schema/performance_pair.schema.json
│   ├── benchmark_aggregate.cpp
│   └── benchmark_pair.cpp
scripts/
├── benchmark/run_benchmarks.sh
└── ci/run_benchmark_tests.sh
```

Python comparator를 채택할 경우 표준 라이브러리만 사용하고 report 생성/validation의
canonical 구현은 C++에 둔다. Python이 production artifact를 해석하는 두 번째 owner가
되어서는 안 된다.

## 5. Fixture 정책

### 5.1 공통 manifest

모든 fixture는 다음 정보를 closed manifest에 가진다.

```text
fixture_id, fixture_version, generator_version, seed
source_kind = generated | locked_external
agent_count, scans_per_agent, total_scan_count
points_per_scan, decoded_point_count, sizeof_point
decoded_point_bytes, pose_count, on_disk_bytes
config files + SHA-256, input files + SHA-256
plugin IDs/capabilities, voxel sizes, parallelism/resource budget
license/provenance/redistribution for external fixtures
```

`decoded_point_bytes`는 `point_count * sizeof(pcl::PointXYZI)`를 runner toolchain에서
계산한다. compressed file size를 decoded bytes로 사용하지 않는다. manifest의 합계와
실제 decode 관측값이 다르면 benchmark를 실행하지 않는다.

### 5.2 size class

| Class | 초기 deterministic shape | Lane | 목적 |
|---|---|---|---|
| `small-v1` | 2 agents × 8 scans × 4,096 points = 65,536 points | PR | command/report 계약, 작은 regression 신호 |
| `medium-v1` | 2 agents × 64 scans × 32,768 points = 4,194,304 points | nightly | DataLoad/MapUpdate/Save/viz peak가 sampler interval보다 충분히 길도록 함 |
| `large-external-v1` | 최소 20M decoded points 또는 2 GiB decoded bytes의 locked Goal 03 bundle | external/nightly | 실제 large-data shape와 allocator/I/O 영향 |

초기 크기는 versioned fixture ID의 일부다. 실행 시간을 맞추기 위해 기존 ID의 크기를
조용히 변경하지 않는다. 변경은 `small-v2`처럼 새 fixture와 baseline으로 추가한다.

Generated fixture는 fixed seed로 ground/vertical structure/overlap/outlier/intensity를
만들고 pose 및 agent order를 canonical sort한다. wall clock, PID, `random_device`, temp
path는 point 값과 manifest hash에 영향을 주면 안 된다.

### 5.3 correctness preflight

성능 측정 전 각 fixture는 다음을 통과해야 한다.

- input-lock SHA-256 및 path traversal/symlink policy
- canonical config validation 및 plugin preflight
- expected agent/frame/point count
- stage artifact readiness와 output digest
- sequential/parallel MapUpdate 결과의 기존 tolerance 계약

correctness preflight 실패는 성능 regression이 아니라 benchmark `invalid`다.

## 6. Scenario 계약

각 raw measurement process는 정확히 하나의 target scenario를 측정한다. prerequisite는
동일 process에서 실행할 수 있지만 target window 밖으로 표시한다. process reuse로
인한 allocator/cache 교차 오염을 피하기 위해 반복마다 fresh process를 사용한다.

| ID | Target | Arrange | 측정 window / exact 확인 |
|---|---|---|---|
| P01 | Runtime Open | config/input preflight 완료 | `Open` 호출부터 ready snapshot; output namespace와 revision 확인 |
| P02 | DataLoad | Open 완료 | DataLoad stage start→completion; retained reservation, decoded/filtered points |
| P03 | Alignment | DataLoad committed | Alignment start→completion; algorithm timing, descriptor/constraint/pose counts |
| P04 | MapUpdate sequential | Alignment committed, parallel=false | stage window, heavy reservation, output PCD bytes/digest |
| P05 | MapUpdate parallel | 동일 state, parallel=true | P04와 같은 metric + worker/concurrency; output parity 확인 |
| P06 | Save fallback | GlobalMap/PcdFile가 ready가 아닌 optimized state | fallback node start→file commit; map assembly peak와 written bytes |
| P07 | Final visualization cold | final map committed, projector cache miss | query start→snapshot ready; decode points/cache bytes/DTO bytes |
| P08 | Repeated visualization warm | P07 cache ready | 같은 key 10회; hit/miss, 반환 copy bytes, p50/p95/max |
| P09 | Full pipeline | fresh Open | RunAll submit→terminal; stage별 및 total metric, final artifact digest |
| P10 | Cancellation | target stage deterministic safe point | cancel request→terminal owner signal; revision/artifact authority 확인 |

### 6.1 sequential/parallel 비교

P04/P05는 같은 fixture, config fingerprint, CPU budget, plugin capability, output voxel을
사용한다. 비교 report에는 다음을 함께 기록한다.

- wall/CPU time과 speedup ratio
- peak RSS 및 governor peak by class
- executor maximum active/queued tasks
- output bytes, point count, fingerprint/tolerance verdict
- cancellation latency는 별도 실행하며 정상 timing sample에 섞지 않는다.

### 6.2 Save fallback 강제 조건

benchmark용 production flag를 추가하지 않는다. canonical command 순서로 DataLoad와
Alignment만 ready로 만든 뒤 Save stage를 실행해 기존 fallback predicate를 만족시킨다.
MapUpdate output을 삭제하거나 artifact state를 직접 변조하지 않는다.

### 6.3 visualization cold/warm

- process-cold와 projector-cold는 구분한다.
- OS page cache는 shared CI에서 안전하게 drop할 수 없으므로
  `os_page_cache = uncontrolled`로 기록한다.
- cold는 새 projector/epoch의 첫 query다.
- warm은 같은 revision/agent/phase/voxel key를 반복한다.
- warm hit에서도 public snapshot point vector copy가 발생하므로 cache hit와 transfer
  bytes를 모두 기록한다.

## 7. Metric 정의

### 7.1 시간

| Metric | 정의 |
|---|---|
| `wall_time_ns` | target start/end `steady_clock` 차이 |
| `cpu_time_ns` | `CLOCK_PROCESS_CPUTIME_ID` target delta; 모든 process thread 포함 |
| `stage_latency_ns` | subscribed `kStageStarted` 수신부터 terminal stage event 수신까지 |
| `command_latency_ns` | public call/submit부터 `Wait` terminal까지 |
| `cancellation_latency_ns` | `Cancel` request 직전 timestamp부터 terminal event까지 |

execution event sequence와 stage ID가 기대와 다르면 latency sample을 만들지 않고
scenario를 실패시킨다. log timestamp는 baseline에 사용하지 않는다.

### 7.2 process memory

| Metric | 의미 |
|---|---|
| `rss_start_bytes` / `rss_end_bytes` | target window 경계의 `VmRSS` |
| `sampled_peak_rss_bytes` | target window sampler가 본 최대 `VmRSS` |
| `process_hwm_bytes` | process 전체 `VmHWM`; prerequisite를 포함할 수 있음 |
| `target_peak_delta_bytes` | `max(0, sampled_peak_rss - rss_start)` |
| `retained_rss_delta_bytes` | target 완료/allocator settle을 기다리지 않은 즉시 end-start delta |

`retained_rss_delta_bytes`는 감소 시 음수가 되는 signed raw diagnostic이다.
v1 aggregate/baseline summary는 non-negative metric contract이므로 이 값은 raw
report에만 보존하고 regression metric으로 승격하지 않는다. 양수인 실행만 골라
aggregate하면 allocator timing에 따라 metric availability가 달라지므로 금지한다.

sampler는 기본 1 ms 간격으로 observation만 수행한다. 이 timer는 race orchestration에
사용하지 않으며 operation 완료는 event/condition/future로만 판단한다. target이 20 ms
미만이면 sampled peak 신뢰도를 `low`로 표시하고 memory baseline에는 medium fixture를
사용한다.

`VmHWM`은 process lifetime cumulative 값이므로 stage-specific peak라고 부르지 않는다.
allocator가 RSS를 OS에 반환하지 않는 현상도 leak/retained owner로 단정하지 않는다.

### 7.3 owner memory

```text
governor_reserved_total_bytes
governor_resident_payload_bytes
governor_transient_task_bytes
governor_heavy_map_bytes
governor_admission_failures
executor worker/queue/active/waiter maxima
visualization_cache_entries/bytes/hits/misses/evictions
returned_snapshot_bytes
cpu_staging_bytes
gpu_upload_requested_bytes
gpu_estimated_resident_bytes
```

`governor_*`는 admission estimate/owned reservation이고 RSS가 아니다.
`target_peak_delta_bytes`는 sampled process 증가량이고 allocator attribution이 아니다.
두 값을 같은 `memory_bytes` 하나로 합치거나 차이를 “untracked leak”로 단정하지 않는다.

### 7.4 points와 decoded bytes

- input compressed bytes 및 logical file bytes
- decoded source points/bytes
- retained filtered points/capacity bytes
- optimized pose count
- MapUpdate output points/file bytes
- visualization source/displayed points
- cache entry points/bytes
- public DTO point bytes
- GUI XYZ/color staging estimate

capacity 기반 retained bytes와 size 기반 logical bytes를 별도 field로 둔다.

### 7.5 disk I/O

Linux `/proc/self/io`에서 target delta를 기록한다.

```text
rchar, wchar, syscr, syscw, read_bytes, write_bytes, cancelled_write_bytes
```

`rchar/wchar`는 logical I/O, `read_bytes/write_bytes`는 kernel storage accounting이다.
page cache 때문에 physical read가 0이어도 invalid가 아니다. output directory의 실제
file size와 write counter를 별도로 보존한다.

### 7.6 bounded output 대 bounded peak

각 P02/P04/P05/P06/P07 report는 다음 비교를 반드시 포함한다.

```text
logical_output_bytes
retained_owner_bytes
sampled_target_peak_delta_bytes
process_hwm_bytes
peak_to_output_ratio
peak_to_retained_ratio
```

ratio denominator가 0이면 `null`이다. bounded output 또는 cache entry cap만으로 peak가
bounded라고 판정하지 않는다.

## 8. 최소 production diagnostics

### 8.1 ResourceGovernor

private `src/runtime/resources/resource_governor.hpp`에 read-only snapshot을 추가한다.

```text
ResourceGovernorDiagnostics
  budget
  reserved_total_bytes
  reserved_by_class[3]
  admission_failures
  heavy_phase_active
  executor_snapshot
```

snapshot은 counter acquire-load와 기존 mutex 아래의 `heavy_phase_active` 복사만 한다.
reservation lifetime을 늘리거나 admission ordering을 변경하지 않는다. installed header로
이동하지 않는다.

### 8.2 VisualizationProjector

private diagnostics에 다음 cumulative counter를 추가한다.

```text
entries, bytes, hits, misses, insertions, evictions, clears
```

기존 projector mutex 아래에서 갱신한다. cancelled/stale candidate는 insertion으로
계산하지 않는다. counter는 관측값이며 eviction 정책을 변경하지 않는다.

### 8.3 stage event recorder

새 production event type이나 timestamp field를 public ABI에 추가하지 않는다.
benchmark subscriber가 callback 진입 시 `steady_clock` timestamp와 existing sequence를
기록한다. callback은 lock-free queue 또는 짧은 mutex append만 수행하고 파일 I/O와 JSON
serialization을 하지 않는다.

### 8.4 algorithm profiler

`AlgorithmExecutionTimer`와 `AlgorithmProfiler`는 이미 존재하지만 normal runtime의
`MakeAlgorithmExecutionContext`는 profiler를 주입하지 않는다. 첫 구현에서는 이를
위해 public/global sink를 추가하지 않는다. private owner runner가 직접 canonical
algorithm context를 만들 때만 profiler callback을 주입한다. 전체 workflow baseline은
stage event timing을 authoritative 값으로 사용한다.

## 9. 반복과 통계

| Profile | Fixture | Warmup / measured | Scenario | 기본 판정 |
|---|---|---:|---|---|
| `contract` | synthetic scalar | 0 / 1–5 | schema/statistics/sampler | exact |
| `pr` | small-v1 | 1 / 5 | P01, P02, P04, P07, P09 | correctness + calibrated stable metric |
| `nightly` | medium-v1 | 2 / 10 | P01–P10 | full summary + regression |
| `external` | large-external-v1 | 1 / 5 | P02–P09 | provenance-required |
| `gpu` | medium/large | 2 / 10 | P07/P08 real driver | dedicated runner |

각 measured repetition은 fresh process다. P08의 10 warm queries만 cache semantics 때문에
한 repetition 안에서 같은 process/projector를 사용한다.

요약 통계:

- raw sample 전부 보존
- median, p95(nearest-rank), MAD, min, max
- sequential/parallel paired ratio
- process peak와 owner peak는 bytes 단위 integer로 보존
- p95 계산에 표본이 부족하면 schema상 `null`로 두고 baseline gate에서 제외

평균만으로 regression을 판정하지 않는다. outlier를 조용히 제거하지 않으며 실패한
repetition은 통계에서 빼지 않고 bundle result를 실패시킨다.

## 10. Baseline과 regression threshold

### 10.1 calibration admission

baseline 후보는 다음을 모두 만족해야 한다.

1. clean worktree와 exact commit
2. pinned container digest와 compiler/build type
3. 동일 fixture/config/input hash
4. GCC 12와 Clang 15 각각 최소 5개의 독립 nightly workflow run
5. run별 10 measured repetitions
6. correctness, sanitizer, architecture policy 통과
7. CPU/memory machine class가 동일하거나 명시적으로 별도 baseline key

### 10.2 baseline key

```text
schema/measurement_role/fixture/scenario/compiler/build_type
cpu_model/cpu_count/memory_class/container_digest
plugin IDs + config fingerprint + parallelism
```

container digest나 fixture hash가 바뀌면 기존 baseline에 강제로 비교하지 않고
`baseline_mismatch`로 fail closed한다.

`performance_baseline.json`은 위 exact key를 가진 entry들의 closed catalog다. 하나의
reviewed catalog를 `all-required` 실행에 전달하면 각 scenario/compiler/machine entry를
독립 선택한다. `measurement_role=public|owner`도 key에 포함하므로 동일 scenario의 public
latency/RSS와 private governor/cache 수치를 서로 다른 reviewed threshold로 gate한다.
exact key가 없거나 중복되면 전체 실행을 fail closed하며 scenario별
파일을 암묵적으로 추측하지 않는다. 단일-entry v1 파일은 기존 도구 호환 입력으로만
허용하고 repo의 canonical 산출물은 catalog schema를 따른다.
compiler key는 CMake가 선택한 executable 경로와 해당 executable의 `--version` 첫 줄을
함께 기록해 같은 경로에 설치된 compiler 교체도 key mismatch로 드러낸다.

### 10.3 초기 threshold 후보

아래 값은 calibration review의 시작점이며 측정 전 required gate가 아니다.

| Metric | 후보 |
|---|---|
| wall median | baseline 대비 `+15%`와 `+5 ms` 중 큰 allowance |
| wall p95 | baseline 대비 `+25%`와 `+10 ms` 중 큰 allowance |
| CPU median | baseline 대비 `+20%` |
| sampled peak RSS | baseline 대비 `+10%` 또는 `+64 MiB` 중 큰 allowance |
| retained owner bytes | 동일 fixture에서 exact 또는 설명된 structure-size 변화 |
| governor peak | baseline 대비 증가 시 owner review 필수; budget 초과는 즉시 실패 |
| cache hit/miss | P07 miss=1, P08 추가 miss=0 및 hit 증가 exact |
| output points/bytes/digest | correctness contract에 따라 exact/tolerance |
| cancellation max | watchdog 이내; 성능 gate는 calibration 후 설정 |

threshold는 `performance_baseline.json`의 review된 값으로만 활성화한다. CI가 최근
결과를 baseline으로 복사하거나 실패 후 allowance를 늘리면 안 된다.

### 10.4 baseline 갱신

baseline PR은 다음을 포함한다.

- old/new raw bundle artifact URL
- machine/config/fixture 동일성 확인
- metric별 old/new/delta 표
- correctness 및 resource-owner 영향 설명
- 성능 개선이면 원인, regression이면 승인 근거
- explicit performance review

production change와 baseline 갱신을 같은 PR에 넣을 수는 있지만 baseline diff를 별도
commit으로 둔다. 무관한 여러 scenario baseline을 일괄 재기록하지 않는다.

## 11. Report 계약

### 11.1 raw report v1

```json
{
  "schema_version": 1,
  "run_id": "medium-v1-map-update-sequential-0001",
  "measurement_role": "public",
  "profile": "nightly",
  "scenario": "map-update-sequential",
  "iteration": 1,
  "git": {"commit": "<40 hex>", "dirty": false},
  "build": {
    "compiler": "gcc-12.3.0",
    "build_type": "Release",
    "sanitizer": "none",
    "container_digest": "sha256:<64 hex>"
  },
  "machine": {
    "os": "Linux",
    "kernel": "<version>",
    "cpu_model": "<model>",
    "cpu_count": 16,
    "cpu_affinity": "0-15",
    "memory_bytes": 34359738368
  },
  "fixture": {
    "id": "medium-v1",
    "manifest_sha256": "<64 hex>",
    "decoded_point_count": 4194304,
    "decoded_point_bytes": 67108864
  },
  "timing": {},
  "process_memory": {},
  "owner_memory": {},
  "points": {},
  "io": {},
  "cache": {},
  "artifacts": {},
  "failures": [],
  "result": "pass"
}
```

closed schema는 unknown field, negative byte/time, NaN/Infinity, missing machine/fixture hash를
거부한다. unavailable metric은 field 생략이 아니라 `value:null`과 reason을 함께 둔다.

### 11.2 aggregate bundle

bundle은 raw report 경로/hash와 다음 summary를 가진다.

```text
sample_count, median, p95, MAD, min, max
baseline key/hash
comparison = pass | fail | uncalibrated | baseline_mismatch
metric별 expected/actual/allowance/delta
```

raw report를 aggregate JSON에 전부 복제하지 않고 content hash로 참조해 artifact 크기를
제한한다. report, bundle, baseline은 기존 파일을 덮어쓰지 않는다.

### 11.3 versioned outputs

```text
docs/post_freeze_results/05_benchmark_resource_observability.md
docs/post_freeze_results/performance_baseline.json
open_lmm/test/benchmark/schema/performance_report.schema.json
open_lmm/test/benchmark/schema/performance_bundle.schema.json
open_lmm/test/benchmark/schema/performance_pair.schema.json
open_lmm/test/benchmark/schema/performance_baseline.schema.json
.ci-build/benchmark/<profile>/<run-id>/**
```

repo baseline에는 raw samples를 모두 넣지 않고 승인된 summary/threshold와 source artifact
hash를 기록한다. raw evidence는 CI artifact로 보존한다.

## 12. 명령 계약

대표 local command:

```bash
scripts/benchmark/run_benchmarks.sh \
  --build <build-dir> \
  --profile pr \
  --fixture small-v1 \
  --scenario all-required \
  --repetitions 5 \
  --warmup 1 \
  --output <new-directory> \
  --container-digest sha256:<digest>
```

Calibration 승인 후 동일 key의 required run에는
`--baseline docs/post_freeze_results/performance_baseline.json`을 추가한다.

Nightly:

```bash
scripts/ci/run_benchmark_tests.sh \
  /usr/bin/gcc-12 \
  /usr/bin/g++-12 \
  medium-v1 \
  sha256:<digest> \
  nightly
```

명령 규칙:

- output directory는 존재하지 않아야 한다.
- required PR/nightly는 dirty tree를 거부한다.
- seed/fixture/hash/repetition 수를 report에 그대로 기록한다.
- unknown scenario/profile, 0 repetition, missing metric은 exit 2 또는 fail closed다.
- missing external/GPU asset은 전용 lane에서 exit 77이며 headless PASS로 변환하지 않는다.
- baseline update mode는 일반 run 명령에 제공하지 않는다.
- optional `OPEN_LMM_PERFORMANCE_BASELINE`이 지정되면 CI wrapper는 파일 부재를
  실패시킨다. 지정하지 않은 nightly는 canonical repo baseline이 실제 존재할 때만 이를
  자동 소비하며, 파일 생성이나 수치 갱신은 하지 않는다.

## 13. CMake / CTest 등록

```text
open_lmm_benchmark_report_contract_tests
  layer:L2;module:workflows.benchmark;owner:BenchmarkReport;lane:pr

open_lmm_benchmark_statistics_tests
  layer:L1;module:workflows.benchmark;owner:BenchmarkStatistics;lane:pr

open_lmm_benchmark_fixture_policy_tests
  layer:L2;module:workflows.benchmark;owner:BenchmarkFixture;lane:pr

open_lmm_benchmark_small_smoke_tests
  layer:L6;module:workflows.benchmark;owner:RuntimeWorkflow;lane:benchmark-pr

open_lmm_benchmark_medium_tests
  layer:L6;module:workflows.benchmark;owner:RuntimeWorkflow;lane:benchmark-nightly
```

benchmark binary는 기본 full CTest에 heavy scenario를 등록하지 않는다.

- contract/statistics/sampler tests는 기본 PR CTest에 포함한다.
- small smoke는 기본 CTest에서 command/report contract를 검증한다.
- fresh-process workflow CTest는
  `OPEN_LMM_ENABLE_BENCHMARK_WORKFLOW_TEST=ON`에서만 등록한다.
- 실제 PR/medium 반복은 CTest에 heavy scenario로 넣지 않고 전용 CI script가 실행한다.
- external/GPU는 각 전용 option/runner가 있을 때만 등록한다.
- sanitizer build는 benchmark timing을 baseline으로 저장하지 않는다.

architecture policy는 public workflow runner가 private `runtime/`, `config/`,
`open_lmm_map_server`에 의존하지 않고 `RuntimeClient`만 소비하는지 검사한다. owner runner가
production `.cpp`를 재컴파일하지 않고 canonical target을 링크하는지도 검사한다.

## 14. CI topology

### 14.1 PR benchmark

- dedicated stable job name: `benchmark / gcc12-small`
- source Dockerfile build의 exact image digest 기록, Release, GUI off
- report contract/statistics tests
- small-v1 P01/P02/P04/P07/P09, warmup 1 + measured 5
- small-v1은 correctness/report contract gate로 유지하며 timing 결과는
  `uncalibrated`로 보존한다. 승인 baseline은 medium-v1 nightly exact key에만 적용한다.
- report/JUnit/log를 14일 보존

기존 compiler matrix full CTest에 medium benchmark를 넣지 않는다.

### 14.2 nightly benchmark

- GCC 12 / Clang 15, `fail-fast:false`
- repository variable `OPEN_LMM_BENCHMARK_IMAGE`는 반드시
  `image@sha256:<64 hex>` 형식의 review된 immutable image를 가리킨다.
- nightly는 이미지를 재빌드하지 않고 exact digest를 pull하며, 변수가 없거나 tag-only면
  측정 전에 fail closed한다.
- medium-v1 P01–P10, warmup 2 + measured 10
- compiler별 process 격리
- raw/aggregate/comparison report 30일 보존
- exact owner failure와 performance regression을 summary에서 분리
- benchmark와 nightly soak job은 목적이 다르므로 같은 report/baseline으로 합치지 않는다.

### 14.3 external/GPU

- Goal 03 provenance-valid locked bundle만 large evidence로 허용
- real-driver runner는 driver/GPU model/VRAM/API metadata 필수
- software renderer 결과를 hardware GPU baseline으로 승격하지 않는다.
- GPU query/upload 실패는 headless visualization 결과로 대체하지 않는다.

## 15. 구현 단계

### Phase A — contract와 sampler

1. benchmark raw/bundle schema 및 validator
2. median/p95/MAD/ratio와 baseline comparator contract tests
3. process target-window RSS/CPU/I/O sampler
4. no-overwrite evidence writer와 machine/fixture metadata 재사용

완료 조건: runtime 없이 deterministic scalar fixture로 schema/statistics/sampler가 통과한다.

### Phase B — fixture와 public workflow runner

1. small-v1/medium-v1 deterministic generator와 manifest
2. public `RuntimeClient` runner
3. execution event recorder와 P01–P10 command schedule
4. output/artifact correctness digest

완료 조건: 같은 binary/fixture/seed에서 command/result/digest가 반복 동일하고 각 measured
run이 fresh process다.

### Phase C — owner observability

1. `ResourceGovernorDiagnostics`
2. `VisualizationProjectorDiagnostics`
3. private DataLoad/MapUpdate/Save/viz owner runner
4. retained/output/peak comparison과 sequential/parallel paired report

완료 조건: public report와 owner report가 동일 fixture/scenario key로 join되고 class별
reservation/cache cardinality가 exact하다.

### Phase D — PR/nightly evidence

1. opt-in CTest registration
2. scripts와 pinned CI workflow
3. small PR 및 medium nightly raw bundle 생성
4. RSS/latency/I/O/owner 결과를 이 문서에 기록

완료 조건: 대표 benchmark command와 실제 peak RSS/stage latency/resource owner 숫자가
존재한다.

### Phase E — calibration

1. compiler별 5개 독립 clean nightly run 수집
2. variance와 machine class 검토
3. threshold 후보 확정
4. `performance_baseline.json` 수동 작성 및 review

완료 조건: baseline source hash와 승인 근거가 있고 CI가 baseline을 자동 갱신하지 않는다.

### Phase F — optional external/GPU

1. Goal 03 large bundle 연결
2. real-driver visualization/upload runner
3. external/GPU baseline을 headless와 별도 key로 관리

이 phase는 asset/runner availability 때문에 headless completion과 분리할 수 있다.

## 16. 검증 체크리스트

- [x] P01–P10이 canonical public command/state barrier를 사용한다.
- [x] small/medium fixture가 deterministic hash와 decoded point count를 가진다.
- [x] Goal 03 owner-controlled representative bundle은 provenance/input-lock을 통과한다.
  `large-external-v1`은 별도 optional tier로 남는다.
- [x] repetition마다 fresh process이고 warm/cold cache 의미가 명시된다.
- [x] wall/CPU/stage/cancel latency raw sample이 존재한다.
- [x] sampled target peak와 process HWM이 구분된다.
- [x] governor total/resident/transient/heavy와 RSS가 별도 field다.
- [x] output/retained/peak bytes와 ratio가 P02/P04/P05/P06/P07에 존재한다.
- [x] disk logical/physical I/O counter가 구분된다.
- [x] visualization cache hit/miss/eviction/bytes가 exact하다.
- [x] GPU 값은 `gpu_measurement_kind`와 unavailable reason으로 estimate/
  driver-observed 여부를 명시한다. 현재 real-driver 값은 `NOT_AVAILABLE`이다.
- [x] sequential/parallel output parity 후에만 performance ratio를 계산한다.
- [x] report schema가 closed이고 existing evidence를 덮어쓰지 않는다.
- [x] baseline mismatch/uncalibrated가 PASS로 위장되지 않는다.
- [x] PR/nightly/external/GPU lane이 분리된다.
- [x] installed public API/ABI와 plugin ABI가 변하지 않는다.
- [x] architecture policy, full CTest, package consumer, sanitizer가 유지된다.
  2026-08-25 현재 architecture policy, full CTest `80/80`, package consumer,
  ASan/UBSan `71/71`, TSan `22/22`가 통과했다.
- [x] streaming/sidecar/point-cloud redesign이 이 Goal 구현에 섞이지 않는다.

## 17. Known limitations

- sampled RSS는 allocator attribution이나 엄밀한 allocation peak가 아니다.
- 1 ms sampler 사이의 더 짧은 spike를 놓칠 수 있다. `VmHWM`은 process 전체 보조값이다.
- page cache와 shared-runner scheduling을 완전히 통제할 수 없다.
- `ResourceGovernor`는 soft admission estimate이며 plugin-private allocator를 관측하지 않는다.
- PCL/third-party internal workspace와 hash-table allocator overhead는 owner estimate 밖일 수 있다.
- output compression ratio는 dataset shape에 따라 달라 decoded bytes와 직접 비교할 수 없다.
- exact GPU resident memory는 driver API 없이는 보장할 수 없다.
- external large result는 dataset redistribution/provenance와 runner capacity가 필요하다.
- power-loss durability, OOM killer, process-wide allocation failure는 benchmark 결과로 증명하지 않는다.
- threshold는 machine/compiler/container/fixture key를 벗어나 일반화하지 않는다.

## 18. 완료 판정 형식

구현 결과는 scenario/tier별로 다음 중 하나를 사용한다.

- `PASS`: correctness, report contract, owner metric 및 활성 threshold 통과
- `FAIL`: correctness/resource exact gate 또는 활성 regression threshold 실패
- `UNCALIBRATED`: 유효한 측정은 존재하지만 승인 baseline이 없음
- `NOT_AVAILABLE`: external/GPU asset 또는 runner 부재
- `INVALID`: provenance/hash/config/machine contract 실패로 비교 불가

Headless small/medium 결과가 통과해도 external large와 real-driver GPU를 `PASS`로 추론하지
않는다. 반대로 optional tier 부재는 대표 headless baseline 완료를 막지 않지만 반드시
이 문서의 known limitations/result matrix에 남긴다.

## 19. 최종 산출물

```text
docs/post_freeze_results/05_benchmark_resource_observability.md
docs/post_freeze_results/performance_baseline.json
open_lmm/test/benchmark/**
open_lmm/test/support/benchmark/**
open_lmm/test/cmake/registrations/benchmark.cmake
scripts/benchmark/**
scripts/ci/run_benchmark_tests.sh
.github/workflows/nightly-benchmark.yml
```

초기 명세 단계에서는 baseline JSON을 만들지 않았으며, 실제 구현·반복 측정·calibration
review를 완료한 뒤 `performance_baseline.json`을 추가하고 승인했다. 최종 근거와 hash는
아래 20.3을 따른다.

## 20. 구현 진행 기록

2026-08-20 기준 Phase A–D의 핵심 구현이 존재한다.

- `ResourceGovernorDiagnostics`: budget, memory class별 reservation,
  admission failure, heavy-phase, executor snapshot
- `VisualizationProjectorDiagnostics`: entry/byte와
  hit/miss/insertion/eviction/clear 누적치
- benchmark statistics: median, nearest-rank p95, MAD, min/max
- 1 ms target-window sampler: wall/process CPU, RSS/HWM, `/proc/self/io`
- closed raw report validator/schema와 no-overwrite writer
- small/medium fixture manifest policy 및 deterministic `small-v1` generator
- locale-independent fixed-precision fixture serialization (`to_chars`)로
  compiler/runner locale에 무관한 입력 hash 보장
- callback-entry 기반 stage event recorder
- strict profile/scenario/build metadata CLI와 dirty required-lane 거부
- 실제 `RuntimeClient`를 사용하는 P01–P10 public workflow runner
- P04/P05용 동일 ERASOR fixture와 sequential/parallel mode 계약
- callback safe point에서 취소하고 authoritative terminal state/revision을 확인하는 P10
- raw report hash를 참조하는 aggregate bundle, median/p95/MAD 요약,
  fail-closed baseline comparator
- 단일 reviewed catalog에서 scenario/compiler/machine exact key를 선택하고 missing/duplicate
  key를 거부하는 multi-scenario baseline 계약과 closed JSON schema
- public/owner report role을 baseline key로 분리하고 동일 catalog를 양쪽 aggregate에
  적용하며, paired report가 네 bundle의 pass/fail/mismatch/uncalibrated 상태를 전파
- repetition마다 public/owner runner를 새 process로 실행하는 strict orchestrator
- P09 RunAll의 ordered DataLoad/Alignment/(MapUpdate)/Save event window를
  `stage_timings`에 보존하는 stage별 breakdown
- P04/P05에서 parallel flag만 정규화한 pair fingerprint, PCD semantic parity,
  iteration별 wall/CPU speedup, RSS 및 governor/executor peak를 결합하는 closed pair report
- private canonical `open_lmm_map_server` target을 소비하는 owner runner와
  governor/executor/projector peak diagnostics
- required `benchmark / gcc12-small` job과 immutable image digest를 강제하는 별도
  GCC 12/Clang 15 nightly workflow
- 실제 small-v1 P01–P10 contract 실행 및 raw/owner/aggregate report 검증

P04/P05 paired comparator는 raw artifact digest identity와 PCD semantic parity를 분리한다.
PCD parity 기준은 output voxel 0.4 m에 대해 양방향 nearest-neighbor RMS 0.04 m,
max 0.4 m, point-count ratio 0.001이다. threshold와 근거는 pair report에 기록된다.
KISS alignment metadata 때문에 byte digest가 달라도 semantic comparator를 생략하거나
digest equality를 조작하지 않는다.

local contract evidence의 대표 관측값은 다음과 같다. 이 실행은 dirty worktree에서 허용되는
`contract` profile이므로 승인 baseline이나 성능 gate로 사용하지 않는다.

| Scenario | 관측값 | Owner 관측값 | 판정 |
|---|---|---|---|
| P01 Open | wall 30.0 ms, sampled RSS 74,940,416 B | public boundary에서 unavailable | `UNCALIBRATED` |
| P02 DataLoad | stage 42.3 ms, sampled RSS 44,535,808 B | resident/peak 2,097,598 B | `UNCALIBRATED` |
| P04 MapUpdate sequential | stage 45.1 ms, sampled RSS 75,595,776 B | heavy peak 3,140,936 B, total peak 5,238,534 B | `UNCALIBRATED` |
| P05 MapUpdate parallel | stage 27.1 ms, sampled RSS 78,512,128 B | heavy peak 6,281,872 B, total peak 8,379,470 B, max active 2 | `UNCALIBRATED` |
| P04/P05 pair | PCD parity PASS, byte digest unequal, wall speedup 1.77× | sequential/parallel peak 5,238,534/8,379,470 B | `UNCALIBRATED` |
| P08 visualization warm | 10 warm queries | entries 1, hits 10, additional misses 0 | `UNCALIBRATED` |
| P09 full pipeline | DataLoad 44.3 ms, Alignment 49.0 ms, Save 13.1 ms | ordered event windows verified | `UNCALIBRATED` |
| P10 cancellation | cancel-to-terminal 558,519 ns, final revision unchanged | public cancellation authority verified | `UNCALIBRATED` |

검증 결과:

- current build 전체 CTest `80/80` 통과
- opt-in fresh-process orchestrator CTest 통과
- `scripts/ci/check_architecture_policy.sh` 통과
- install/relocation, 공개 헤더 27개, full/contracts/client/plugin-SDK 최소 소비자를
  포함한 package consumer 검증 통과
- raw/bundle/pair JSON schema 구문 검증 통과
- `git diff --check` 및 benchmark shell syntax 검사 통과
- Goal 05 ASan/UBSan benchmark 7개 통과, fixture policy 20회 연속 통과
- current public/owner baseline catalog 변경을 포함한 Clang 15 ASan/UBSan benchmark
  7개 재빌드·재실행 통과
- 변경된 governor/executor/visualization TSan 대상 3개를 각 3회 연속 통과

과거 sanitizer 실행에서는 reconfiguration E2E의 비결정적 `SIGILL`, fixture의 glibc
float formatting `SIGSEGV`, runtime concurrency stress의 일시적 thread-count 편차가
각각 관찰됐다. fixture formatting은 locale-independent `to_chars`로 교체했고 catalog
contract의 단일 JSON entry도 `Json::array(...)`로 명시했다.

2026-08-25 fresh ASan/UBSan 전체 실행은 `DescriptorStore::rebuild_merged_map()`의
Eigen lazy-expression lifetime 문제를 결정적으로 검출했다. homogeneous point와 변환
결과를 값으로 materialize하는 최소 수정 후 최초 실패한 4개 target을 재실행해 `4/4`,
이어서 전체 suite를 재실행해 `71/71` 통과했다. fresh TSan suite도 `22/22` 통과했고
race report가 없었다. 따라서 현재 sanitizer checkpoint는 green이다.

이후 immutable OCI image와 고정 machine/cpuset에서 medium-v1 compiler별 5회
calibration을 완료하고 `performance_baseline.json`을 승인했다. Goal 03
owner-controlled representative bundle 연결도 완료됐다. external large 및
real-driver GPU lane은 별도 optional tier로 남는다. KISS output의 byte identity가
반복마다 달라질 수 있으므로 P04/P05는 byte digest와 semantic parity를 계속 별도
evidence로 유지한다.

### 20.1 2026-08-25 calibration 진행 기록

`small-v1`, `profile=pr`, warmup 1 + measured 5 조건으로 compiler별 5개의
독립 clean source run을 수집했다.

```text
source commit: 001a80730c217975b267cbce85a1aaab683eb5a2
machine: Intel(R) Core(TM) Ultra 9 285K / 24 logical CPUs / 30 GiB class
GCC: /usr/bin/g++-12, 12.3.0
Clang: /usr/bin/clang++-15, 15.0.7
environment source-spec digest:
  sha256:0d1952784a97306414b1b1ab175d98c1dc1d9bf6dcf76f86173f6d61a37a77f4
```

GCC의 reviewed job set은 `job-4,6,7,8,9`, Clang은
`job-1,2,3,5,6`이다. 처음 생성한 일부 job은 signed
`retained_rss_delta_bytes`를 aggregate가 양수 sample에서만 unsigned
metric으로 수집하는 결함 때문에 visualization bundle 단계에서 중단됐다. 실패
evidence는 삭제하지 않았고, raw signed 값은 보존하되 non-negative baseline
aggregate에서는 일관되게 제외하도록 수정하고 회귀 테스트를 추가했다.

5-job wall median의 범위는 다음과 같다.

| Compiler | Open | DataLoad | MapUpdate seq | Viz cold | Full pipeline |
|---|---:|---:|---:|---:|---:|
| GCC 12 | 28.946–29.634 ms | 43.452–44.400 ms | 44.866–45.672 ms | 2.095–2.362 ms | 90.388–97.544 ms |
| Clang 15 | 109.131–109.705 ms | 42.844–43.343 ms | 47.315–48.914 ms | 2.234–3.120 ms | 93.714–103.008 ms |

이 결과로 아래 evidence-only candidate를 만들고 16개 exact
compiler/scenario/role key를 각각 원 raw report에서 재-aggregate하여 16/16
comparison PASS를 확인했다. Threshold는 관측 max에 맞춰 자동 확장하지 않았고,
명세의 사전 review policy를 그대로 사용했다.

```text
/root/dataset_root/openlmm-benchmark-evidence/
goal05-performance-baseline-candidate-small-v1-001a807.json
```

이 파일은 승인되지 않았으므로 canonical
`docs/post_freeze_results/performance_baseline.json` 경로에는 두지 않는다.

- wall median: `+15%` 또는 `+5 ms`
- wall p95: `+25%` 또는 `+10 ms`
- CPU median: `+20%`
- sampled peak RSS median: `+10%` 또는 `+64 MiB`
- deterministic governor/cache owner maxima: exact upper bound

Candidate catalog SHA-256은
`daaaaec1f7ddce76223ab6b967d22838a7b826a30c66d6af508e339bf2349038`이고,
5-job selection/source bundle review record SHA-256은
`d634906a28ecb9cc93607f178f52c4659616af18617f8a9ffd1a9cd613dfa769`이다.

이 calibration은 현재 컨테이너에서 얻은 host evidence다. 위 digest는
`docker/open_lmm.Dockerfile` source specification hash이며 registry OCI manifest
digest가 아니다. 따라서 아직 최종 pinned nightly baseline으로 승인하지 않는다.
실제 `image@sha256:<manifest>`와 동일 image/machine에서 medium-v1 nightly 5회를
GCC/Clang 각각 완료한 뒤 catalog key/source bundle을 교체해야 한다. 이 구분은
moving/tag-only environment를 승인 baseline으로 잘못 승격하지 않기 위한
fail-closed 정책이다.

### 20.2 medium-v1 host smoke 및 최종 로컬 checkpoint

최종 pinned calibration에 들어가기 전, 같은 clean source commit과 현재 host에서
`profile=nightly`, `medium-v1`, P01–P10 전체를 warmup 2 + measured 10으로 1회 실행했다.

```text
evidence root:
  /root/dataset_root/openlmm-benchmark-evidence/
  goal05-nightly-gcc12-host-001a807/job-1
compiler: GCC 12.3.0
source commit: 001a80730c217975b267cbce85a1aaab683eb5a2
bundle/report manifest SHA-256:
  380d8d349f5906712933e0d264ace70eb21291e547da0dd5e3ebea3e872e3d49
```

| Scenario | wall median | wall p95 | sampled RSS delta median | process HWM |
|---|---:|---:|---:|---:|
| P01 Open | 30.147 ms | 30.577 ms | 43,468,800 B | 76,480,512 B |
| P02 DataLoad | 2.773 s | 2.788 s | 170,274,816 B | 210,614,272 B |
| P03 Alignment | 5.996 s | 6.056 s | 294,166,528 B | 503,697,408 B |
| P04 MapUpdate sequential | 3.004 s | 3.062 s | 273,115,136 B | 759,937,024 B |
| P05 MapUpdate parallel | 1.747 s | 1.755 s | 479,406,080 B | 964,190,208 B |
| P06 Save fallback | 331.263 ms | 343.031 ms | 152,799,232 B | 646,502,400 B |
| P07 visualization cold | 157.696 ms | 164.545 ms | 99,643,392 B | 646,279,168 B |
| P08 visualization warm | 2.182 ms | 2.339 ms | 0 B | 642,367,488 B |
| P09 full pipeline | 9.068 s | 9.149 s | 606,814,208 B | 647,608,320 B |
| P10 cancellation | 1.123 ms | 1.750 ms | 475,136 B | 76,972,032 B |

P04/P05 semantic parity는 10회 모두 통과했고, iteration wall speedup은 약 `1.71x`였다.
parallel mode가 더 빠른 대신 sampled RSS와 HWM이 더 높다는 결과도 owner/resource report와
일치한다. 모든 bundle은 의도대로 `UNCALIBRATED`이며 이 1회 host smoke를 승인 baseline으로
승격하지 않는다.

최종 로컬 검증 결과는 다음과 같다.

- architecture/release policy: PASS
- main full CTest: `80/80` PASS
- fresh ASan/UBSan: `71/71` PASS
- fresh TSan: `22/22` PASS, data-race report 없음
- `git diff --check`: PASS

이 section은 최종 pinned calibration 전의 host smoke 기록이다. 이 smoke 자체는
canonical baseline source가 아니며, 최종 승인 증거는 아래 20.3을 따른다.

### 20.3 immutable medium-v1 calibration 및 canonical baseline 승인

최종 calibration은 다음 공통 계약으로 실행했다.

```text
source commit: b168b3db89096ca7010afc15707abb223bcd0c71
fixture/profile: medium-v1 / nightly
per run: warmup 2 + measured 10, P01-P10 all-required
isolation: one Docker container per run
CPU: Docker --cpuset-cpus=0-7
reported host: Intel(R) Core(TM) Ultra 9 285K / 24 logical CPUs / 16-31GiB
```

GCC 12는 사용자 소유 이미지의 immutable manifest를 직접 사용했다.

```text
hwan0806/open-lmm@sha256:aabcc53791995ce4ddf9606f9710cbffa730bb555646f01b61844c7c6724eb6c
approved: calibration-run-2,3,4,5,6
```

Clang 15는 같은 immutable base에 정확한 Clang/LLVM 15.0.7과 libomp 15를 설치하는
repo Dockerfile로 파생했다. 파생 recipe SHA-256은
`0e5c492f185866781b6e6d1404c945de23359d40c0fa8d6d4dca816bdb3c44b4`이고,
측정에 사용한 local registry manifest는 다음과 같다.

```text
127.0.0.1:5000/openlmm-calibration-clang15@
sha256:b546cb17dd9ea8a7f2d7c93e4bd86abafff83d5653d0f9085f01fb404b8e0fc6
approved: calibration-run-1,2,3,5,7
```

승인 5-run의 public wall median 범위와 cross-run median/MAD는 다음과 같다.

| Scenario | GCC 12 min / median / max / MAD | Clang 15 min / median / max / MAD |
|---|---:|---:|
| P01 Open | 28.871 / 28.949 / 29.050 / 0.077 ms | 109.013 / 109.043 / 109.345 / 0.030 ms |
| P02 DataLoad | 2793.070 / 2796.790 / 2803.704 / 2.351 ms | 2741.927 / 2746.545 / 2758.389 / 4.618 ms |
| P03 Alignment | 6562.554 / 6593.114 / 6613.502 / 8.306 ms | 6761.459 / 6796.752 / 6823.811 / 12.268 ms |
| P04 MapUpdate seq | 3003.230 / 3004.619 / 3012.658 / 1.389 ms | 2923.931 / 2925.931 / 2931.484 / 0.671 ms |
| P05 MapUpdate parallel | 1746.183 / 1753.421 / 1761.391 / 2.536 ms | 1696.116 / 1699.431 / 1702.500 / 1.753 ms |
| P06 Save fallback | 327.974 / 330.386 / 331.534 / 0.770 ms | 341.717 / 341.956 / 343.609 / 0.239 ms |
| P07 Visualization cold | 158.734 / 158.813 / 159.817 / 0.080 ms | 133.948 / 137.192 / 138.857 / 0.726 ms |
| P08 Visualization warm | 2.183 / 2.199 / 2.233 / 0.004 ms | 1.362 / 1.427 / 1.453 / 0.012 ms |
| P09 Full pipeline | 9700.087 / 9711.227 / 9733.586 / 2.575 ms | 9852.516 / 9864.232 / 9929.008 / 7.418 ms |
| P10 Cancellation | 0.521 / 0.607 / 0.686 / 0.075 ms | 0.719 / 0.771 / 0.813 / 0.027 ms |

각 승인 run은 17개 bundle, 170개 measured/owner raw report와 P04/P05 semantic
parity `10/10 PASS`를 갖는다. 다음 run은 승인 집합에서 제외했으며 삭제하지 않았다.

- GCC run 1: 대형 registry push 직후의 다변량 host-load outlier
- Clang run 4: Linux kernel `task_mm_cid_work` Oops로 benchmark PID가 종료된 불완전 run
- Clang run 6: Open/DataLoad/Alignment가 동시에 상승한 host-load outlier

16-entry catalog는 Open, DataLoad public/owner, MapUpdate sequential public/owner,
Visualization cold public/owner, Full pipeline public을 compiler별 exact key로 고정한다.
대표 bundle은 각 항목에서 5-run wall median의 중앙값에 해당하는 실제 bundle이다.
Threshold는 사전에 정의한 `+15%/+5ms`, `+25%/+10ms`, CPU `+20%`, RSS
`+10%/+64MiB`, owner maxima exact 규칙을 그대로 사용했고 관측 max에 맞춰 자동
확대하지 않았다.

```text
canonical catalog:
  docs/post_freeze_results/performance_baseline.json
  sha256:6b8c07ca844c49567a35900621004dabbf15a266c70e6dc676ef6feb360dbca2
manual review record:
  /root/dataset_root/openlmm-benchmark-evidence/
  goal05-immutable-calibration-review-b168b3d.json
  sha256:86915f37c301ba01e6c6009c033da4f0747f55b85de25f6b720e6363a386c379
independent re-aggregation:
  16/16 PASS
  validation-set sha256:af749c3643bf32da6d813ff73d646ae06e12beb2b32a95b3316bc12a1d33aee1
```

따라서 대표 headless scope는 `PASS`다. `large-external-v1`과 real-driver GPU는
`NOT_AVAILABLE`이며, optional tier의 부재를 headless 실패나 성공으로 재해석하지 않는다.
