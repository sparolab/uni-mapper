# OpenLMM Post-Freeze 01--05: 현재 상태 및 다음 진행 순서

> 2026-08-25 실행 업데이트: 이 문서는 착수 당시의 gap과 순서를 설명하는 가이드다.
> Goal 03은 owner-controlled `test1`/`test2`를 tiny/small/representative/failure로
> 고정하고 5-run baseline, comparator, replay workflow까지 구현하여 **internal replay
> 범위에서 완료**됐다. Goal 05도 immutable OCI image, 고정 CPU affinity, GCC 12/Clang 15
> medium-v1 각 5-run과 canonical `performance_baseline.json` 16-entry 승인까지 완료됐다.
> 따라서 03/05 P0는 닫혔고, external large·real-driver GPU 및 hosted secret/첫 scheduled
> run은 optional/운영 후속 checkpoint다.

## 1. 현재 상태 요약

현재 Post-Freeze Goal 01--05의 상태는 다음과 같다.

  -----------------------------------------------------------------------
  Goal                    현재 상태               핵심
  ----------------------- ----------------------- -----------------------
  Goal 01 ---             완료                    Architecture baseline,
  Architecture Freeze                             invariant, dependency
  Baseline                                        gate 확립

  Goal 02 --- Test        완료                    Owner/layer/invariant
  Architecture                                    기반 테스트 구조 확립
  Reorganization

  Goal 03 ---             완료                    owner-controlled
  Real-dataset E2E Replay                         test1/test2, four tiers,
                                                  baseline, rollback,
                                                  trusted CI policy 완료

  Goal 04 --- Soak /      Headless 완료           Headless
  Fault / Stress                                  stress/fault/soak
                                                  infrastructure와 반복
                                                  검증 완료

  Goal 05 --- Benchmark / 완료                    P01-P10, owner metrics,
  Resource Observability                          immutable compiler별
                                                  5-run, canonical baseline
  -----------------------------------------------------------------------

따라서 이 문서의 Track A/B 실행 절차는 완료 이력이다. 다음 Goal 착수 시에도
여기서 고정한 replay/baseline을 regression gate로 유지한다.

## 2. Track A --- Goal 03 실제 데이터셋 공식화

Goal 03의 핵심 blocker는 replay 코드가 아니라 **공식 regression
baseline으로 사용할 수 있는 실제 데이터셋**이다.

직접 취득한 데이터를 다음 tier로 구성한다.

### tiny

-   2개 map/session
-   짧은 scan 구간
-   두 map 사이 실제 overlap 존재
-   cross-map alignment/loop가 발생
-   PR CI에서 빠르게 실행 가능한 크기

### small

-   tiny보다 긴 2개 이상의 map/session
-   Save 및 config rerun 검증
-   numeric regression을 확인할 수 있는 크기

### representative

-   실제 OpenLMM 사용 환경에 가까운 데이터
-   가능하면 3--7 map/session
-   nightly/external CI용

### failure

-   tiny에서 결정적으로 파생
-   PCD 손상/누락/cardinality mismatch 등
-   commit 전 실패와 rollback 검증

## 3. Goal 03에서 GT는 필수가 아님

Goal 03은 **map-merging accuracy 평가가 아니라 regression replay**가
목적이다.

따라서 다음은 필수가 아니다.

``` text
GT merged map
GT session-to-session transform
GT global trajectory
```

필요한 핵심 입력은 다음과 같다.

``` text
각 session의 scan/PCD
+
각 session의 local pose
+
session 간 overlap
+
OpenLMM config
```

최초 정상 실행을 baseline으로 만들고 이후 결과를 비교한다.

``` text
Canonical metadata / stage / revision semantics
→ exact comparison

Alignment/loop identity
→ exact 또는 reviewed allowed set

Pose / trajectory
→ tolerance

Merged-map geometry
→ tolerance / range

Timestamp / scheduling-dependent 정보
→ diagnostic only
```

별도의 알고리즘 정확도 평가를 수행할 때는 session-to-session SE(3) GT
또는 global trajectory GT를 추가하는 것이 좋다.

## 4. Dataset provenance 및 lock

직접 취득한 데이터라면 다음 정보를 기록한다.

``` text
dataset_id: openlmm-office-01
source: self-collected
owner: project/internal
acquisition_date: YYYY-MM-DD
sensor: <sensor information>
redistribution: permitted | internal-only
```

그리고 다음을 고정한다.

-   데이터 출처/소유자
-   취득일
-   sensor/config 정보
-   가공/다운샘플/프레임 선택 이력
-   재배포 가능 여부
-   archive SHA-256
-   file-level SHA-256
-   config SHA-256
-   versioned frame/session manifest

Public CI라면 tiny/small을 공개 가능한 artifact로, private CI라면
immutable private artifact로 관리할 수 있다.

## 5. Goal 03 baseline 생성

``` text
clean environment
      ↓
tiny replay × 5
      ↓
exact-field stability 확인
      ↓
numeric variation 확인
      ↓
tolerance 결정
      ↓
baseline review/승인
```

확인 대상: - agent ordering - stage ordering - revision 변화 - artifact
state - alignment/loop identity - pose variation - merged-map point
statistics - map bounds/centroid - output artifact - Close 성공

Baseline은 CI가 자동 갱신하지 않는다.

## 6. Goal 03 CI 연결

``` text
PR / main
└── replay / tiny-small
    ├── tiny normal
    ├── small normal
    └── failure replay

Nightly
└── replay / representative
```

보존할 evidence: - replay report - comparator diff - JUnit - failure
logs - dataset/baseline hash - commit/build metadata

여기까지 완료하면 Goal 03을 완료 상태로 승격한다.

## 7. Track B --- Goal 05 Calibration

Goal 05는 이제 benchmark 기능을 더 개발하기보다 **clean environment에서
공식 baseline을 만드는 단계**다.

현재 구현된 범위:

``` text
P01 Runtime Open
P02 DataLoad
P03 Alignment
P04 MapUpdate Sequential
P05 MapUpdate Parallel
P06 Save Fallback
P07 Visualization Cold
P08 Visualization Warm
P09 Full Pipeline
P10 Cancellation
```

또한 fresh-process orchestrator, RSS/HWM, CPU, I/O,
ResourceGovernor/Visualization diagnostics, stage timing, public/owner
runner, sequential/parallel parity, aggregate/pair report와 comparator가
구현되어 있다.

## 8. Goal 05 Calibration 실행 순서

``` text
Pinned benchmark environment 확정
        ↓
GCC 12 clean nightly × 5
        +
Clang 15 clean nightly × 5
        ↓
각 run measured repetitions
        ↓
raw / aggregate / pair artifact 저장
        ↓
median / p95 / MAD 분석
        ↓
RSS / owner metric variance 분석
        ↓
threshold review
        ↓
performance_baseline.json 생성
        ↓
PR regression gate 활성화
```

고정할 환경: - compiler/version - build type - container image digest -
CPU model/class - CPU count - memory class - fixture hash - config
hash - plugin IDs - parallelism

환경 key가 다르면 기존 baseline과 강제 비교하지 않는다.

## 9. Goal 05 주요 threshold

실제 threshold는 calibration 결과를 보고 승인한다.

검토 대상: - wall median - wall p95 - CPU median - sampled peak RSS -
retained owner bytes - ResourceGovernor peak - visualization cache
hit/miss - output parity - cancellation latency

최종 산출물:

``` text
docs/post_freeze_results/performance_baseline.json
```

## 10. Sanitizer 최종 재검증

Calibration과 함께 전체 sanitizer matrix의 clean green evidence를
남긴다.

``` text
Clean build
   ↓
ASan / UBSan full lane
   ↓
TSan full lane
   ↓
all green
```

단독 재실행만으로 flaky failure를 PASS 처리하지 않고 clean full lane이
green인지 확인한다.

## 11. Goal 03과 Goal 05는 병렬 진행

``` text
             ┌─ Goal 03
             │  실제 dataset 정리
             │      ↓
             │  tiny/small/representative
             │      ↓
현재 HEAD ───┤  baseline + replay CI
             │
             └─ Goal 05
                pinned environment
                     ↓
                GCC/Clang calibration
                     ↓
                performance baseline
```

Goal 05의 small/medium headless benchmark는 Goal 03 없이 진행할 수 있다.

## 12. Goal 03 완료 후 Goal 05와 연결

``` text
Goal 03 representative locked dataset
             ↓
Goal 05 large-external-v1
             ↓
Real multi-map workload benchmark
```

이후 실제 데이터에서
DataLoad/Alignment/MapUpdate/Save/visualization/full-pipeline의
latency와 memory peak를 측정한다.

## 13. P1 --- External / GPU

P0 완료 후 필요하면 진행한다.

-   Goal 03 representative 기반 large external benchmark
-   실제 GPU/Iridescence lifecycle
-   upload/remove/supersession
-   GPU model/VRAM/driver metadata
-   GPU upload/resident metric

Headless core completion의 선행 blocker는 아니지만 GUI/GPU가 제품 핵심
경로라면 release 전 검증한다.

## 14. P2 --- 비차단 보강

후순위: - Goal 04 RSS hard threshold calibration - scheduled workflow
실제 성공 기록/URL 보존 - CI artifact retention evidence - Goal 03 결과
문서 체크박스 갱신 - historical test count 정리 - architecture freeze
annotated tag 생성 여부 결정

## 15. 완료된 실행 순서

### Track A --- Goal 03

``` text
1. 직접 취득 dataset inventory
2. tiny/small/representative 후보 선정
3. overlap/alignment 발생 여부 검증
4. deterministic subset 생성
5. provenance manifest 작성
6. SHA-256 lock
7. tiny 5회 calibration
8. normal/failure baseline 승인
9. replay CI 연결
10. representative nightly 연결
```

### Track B --- Goal 05

``` text
1. pinned benchmark image/machine 확정
2. GCC12 clean nightly ×5
3. Clang15 clean nightly ×5
4. raw/aggregate/pair evidence 보존
5. variance 분석
6. threshold 승인
7. performance_baseline.json 생성
8. full ASan/UBSan + TSan green 확인
9. performance regression gate 활성화
```

두 Track은 병렬로 진행했고 2026-08-25에 완료됐다.

## 16. 다음 단계 해제 조건

03/05 P0가 닫혔으므로 다음 Goal은 별도 scope와 architecture invariants를 유지하는
조건으로 착수할 수 있다.

``` text
Goal 06 Python Binding
Goal 07 Experiment Toolkit
새 대규모 feature
새 architecture redesign
Generic PointCloud migration
```

먼저 다음 milestone을 만든다.

``` text
Architecture Freeze
        +
Test Architecture
        +
Real Dataset Regression
        +
Stress / Fault / Soak
        +
Performance Baseline
        ↓
Post-Freeze Production Validation Core Complete
```

## 17. Codex에 바로 줄 수 있는 다음 Goal

### Goal 03 Dataset Onboarding

``` text
현재 직접 취득한 dataset을 Goal 03 replay dataset으로 승격하라.

1. dataset 구조, agent/session, scan, pose, config를 inventory한다.
2. tiny/small/representative 후보를 선정한다.
3. 각 tier에서 실제 cross-map overlap/alignment가 발생하는지 실행 검증한다.
4. reproducible subset generator를 작성한다.
5. dataset provenance manifest를 작성한다.
6. archive/file/config SHA-256 lock을 생성한다.
7. tiny normal replay를 clean environment에서 최소 5회 실행한다.
8. exact-field stability와 numeric variation을 분석한다.
9. reviewed normal/failure baseline을 생성한다.
10. PR/main replay / tiny-small과 nightly replay / representative CI를 연결한다.

Merged-map Ground Truth는 요구하지 않는다.
이 Goal의 목적은 accuracy evaluation이 아니라 reproducible regression baseline 완성이다.

기존 RuntimeClient/public boundary, architecture invariant, API/ABI를 변경하지 않는다.
```

### Goal 05 Calibration

``` text
현재 구현된 Goal 05 benchmark infrastructure에 기능을 추가하지 말고
production calibration과 baseline 승격을 완료하라.

1. immutable benchmark container와 machine class를 확정한다.
2. GCC 12 clean nightly를 독립 5회 실행한다.
3. Clang 15 clean nightly를 독립 5회 실행한다.
4. raw/aggregate/pair report와 content hash를 보존한다.
5. median/p95/MAD, RSS, owner metric variance를 분석한다.
6. regression threshold를 review한다.
7. docs/post_freeze_results/performance_baseline.json을 별도 baseline commit으로 추가한다.
8. full ASan/UBSan 및 TSan clean lane을 다시 실행해 모두 green임을 확인한다.
9. 승인 baseline을 사용하는 PR performance regression gate를 활성화한다.

Baseline이나 threshold를 CI가 자동 생성/확대하지 않도록 한다.
```

## 18. 최종 진행 그림

``` text
Goal 03 P0 ─────┐
                ├── Production Validation Core Complete
Goal 05 P0 ─────┘
        ↓
Goal 03 representative
        ↓
Goal 05 large-external
        ↓
optional GPU / real-driver
        ↓
Goal 06+ / feature development
```

지금은 새로운 기능을 추가하기보다, 이미 만들어진 architecture와
validation infrastructure를 **실제 데이터와 clean calibration evidence로
공식화할 시점**이다.
