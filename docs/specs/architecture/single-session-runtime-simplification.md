# OpenLMM 단일 세션 런타임 단순화 구현 명세

상태: **구현 완료 — 최종 검증 완료**
작성일: 2026-08-17
구현 기준점: `2e32ff159027b22b6a930b10e5ef62ac1308f00e`
권장 작업 브랜치: `feat/single-session-runtime`

## 구현 결과

`feat/single-session-runtime`에는 이 명세의 Phase 0~6이 반영됐다.

- 공개 control plane은 `RuntimeClient`와 `JobHandle`만 사용하며 `SessionId`,
  `RuntimeSessionClient`, registry, `CreateSession`/`CloseSession`/`ListSessions`는 없다.
- committed state는 `RuntimeStateStore`가 단 하나만 소유하며 root/DataLoad 변경은
  private candidate를 준비한 뒤 revision-guarded file/state barrier에서 교체한다.
- GUI, ROS, Batch는 같은 unkeyed runtime contract를 사용한다. ROS는 설치된
  `client`/`gui` component만으로 PCL/GTSAM 탐색 없이 build된다.
- ABI v1만 남았고 ABI v2 source, target, configuration, CI, current 문서 참조는 없다.
- 마지막 검증은 normal CTest 28/28, TSAN runtime/controller/gui gate 5/5,
  installed package consumer, installed ROS consumer, test1/test2 ordered replay로 수행했다.

이후의 작업은 이 명세의 구현이 아니라 별도 기능 변경으로 취급한다.

## 1. 결정 요약

OpenLMM의 본질적인 요구사항은 여러 로봇을 하나의 작업에서 처리하는
**멀티에이전트**이지, 서로 독립적인 여러 mapping 작업을 한 프로세스에서 동시에
운영하는 **멀티세션**은 아니다.

따라서 기준 커밋의 안전성 개선은 유지하되 런타임 모델을 다음과 같이 축소한다.

```text
기존

GUI / ROS / SDK
        |
        v
RuntimeClient
        |
        v
RuntimeSessionClient ---- SessionId
        |
        v
RuntimeService
  └─ session registry
       ├─ Session A
       ├─ Session B
       └─ Session C

목표

GUI / ROS / Batch / SDK
          |
          v
RuntimeClient (PImpl, 공개 façade)
          |
          v
RuntimeService
  ├─ active RuntimeState 1개
  └─ replacement candidate 최대 1개(외부 비공개)
          |
          v
PipelineController -> StageCoordinator -> Executors
```

공개 API에서 `SessionId`와 session registry를 제거한다. 다만 root/DataLoad 설정 교체
중 오래된 job과 event가 새 상태에 적용되는 것을 막기 위해 내부 `RuntimeEpoch`은
유지한다. 즉 **멀티세션은 제거하지만 세대 구분까지 제거하지는 않는다.**

## 2. 기준점 선택

새 브랜치는 다음 명령으로 만든다.

```bash
git switch -c feat/single-session-runtime \
  2e32ff159027b22b6a930b10e5ef62ac1308f00e
```

`2e32ff1`을 기준으로 선택하는 이유는 다음과 같다.

- Plugin ABI v2 제거가 끝나 ABI v1만 남아 있다.
- `GlobalConfig` 대신 bounded immutable config snapshot을 사용한다.
- schema validation과 typed config decode가 구현돼 있다.
- config file과 runtime state의 failure-atomic transaction이 있다.
- stage 실행이 coordinator와 executor로 분리돼 있다.
- `Result<T>`, `AlgorithmExecutionContext`, strong `AgentId` 계약이 있다.
- alignment policy, descriptor artifact/engine 경계가 분리돼 있다.
- install/export/public-header 경계가 정리돼 있다.
- FreeDOM FOV 정책 수정과 관련 검증이 포함돼 있다.

`69c6958`은 멀티세션 도입 전이라는 장점은 있지만, 그 뒤에 추가된 config transaction,
알고리즘 경계, package 및 correctness 수정을 다시 선별 이식해야 한다. 이식 누락
위험보다 `2e32ff1`에서 멀티세션 부분을 구조적으로 제거하는 편이 안전하다.

## 3. 목적

1. 동시에 외부에 노출되는 runtime을 정확히 하나로 제한한다.
2. 공개 control-plane API에서 `SessionId`를 제거한다.
3. GUI, ROS, Batch가 동일한 `RuntimeClient` 계약을 사용하게 한다.
4. root/DataLoad 설정 변경은 기존 상태를 유지한 채 candidate를 검증한 후 한 번에
   교체한다.
5. module 설정 변경은 기존 config/state/file transaction 의미를 유지한다.
6. 기존 stage candidate/commit, artifact revision, cancellation, event 의미를 보존한다.
7. session registry와 binding 때문에 발생한 lifecycle 및 동시성 위험을 제거한다.
8. ABI v1과 설치 패키지 경계를 그대로 유지한다.

## 4. 비목표

다음은 이 작업의 범위가 아니다.

- loop detection, registration, optimization, dynamic removal 수학 변경
- 멀티에이전트 처리 제거
- Plugin ABI v2 재도입
- 분산 worker 또는 원격 runtime 구현
- 여러 사용자나 여러 dataset의 동시 실행
- GUI 레이아웃 재설계
- 범용 DAG workflow engine 도입
- PCL/Eigen/GTSAM 내부 의존성 전면 제거
- 파일 transaction의 전원 장애까지 보장하는 완전한 crash durability

## 5. 유지·축소·제거 범위

### 5.1 그대로 유지할 기능

- immutable `BootstrapConfigSnapshot`
- bounded single-snapshot config loading
- declarative schema, migration, typed decoding
- `ConfigCandidate -> validate -> stage files -> commit` transaction
- `PipelineController`의 비동기 job, cancellation, event journal
- `StageCoordinator`와 DataLoad/Alignment/Optimize/MapUpdate/Save executor
- `SessionPayload`에 해당하는 committed payload의 immutable snapshot 의미
- `ArtifactRepository` revision 및 invalidation
- alignment feedback, visualization snapshot
- process 내 bounded agent parallelism
- ABI v1 plugin loading과 preflight
- `RuntimeClient` PImpl 및 public package façade
- public-header allowlist와 package consumer gate

### 5.2 단일 런타임 의미로 축소할 기능

| 현재 개념 | 목표 개념 | 비고 |
|---|---|---|
| `SessionState` | `RuntimeState` | 불변 committed state 의미 유지 |
| `SessionManager` | `RuntimeStateStore` | 한 개의 state와 revision만 소유 |
| session별 governor | process-wide `ResourceGovernor` | 한 runtime의 전체 예산 |
| session replacement | active/candidate state replacement | 외부 session 생성 없음 |
| `ExpectedRevision{session, config}` | `ExpectedRevision{runtime, config}` | 이름만 명확화, 두 revision 의미 유지 |
| session-scoped artifact | runtime-scoped artifact | revision/invalidation 의미 동일 |
| `BoundJob{SessionId, JobId}` | opaque `JobHandle` | 내부 epoch로 stale 작업 차단 |

### 5.3 최종적으로 제거할 기능

- public `SessionId`
- `RuntimeSessionClient`
- session registry/map
- `CreateSession`, `CloseSession`, `ListSessions`
- `maximum_sessions`
- 모든 API의 반복 `SessionId` 인자
- 여러 session 간 resource admission/accounting
- old/new session subscription fan-out
- GUI/ROS의 session binding 교체 계층
- session별 output registry
- ABI v2 target, source, 설정, 문서 및 CI 참조

`SessionManager`를 첫 커밋에서 바로 삭제하거나 이름만 바꾸지 않는다. 기능 migration이
완료된 후 한 state 소유자라는 사실이 확인됐을 때 `RuntimeStateStore`로 축소한다.

## 6. 핵심 불변식

다음 불변식은 모든 단계에서 유지해야 한다.

### RT-001 단일 committed state

프로세스에는 외부에서 관찰 가능한 committed `RuntimeState`가 정확히 0개 또는 1개다.
초기 open 전에는 0개, open 성공 후에는 1개다.

### RT-002 candidate 비공개

replacement candidate는 최대 1개이며 snapshot, event, visualization, config query에서
관찰되지 않는다. 성공한 단일 commit 이후에만 active가 된다.

### RT-003 실패 원자성

bootstrap, plugin preflight, schema validation, output 준비, config 파일 설치 또는 state
commit 중 하나라도 실패하면 다음 값이 이전과 동일해야 한다.

- active state pointer 및 revision
- config canonical document와 파일
- artifact snapshot
- resident resource reservation owner
- GUI/ROS가 보는 runtime epoch
- event subscription의 active source

### RT-004 세대 격리

replacement 성공 시 내부 epoch가 증가한다. 이전 epoch의 job, callback, alignment response,
config receipt는 새 state를 변경할 수 없다.

### RT-005 한 command 한 commit

성공한 stage 또는 config command는 committed runtime revision을 정확히 한 번 증가시킨다.
실패와 cancellation은 revision을 증가시키지 않는다.

### RT-006 disk/state 일치

설정 적용 성공 응답 시 canonical config file과 committed state가 같은 candidate를 가리킨다.
GUI가 파일을 먼저 쓰고 runtime에 적용하는 경로는 금지한다.

### RT-007 callback 안전성

event callback은 runtime/controller/state lock을 잡은 채 호출하지 않는다. subscription
`Reset()` 반환 뒤에는 해당 callback이 다시 호출되지 않는다. callback 안의 self-reset은
교착하지 않는다.

### RT-008 알고리즘 상태 보존

executor는 committed state를 직접 변경하지 않고 candidate만 만든다. payload,
artifact, output file은 coordinator의 한 commit 경계에서만 공개된다.

## 7. 공개 API 명세

### 7.1 타입

```cpp
struct RuntimeRevision {
  uint64_t state;
  uint64_t config;
};

struct JobHandle {
  uint64_t value;  // process lifetime 동안 재사용하지 않는 opaque ID
};

struct RuntimeSnapshot {
  RuntimeRevision revision;
  RuntimeStatus status;
  std::optional<JobHandle> active_job;
  std::vector<ExecutionEvent> recent_events;
  ArtifactSnapshot artifacts;
};
```

- 공개 snapshot과 event에 `SessionId`가 없어야 한다.
- `JobHandle`은 replacement 뒤에도 재사용하지 않는 process-lifetime monotonic 값으로 한다.
- 내부 구현은 `RuntimeEpoch`과 controller-local job ID를 추가로 보유할 수 있다.
- wire compatibility가 필요하면 `JobHandle`을 `{epoch, local_id}`의 opaque value로 구현한다.

### 7.2 RuntimeClient

```cpp
class RuntimeClient {
 public:
  static Result<RuntimeClient> Open(const BootstrapRequest& request);

  Result<JobHandle> Submit(const ExecutionRequest& request);
  Result<void> Cancel(JobHandle job);
  Result<void> Wait(JobHandle job);

  Result<RuntimeSnapshot> Snapshot() const;
  Result<std::vector<NodeDescriptor>> NodeDescriptors() const;
  Result<VisualizationSnapshot> Visualization(const AgentId& agent) const;
  Result<std::optional<AlignmentFeedbackSnapshot>>
  AlignmentFeedback() const;
  Result<void> RespondToAlignment(
      JobHandle job, const AlignmentResponse& response);
  Result<void> SetAlignmentFeedbackEnabled(bool enabled);

  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate,
      const ExpectedRevision& expected);

  Result<RuntimeReplaceReceipt> ReplaceRootConfig(
      const BootstrapConfigCandidate& candidate,
      const ExpectedRevision& expected);

  Result<ExecutionEventSubscription> SubscribeEvents(
      ExecutionEventCallback callback);
};
```

`RuntimeClient`는 외부 SDK와 설치 패키지의 안정적인 PImpl façade로 유지한다. 내부
`RuntimeService` header, controller, executor, plugin host는 공개하지 않는다.

### 7.3 module config와 root config 구분

module config 적용과 root/DataLoad config 교체는 같은 API로 위장하지 않는다.

#### module config

- 현재 `RuntimeState`의 일부 domain만 변경한다.
- `ExpectedRevision`을 확인한다.
- candidate typed decode와 plugin preflight를 수행한다.
- module file과 runtime state를 같은 commit barrier에서 적용한다.
- 성공 시 state/config revision을 증가시킨다.

#### root/DataLoad config

- active state를 직접 변경하지 않는다.
- 별도 candidate state, output namespace, reservation을 준비한다.
- 준비 중 query는 계속 이전 committed state를 반환한다.
- active job 또는 다른 operation lease가 있으면 `Busy`로 즉시 거부한다.
- 준비와 파일 설치가 성공하면 한 번에 active pointer와 epoch를 교체한다.
- 실패하면 이전 state와 disk를 그대로 유지한다.

## 8. 내부 상태 모델

```cpp
class RuntimeService::Impl {
  mutable std::mutex mutex_;
  RuntimeLifecycle lifecycle_;
  uint64_t epoch_;
  std::unique_ptr<RuntimeInstance> active_;
  bool replacement_in_progress_;
  uint64_t next_job_handle_;
  ResourceGovernor governor_;
};

class RuntimeInstance {
  RuntimeStateStore state_store_;
  PipelineController controller_;
  StageCoordinator coordinator_;
  EventJournal events_;
  OutputNamespace output_;
};
```

`RuntimeInstance`는 여러 개를 registry에 넣는 단위가 아니다. `active_`와 준비 중인
지역 변수 candidate에만 존재한다.

### 8.1 lifecycle

```text
Closed
  -> Opening
  -> Ready

Ready
  -> Running
  -> Cancelling
  -> Ready

Ready
  -> Replacing
  -> Ready(new epoch)       on success
  -> Ready(previous epoch)  on failure

Ready/Running
  -> Closing
  -> Closed

Any state
  -> Fatal
```

- `Replacing` 동안 `Submit`, `ApplyConfig`, feedback mutation은 `Busy`로 거부한다.
- `Snapshot`과 read-only query는 이전 committed state를 계속 반환한다.
- active job이 있을 때 replacement는 기다리지 않고 `Busy`로 거부한다.
- explicit `Cancel -> Wait -> Replace`는 caller가 수행한다.
- protocol corruption 등 `Fatal` 상태는 명시적인 reopen 외에는 복구하지 않는다.

### 8.2 operation lease

각 mutating operation은 시작 시 다음 값을 캡처한다.

```cpp
struct OperationLease {
  uint64_t epoch;
  OperationKind kind;
};
```

- lease 취득과 replacement 시작은 같은 mutex에서 직렬화한다.
- 작업 완료 시 epoch가 다르면 결과를 commit하지 않고 `StaleOperation`으로 반환한다.
- plugin 호출, 파일 I/O, callback 동안 runtime mutex를 유지하지 않는다.

## 9. replacement transaction

root/DataLoad 설정 교체는 다음 순서로 수행한다.

```text
1. expected revision 및 Ready/idle 상태 확인
2. replacement_in_progress = true
3. bounded canonical config candidate 검증
4. plugin ABI v1 preflight
5. 새 RuntimeState와 algorithms 준비
6. resource/output reservation 준비
7. 새 event/controller 경계 준비(아직 구독 공개 금지)
8. canonical root/module file temp staging
9. registry가 아닌 active pointer의 commit barrier 진입
10. file set install
11. active = candidate, epoch++, revision receipt 확정
12. 새 event source 공개
13. 이전 instance를 callback worker 밖에서 종료
14. replacement_in_progress = false
```

3~8단계 실패 시 candidate만 폐기한다. 10단계 실패 시 file transaction rollback 후 이전
active를 유지한다. 11단계 이후의 backup cleanup 실패는 commit을 되돌리지 않고
structured recovery warning을 남긴다.

### 9.1 자원 예산

replacement 준비 중에는 old active와 candidate가 잠시 공존한다. 멀티세션을 제거했다고
이 peak를 무시해서는 안 된다.

- resident active payload는 계속 accounting한다.
- 동일 dataset 교체에는 old reservation credit을 명시적으로 적용할 수 있다.
- candidate의 실제 reservation을 commit 때 active owner로 이동한다.
- 준비 실패 시 candidate reservation과 임시 output을 모두 해제한다.
- algorithm/plugin 내부 thread와 heavy-phase admission은 process-wide governor를 사용한다.

## 10. 동시성 및 소멸 계약

### 10.1 lock 원칙

1. runtime lifecycle lock 아래에서 controller 호출, plugin 호출, 파일 I/O를 하지 않는다.
2. state store commit lock 아래에서 callback을 호출하지 않는다.
3. callback 목록은 slot을 확보한 뒤 lock 밖에서 호출한다.
4. subscription reset은 해당 slot의 `in_flight == 0`을 기다린다.
5. callback에서 자기 subscription을 reset하면 현재 호출만 drain 대상에서 제외한다.
6. lock 순서를 문서와 debug assertion으로 고정한다.

권장 lock 순서:

```text
runtime lifecycle -> state store -> controller snapshot -> subscriber slot
```

역순 취득은 금지한다.

### 10.2 기준점의 알려진 lifecycle 결함 처리

구현 과정에서 다음 경로를 회귀 테스트로 먼저 고정한다.

#### Wait와 close/replacement

기준 구현은 command-in-progress drain 뒤 active job을 cancel하면, alignment feedback을
기다리는 `Wait()`와 close가 서로 기다릴 수 있다. 단일 runtime에서는 다음과 같이
정의한다.

- replacement는 active job/Wait이 있으면 기다리지 않고 `Busy`를 반환한다.
- destructor/explicit close는 먼저 cancellation을 발행하고 그다음 wait/join한다.
- callback과 feedback 대기자는 cancellation을 terminal result로 관찰한다.

#### callback worker의 self-join

마지막 `RuntimeClient` owner가 event callback 안에서 파괴되거나 move assignment로 기존
Impl을 교체해도 controller worker가 자기 자신을 join해서는 안 된다.

- destructor뿐 아니라 move assignment도 같은 deferred teardown 경계를 사용한다.
- callback worker에서는 소유 state만 분리하고 neutral cleanup thread/executor에 종료를
  넘긴다.
- self-join은 assert 또는 `std::terminate`에 의존하지 않고 구조적으로 불가능해야 한다.

## 11. Adapter 명세

### 11.1 GUI

- `RuntimeSessionClient`와 session binding을 사용하지 않는다.
- `RuntimeClient` 한 개를 소유한다.
- root/DataLoad Apply는 `ReplaceRootConfig`를 호출한다.
- module Apply는 authoritative `Snapshot()` revision으로 `ApplyConfig`를 호출한다.
- GUI가 config 파일을 먼저 저장해서는 안 된다.
- replacement 성공 시 model/event queue epoch를 초기화하고 즉시 snapshot을 동기화한다.
- live event, snapshot job, recent event, cancel, feedback은 같은 `JobHandle` namespace를 쓴다.

### 11.2 ROS

- `RuntimeClient` 한 개만 소유한다.
- public client/gui component만 링크하고 internal server header를 include하지 않는다.
- typed action/service/message 계약을 유지한다.
- action goal은 `JobHandle`에 묶으며 cancel/wait/feedback도 같은 handle을 사용한다.
- 동시 goal은 하나만 admit하고 두 번째 goal은 submit 전에 거부한다.
- PCL/GTSAM을 설치한 환경에 의존하지 않는 lightweight adapter gate를 유지한다.

### 11.3 Batch

Batch도 별도 `MapServer` 실행 경계를 새로 만들지 않는다.

```cpp
auto runtime = RuntimeClient::Open(request);
auto job = runtime.Value().Submit(ExecutionRequest::RunAll());
auto result = runtime.Value().Wait(job.Value());
```

Batch는 동기적으로 wait하고 exit code를 변환하는 얇은 adapter만 담당한다.

## 12. 단계별 구현 계획

### Phase 0 — 기준점과 회귀 증거 고정

작업:

- `2e32ff1`에서 새 브랜치 생성
- clean configure/build/test
- test1/test2 ordered replay와 pose/descriptor/PCD baseline 저장
- lifecycle fault test를 변경 전 상태에서도 재현 가능한 형태로 추가

완료 조건:

- full CTest PASS
- TSAN controller/runtime/gui gate 결과 기록
- test1/test2 descriptor count와 pose tolerance 기록
- public/install/ROS consumer gate 결과 기록

### Phase 1 — unkeyed façade 추가

기존 multi-session 내부를 당장 삭제하지 않고 한 session에 고정하는 unkeyed façade를
먼저 추가한다.

작업:

- `RuntimeClient`에 SessionId 없는 API 추가
- 내부 임시 adapter가 유일 session으로 전달
- process-lifetime `JobHandle` 도입
- 기존 session-keyed API를 deprecated/internal로 격리

완료 조건:

- 기존 동작과 revision/event 결과 동일
- 공개 header 신규 API에 `SessionId` 없음
- old/new API equivalence fixture PASS

### Phase 2 — GUI/ROS/Batch migration

작업:

- GUI를 unkeyed `RuntimeClient`로 전환
- ROS를 unkeyed `RuntimeClient`로 전환
- Batch를 동일 façade로 전환
- config pre-write 제거 및 root/module 경로 분리

완료 조건:

- adapter source에 `RuntimeSessionClient`, direct `MapServer`, direct
  `PipelineController` 사용 0건
- GUI Apply, ROS action, Batch run E2E PASS
- replacement 뒤 model/event/job namespace 일치

### Phase 3 — registry를 단일 RuntimeInstance로 교체

작업:

- session map을 `unique_ptr<RuntimeInstance> active_`로 교체
- `maximum_sessions`, `ListSessions` 제거
- process-wide governor로 통합
- internal epoch와 operation lease 도입
- candidate replacement transaction 구현

완료 조건:

- 외부에서 두 runtime을 동시에 생성/조회할 방법이 없음
- active state 1개/candidate 1개 invariant fixture PASS
- replacement failure 때 old state/file/output/resource 동일

### Phase 4 — session API와 binding 제거

작업:

- `RuntimeSessionClient` 삭제
- public `SessionId` 삭제
- `CreateSession`, `CloseSession`, session-keyed overload 삭제
- session event fan-out 및 registry cleanup 제거
- package allowlist/export 갱신

완료 조건:

```bash
rg -n "RuntimeSessionClient|ListSessions|maximum_sessions|CreateSession|CloseSession" \
  open_lmm ros
```

결과가 migration 문서와 삭제 검증 fixture 외에는 0건이어야 한다.

### Phase 5 — state 명명과 내부 단순화

작업:

- `SessionState` -> `RuntimeState`
- `SessionManager` -> `RuntimeStateStore`
- session-scoped DTO를 runtime-scoped DTO로 변경
- 중복 façade, dead overload, transitional adapter 제거
- generic port 계층은 실제 외부 대체점이 없으면 축소하되 한 커밋에서 대규모 rename과
  동작 변경을 섞지 않는다.

완료 조건:

- 한 state store만 committed state를 소유
- controller/coordinator가 서로 다른 authoritative revision을 보유하지 않음
- transitional adapter와 compatibility overload 0건

### Phase 6 — 최종 회귀 및 문서 동기화

작업:

- full normal/TSAN/package/ROS/replay gate 실행
- architecture boundary test를 단일 runtime 규칙으로 갱신
- `docs/current`를 실제 구현과 동기화
- 멀티세션과 ABI v2 설명 제거

## 13. 필수 테스트

### 13.1 state/config transaction

- module config 성공 시 file/state revision 한 번 증가
- stale runtime/config revision 거부
- wrong-domain candidate 거부
- root replacement schema/plugin/output prepare 실패 시 old state 완전 동일
- root config install rename/backup fault 시 old file/state 유지
- post-install cleanup failure 시 new state 유지 + recovery warning
- duplicate temp/final/backup path 사전 거부

### 13.2 lifecycle/concurrency

- active job 중 replacement 즉시 `Busy`
- `Wait()` 중 replacement 즉시 `Busy`, bounded timeout
- alignment feedback wait 중 close가 cancel 후 정상 종료
- replacement 준비 중 Submit/ApplyConfig/feedback mutation 거부
- subscription Reset이 in-flight callback 종료까지 대기
- callback self-reset 무교착
- callback에서 마지막 client owner 해제 무교착/no self-join
- callback에서 move assignment 무교착/no self-join
- old epoch event/response/config receipt가 new epoch에 적용되지 않음
- TSAN 반복 실행에서 race report 0건

### 13.3 resource/output

- active payload resident bytes accounting
- replacement candidate admission 성공/실패와 release
- 같은 dataset reload에서 replacement credit의 false rejection 방지
- 실패 candidate output directory/temp file 정리
- 성공 replacement 뒤 old reservation/output owner 해제

### 13.4 adapter

- GUI module Apply가 authoritative snapshot revision 사용
- GUI root Apply가 file pre-write 없이 replacement 수행
- replacement 성공 직후 GUI model epoch reset/snapshot 동기화
- synchronous queued event부터 terminal event까지 동일 `JobHandle`
- ROS submit/cancel/wait/feedback typed E2E
- ROS concurrent goal admission 1개만 성공
- Batch RunAll 결과가 GUI/ROS와 같은 stage/revision 의미

### 13.5 algorithm regression

- controller/runtime/stage/orchestration/safety tests
- test1/test2 ordered replay
- descriptor 총수 및 agent별 수 동일
- pose translation/rotation 기존 tolerance 통과
- loop pair와 artifact revision 동일
- map PCD의 point count/finite field/허용 오차 비교
- FreeDOM FOV 정책과 결과 회귀 없음

### 13.6 package

- public header exact allowlist
- 모든 public header 개별 TU compile
- contracts/client/plugin-sdk component-only consumer
- internal runtime/plugin host target 비export
- installed ROS가 client component로 clean build
- ROS adapter gate에서 PCL/GTSAM discovery 비활성
- ABI v1 plugin fixture PASS, ABI v2 source/target/reference 0건

## 14. 빌드 및 검증 명령 기준

일반 build parallelism은 16을 사용한다.

```bash
colcon build --packages-select open_lmm --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 16

cmake --build build/open_lmm -j16
ctest --test-dir build/open_lmm --output-on-failure -j16
```

TSAN은 공식 suppression을 사용하되 OpenNI2 등 알려진 외부 DSO 경고만 scoped suppression
대상으로 둔다. sanitizer failure를 전역 `exitcode=0`으로 숨기지 않는다.

## 15. 커밋 구성 권장안

변경은 다음 단위로 나눈다.

1. `test: capture single-runtime migration baseline`
2. `refactor: add unkeyed runtime facade and opaque job handles`
3. `refactor: migrate GUI ROS and batch to the runtime facade`
4. `refactor: replace the session registry with one runtime instance`
5. `refactor: make root configuration replacement failure-atomic`
6. `refactor: remove session-keyed clients and APIs`
7. `refactor: rename session state ownership to runtime state`
8. `test: close single-runtime lifecycle and package gates`
9. `docs: describe the single-session runtime architecture`

동작 변경, 대규모 rename, 파일 이동을 한 커밋에 섞지 않는다. 각 커밋은 build 가능하고
해당 단계의 targeted test를 통과해야 한다.

## 16. 완료 조건

다음 조건을 모두 만족해야 작업을 완료로 판정한다.

- 공개 runtime API에 `SessionId`가 없다.
- `RuntimeSessionClient`, session registry, `maximum_sessions`, `ListSessions`가 없다.
- committed `RuntimeState`는 최대 1개다.
- candidate는 최대 1개이며 commit 전 외부에 보이지 않는다.
- GUI, ROS, Batch가 동일한 `RuntimeClient`를 사용한다.
- GUI/ROS/Batch가 `MapServer`나 `PipelineController`를 직접 생성하지 않는다.
- module config 및 root replacement가 disk/state failure-atomic하다.
- replacement 중 active job은 명시적으로 거부되며 교착하지 않는다.
- old epoch job/event/feedback/config operation이 새 state를 변경하지 못한다.
- callback reset, callback 내 destruction/move assignment가 안전하다.
- executor candidate/commit과 artifact revision 의미가 기준점과 동일하다.
- process-wide resource budget과 replacement peak accounting이 검증된다.
- ABI v1만 남고 ABI v2 코드, target, 설정, CI, current 문서 참조가 없다.
- normal full CTest, TSAN, package consumer, installed ROS gate가 통과한다.
- test1/test2 replay 결과가 기준 tolerance 안에 있다.

LOC 감소량은 완료 조건이 아니다. 목표는 코드를 무조건 줄이는 것이 아니라, 범용
멀티세션 lifecycle을 제거하면서 현재의 correctness 경계를 보존하는 것이다.

## 17. 최종 목표 구조

```text
Public adapters
  GUI / ROS / Batch / SDK
              |
              v
       RuntimeClient (PImpl)
              |
              v
       RuntimeService
       - lifecycle
       - one active RuntimeInstance
       - one private candidate
       - process-wide governor
       - epoch / operation leases
              |
              v
       RuntimeStateStore
       PipelineController
       StageCoordinator
       ArtifactRepository
              |
              v
       Stage Executors
       Core algorithms
       ABI v1 plugins
```

이 구조에서 단일세션은 “상태를 바로 덮어쓰는 단순 전역 객체”를 뜻하지 않는다.
하나의 committed state, 비공개 candidate, 원자적 replacement, stale operation 차단을
가진 **단일 active runtime**을 뜻한다.
