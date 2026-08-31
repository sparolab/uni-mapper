# H1 Runtime·ROS·Plugin Production Hardening 구현명세서

- 상태: `PLANNED / NOT IMPLEMENTED`
- 기준선: `dev/reorder` / `fa0c8e4`
- 구현 순서: thread safety → RuntimeClient cleanup → ROS early-cancel → plugin generation
- 결과 문서: `docs/apps/results/13_production_hardening_result.md` (구현 완료 시 작성)

## 1. 목적과 완료 경계

H1은 이미 분리된 core/application package 구조를 다시 변경하지 않고, 자원 고갈과
callback 수명, ROS action race, stale algorithm DSO가 process 종료나 잘못된 실행으로
이어지는 경계를 강화한다.

다음 네 work package를 반드시 순서대로 구현한다.

```text
H1-1 thread creation safety
  ↓ green gate
H1-2 RuntimeClient retirement ownership
  ↓ green gate
H1-3 ROS accepted-goal early cancellation
  ↓ green gate
H1-4 algorithm plugin generation validation
```

각 work package는 독립적으로 revert 가능한 green commit이어야 한다. 앞 단계의 fault
test와 기존 regression이 통과하기 전에는 다음 단계의 production source를 수정하지
않는다. H1 완료는 Goal 09 공급망, crash durability 또는 hosted release evidence 완료를
의미하지 않는다.

## 2. 보존할 architecture 계약

- `RuntimeStateStore`만 committed runtime state를 소유한다.
- thread 준비 실패는 candidate 단계 실패이며 job, event, revision 또는 file commit을
  게시하지 않는다.
- committed receipt가 존재하면 late cancellation보다 success가 우선한다.
- `RuntimeClient` PImpl, public object layout과 runtime single-owner 모델을 유지한다.
- ROS와 Python은 public `RuntimeClient`만 소비하는 leaf adapter로 남는다.
- plugin host가 DSO와 instance lifetime을 소유하며 destroy가 `dlclose`보다 먼저 실행된다.
- plugin ABI v1은 same-toolchain model이다. 이번 검사는 portable ABI 보장을 추가하지
  않는다.

공개 계약 변화는 `Error::Code::kResourceExhausted`의 끝자리 추가뿐이다. enum underlying
type, `Error` layout, RuntimeClient exported signature와 SONAME은 변경하지 않는다. ROS
action/service/message wire shape와 plugin ABI-v1 C struct도 변경하지 않는다.

## 3. H1-1 Thread 생성 실패 안전성

### 3.1 공통 launcher와 오류 계약

`foundation/concurrency`에 production `std::thread` 생성과 deterministic failure injection을
같은 경계로 모으는 private launcher를 둔다. installed header나 public target으로
노출하지 않는다.

```cpp
using ThreadTask = std::function<void()>;
using ThreadLauncher = std::function<std::thread(ThreadTask)>;
```

기본 launcher는 `std::thread(std::move(task))`를 반환한다. 테스트 launcher는 호출 횟수를
세어 지정된 N번째 호출에서 `std::system_error`를 던지고 나머지는 실제 thread를 만든다.
production code에는 환경변수나 global fault toggle을 추가하지 않는다.

`Error::ResourceExhausted(detail)` factory를 추가한다. `Result`를 반환하는 submit 경계는
thread launch의 `std::system_error`를 `kResourceExhausted`로 변환한다. constructor처럼
`Result`를 반환할 수 없는 경계는 이미 시작한 resource를 모두 정리한 뒤 원래
`std::system_error`를 전파한다. unknown exception을 resource exhaustion으로 잘못
분류하지 않는다.

### 3.2 PipelineController startup transaction

`PipelineController`에 기본 launcher를 사용하는 기존 constructor와 private/test용 launcher
주입 constructor를 둔다. 외부 public API에는 launcher가 나타나지 않는다.

`submit()`은 다음 순서로 동작한다.

1. `command_mutex_`로 admission을 직렬화하고 port, fatal health, maintenance와 active job을
   검증한다.
2. 이전 terminal worker가 있으면 state mutex 밖에서 join한다. 이 시점까지 이전 terminal
   snapshot과 event journal은 유지한다.
3. job ID를 증가시키거나 `job_`을 변경하기 전에 cancellation token, execution context,
   result slot과 start gate를 준비한다.
4. execution worker를 만들고 start gate의 `pending/committed/aborted` 상태를 기다리게 한다.
5. execution worker를 직접 lambda capture로 이동하지 않는다. rollback guard가 소유하는
   shared worker state를 lifecycle worker가 참조하게 해 두 번째 thread constructor 실패
   중 joinable thread destructor가 실행되지 않게 한다.
6. 두 worker가 모두 만들어진 뒤에만 job ID, queued snapshot, cancellation owner와 terminal
   watermark를 commit하고 lifecycle worker를 `worker_`에 이전한다.
7. `kJobQueued`를 journal/callback queue에 게시한 다음 start gate를 `committed`로 전환한다.

첫 번째 launch 실패는 state를 전혀 변경하지 않는다. 두 번째 launch 실패는 gate를
`aborted`로 바꾸고 첫 worker를 깨운 뒤 join한다. aborted worker는 job work, `kJobStarted`,
callback 또는 terminal commit을 수행하지 않는다. rollback destructor와 join 경로는
`noexcept`여야 한다.

실패 전후 비교 대상은 다음과 같다.

- 이전 `job_`과 `worker_`
- `next_job_id_`, `terminal_event_completed_job_id_`
- cancellation token과 telemetry
- alignment feedback publication state
- recent event sequence와 pending callback queue
- committed runtime/config revision

실패한 submit은 handle을 반환하지 않으며 새 queued/started/terminal event도 남기지 않는다.
동일 controller에서 다음 submit이 ID gap 없이 정상 수행돼야 한다.

### 3.3 BoundedExecutor constructor rollback

`BoundedExecutor`도 같은 private launcher를 받을 수 있게 한다. 정상 constructor 반환
전에는 task admission이 외부에 노출되지 않는다.

worker loop는 기존 condition variable에서 대기한다. N번째 worker 생성이 실패하면
constructor rollback guard가 lock 아래 `accepting_=false`, `stopping_=true`를 게시하고
모든 worker를 깨운 뒤 생성된 worker를 전부 join한다. queue는 아직 외부에 노출되지
않았으므로 task completion이나 cancellation event를 합성하지 않는다.

정상 생성 이후의 queue capacity, submission backpressure, queued cancellation, exception
conversion과 destructor drain semantics는 변경하지 않는다.

### 3.4 H1-1 admission

- Pipeline 첫 번째/두 번째 launch failure가 `kResourceExhausted`로 반환됨
- 실패 전후 snapshot과 event journal이 동일함
- 실패 뒤 다음 submit과 wait가 성공함
- BoundedExecutor 첫/중간/마지막 worker failure에서 생성 worker 수와 join 수가 같음
- terminate, joinable-thread unwind와 background thread leak이 없음
- controller/resource targeted ASan+UBSan 및 TSan 통과

## 4. H1-2 RuntimeClient cleanup ownership

### 4.1 단일 retirement owner

현재 callback 내부 마지막 `RuntimeClient` 파괴와 Python GC fallback이 각각 detached thread를
만들고, thread 생성 실패 시 owner를 leak한다. 이를 process-private
`RuntimeRetirementCoordinator` 하나로 대체한다.

coordinator는 H1-1 launcher로 만든 joinable worker 한 개, mutex, condition variable와
intrusive FIFO를 소유한다. 첫 RuntimeClient construction에서 coordinator 생성에 실패하면
부분 `Impl`을 만들지 않고 `std::system_error`를 호출자에게 전파한다.

각 `RuntimeClient::Impl`은 retirement node를 하나 포함한다. callback에서의 handoff는
다음만 수행한다.

1. `impl_.release()`로 exact owner를 분리한다.
2. embedded node가 아직 enqueue되지 않았음을 assert/검증한다.
3. coordinator lock 아래 raw owner node를 FIFO에 연결하고 worker를 깨운다.

이 경로는 allocation-free이고 `noexcept`다. node cardinality는 handoff된 live Impl 수를
넘지 않으며 한 Impl은 한 번만 enqueue할 수 있다. worker는 node를 queue에서 제거한 뒤
`Impl`을 삭제하고 pending/completed/peak counter를 갱신한다. callback이 아직 끝나지 않은
경우 `RuntimeService::Close()`의 기존 callback/lifecycle barrier에서 기다리며, 새 owner나
두 번째 runtime authority를 만들지 않는다.

coordinator shutdown은 accepting을 닫고 FIFO가 빌 때까지 cleanup한 뒤 worker를 join한다.
pending node를 폐기하거나 process-exit leak으로 바꾸지 않는다. private diagnostics/test
barrier는 pending, completed, peak와 idle만 제공하며 installed RuntimeClient API에는
노출하지 않는다.

### 4.2 RuntimeClient destructor와 move

- event callback 밖의 destructor는 지금처럼 동기적으로 `Impl`을 삭제한다.
- event callback 안의 destructor만 coordinator에 owner를 넘긴다.
- move assignment는 기존 `impl_`을 동일 helper로 retire한 뒤 새 `impl_`을 받는다.
- self move는 no-op이며 moved-from destructor는 아무 일도 하지 않는다.
- `RuntimeClient::Close()`는 계속 동기식 명시적 barrier다. adapters의 정상 shutdown은
  subscription reset → worker stop → `Close()` → owner destruction 순서를 유지한다.

RuntimeClient에 `ShutdownAsync`, cleanup handle 또는 public global drain API를 추가하지
않는다. callback-destruction coordinator는 예외 안전 fallback이며 정상 component/DSO
unload는 계속 명시적 `Close()`를 사용한다.

### 4.3 Python binding cleanup

`RuntimeHolder`의 detached deletion fallback을 제거한다.

- runtime이 closed 상태거나 GIL을 보유하지 않으면 동기 삭제한다.
- CPython이 초기화되어 있고 현재 thread가 GIL을 보유하면 `py::gil_scoped_release` 범위에서
  RuntimeClient를 삭제한다.
- Python callback 자신이 마지막 runtime owner를 놓는 경우 core coordinator handoff가
  즉시 반환하고 callback 종료 후 실제 cleanup이 수행된다.
- interpreter finalization에서는 새 Python API를 호출하지 않으며 기존 callback-state의
  finalized-interpreter 방어를 유지한다.

### 4.4 H1-2 admission

- terminal callback에서 마지막 shared RuntimeClient owner 파괴
- callback에서 move assignment로 이전 runtime retirement
- 여러 runtime의 동시 handoff, exact-once destruction과 coordinator idle drain
- 정상 destructor는 불필요한 asynchronous retirement를 사용하지 않음
- Python callback self-release, GC, active subscription과 process exit timeout
- source와 link closure에 `.detach()` cleanup path가 없음
- LeakSanitizer 가능 lane 또는 exact construction/destruction counter 통과

## 5. H1-3 ROS accepted-goal early cancellation

### 5.1 goal coordinator

`GoalAdmissionGate`, `active_goal_`, `active_job_`과 별도 cancel 판단을 ROS-local
`ActionGoalCoordinator`의 한 state machine으로 결합한다. committed runtime authority는
계속 RuntimeClient에 있고 coordinator는 action adaptation state만 소유한다.

goal identity는 pointer 주소가 아니라 action server의 exact Goal UUID를 사용한다.

```text
idle
  → reserved(uuid)
  → accepted_pending_submit(uuid, cancel_pending=false)
  → active_job(uuid, job, cancel_pending=false)
  → terminal
  → idle
```

모든 transition은 `action_mutex_` 아래 수행한다. `RuntimeClient::Submit`, `Cancel`, `Wait`와
ROS terminal callback은 mutex 밖에서 실행한다.

### 5.2 cancel과 submit race

- `HandleGoal`은 decoded goal의 UUID를 reserve한다. 다른 goal은 reject한다.
- `HandleAccepted`는 같은 UUID만 accepted state로 승격하고 goal handle을 기록한다.
- accepted 상태에서 job이 없을 때 같은 UUID의 cancel은 `cancel_pending=true`로 만들고
  `ACCEPT`한다. 반복 요청은 idempotent하게 `ACCEPT`한다.
- active job 상태의 cancel은 exact job을 복사해 lock을 놓은 뒤 core `Cancel()` 결과에
  따라 accept/reject한다.
- 다른 UUID, 이미 terminal인 goal 또는 이전 goal의 stale cancel은 `REJECT`한다.
- submit 성공 시 job publish와 pending flag 인수를 한 transition에서 수행한다. 인수한
  pending cancel은 lock 밖에서 그 job에 정확히 한 번 전달한다.

pending cancel이 수락된 뒤 submit 자체가 실패하면 commit 가능성이 없으므로 ROS goal을
`CANCELED`, `success=false`, job ID 0으로 끝낸다. cancel이 없던 submit 실패는
`ABORTED`다. submit 성공 후 cancel 전달이 늦어 이미 committed success가 된 경우에는
기존 `ResolveRosActionTerminal`이 `SUCCEEDED`를 선택한다.

### 5.3 terminal과 worker failure

exact UUID를 가진 terminal RAII guard가 모든 return path에서 goal handle, job handle,
pending flag와 admission을 한 번만 정리한다. stale guard가 다음 goal state를 지우지 못하게
UUID match를 요구한다.

`std::jthread` action worker 생성 실패는 accepted goal을 error message와 함께 abort하고
guard로 admission을 반환한다. node destruction은 active job이면 cancel을 전달하고 worker를
join한 뒤 runtime을 close한다. pending-submit goal도 cancel pending으로 전환해 orphan
accepted goal을 남기지 않는다.

### 5.4 H1-3 admission

- reserve→accepted와 accepted→job-publish 사이를 각각 차단하는 deterministic seam
- 두 구간의 same-goal cancel accept와 publish 직후 exact core cancel 전달
- wrong UUID, stale previous goal과 repeated cancel
- submit failure with/without pending cancel terminal 차이
- action worker launch failure와 node destruction admission release
- early cancel, normal success, pre-commit cancel, post-commit late cancel과 next goal graph
- 기존 RViz Control Panel과 action/service/message compatibility regression

## 6. H1-4 Algorithm plugin generation 검사

### 6.1 canonical metadata

core configure 단계에서 `${PROJECT_VERSION}`를 사용하는 private generated header를 만든다.
host와 모든 built-in algorithm create target이 이 header를 사용한다.

| Plugin interface | exact capability | schema | exact build generation |
|---|---|---:|---|
| descriptor | `descriptor:kdtree-v3` | `1` | `open-lmm-3.0.0` |
| online remover | `dynamic_remover:online-v3` | `1` | `open-lmm-3.0.0` |
| offline remover | `dynamic_remover:offline-v3` | `1` | `open-lmm-3.0.0` |

literal `open-lmm-1.0`과 generation 없는 algorithm capability를 built-in source에 남기지
않는다. GUI의 `gui:services-v3`와 현재 GUI build metadata는 이번 단계에서 변경하지 않는다.

### 6.2 host expectation과 validation

private `PluginContractExpectation`을 다음 exact 조건을 표현하도록 확장한다.

- capability
- plugin name
- config schema version
- build generation

inspection과 load는 한 metadata validator를 공유한다. 검증 순서는 ABI/incomplete entry →
kind → name → capability → schema → build generation이며, 전부 통과하기 전에는 `create()`를
호출하지 않는다.

`AlgorithmFactory`는 config의 type/model로 expectation을 한 번 구성해 preflight와 actual
load 양쪽에 전달한다. descriptor model은 `scan_context` 또는 `solid`, remover model은
선택된 online/offline model과 exact match해야 한다. null과 empty metadata는 exact expected
값을 만족하지 못하므로 fail-closed한다.

failure는 `kPluginLoadFailed`를 유지하고 plugin path/name, 실패 field, expected와 actual을
message 및 기존 `Error::Context`에 기록한다. schema mismatch에는 schema version도
기록한다. 일반-purpose fixture가 expectation을 생략하는 기존 loader 사용법은 유지하지만
production `AlgorithmFactory`는 expectation을 생략할 수 없다.

### 6.3 compatibility

ABI version, entry symbol, C struct layout, create/destroy signature와 shared handle deleter를
변경하지 않는다. capability/build string 변경은 stale v1 algorithm plugin을 의도적으로
거부하는 generation cutover다. 동일 generation이어도 supported compiler, standard library,
Eigen/PCL 조합을 사용해야 한다는 same-toolchain precondition은 계속 release policy에
명시한다.

### 6.4 H1-4 admission

- old/null/empty capability, wrong name, schema와 build generation fixture의 create count 0
- preflight와 load가 동일 mismatch를 반환함
- built-in 7개 plugin의 exact metadata와 정상 create/destroy
- instance destroy가 DSO close보다 먼저 실행됨
- stale DSO가 남은 installed-prefix upgrade fixture가 fail-closed함
- plugin ABI, selection, soak, package consumer와 source-free runtime smoke 통과

## 7. 구현·검증 순서와 commit 경계

권장 green commit은 다음 네 개다.

1. `fix(runtime): make thread launch rollback exception-safe`
2. `fix(runtime): own deferred client retirement`
3. `fix(ros): preserve accepted-goal early cancellation`
4. `fix(plugins): reject stale algorithm generations`

각 commit은 자기 production change, deterministic fault fixture와 test registration을 함께
포함한다. test owner가 없는 중간 commit, 네 단계를 한꺼번에 되돌려야 하는 commit 또는
config/algorithm tuning diff를 만들지 않는다.

단계별 targeted suite 뒤 최종적으로 다음을 실행한다.

```text
GCC 12 GUI OFF/ON core build + full CTest
GCC 13 GUI OFF
Clang 15 GUI OFF
ROS CTest and installed-core make ros-build
CPython 3.10 wheel lifetime/callback suite
source-free package/distribution ownership gates
ASan+UBSan thread/plugin fault suites
TSan controller/callback/retirement suites
git diff --check
```

## 8. 결과 기록과 중단 조건

구현 완료 후에만 `docs/apps/results/13_production_hardening_result.md`를 만들고 다음을
기록한다.

- 각 work package commit hash와 toolchain
- injected launch index별 created/joined worker 수
- submit 실패 전후 snapshot/event hash
- retirement pending/peak/completed와 exact destruction count
- ROS cancel timing/UUID/terminal matrix
- built-in 및 stale fixture metadata와 create count
- targeted/full/sanitizer/package raw evidence hash
- 잔여 production blocker와 다음 admission

다음 중 하나가 필요하면 해당 work package에서 구현을 중단하고 별도 architecture proposal로
분리한다.

- committed runtime state의 두 번째 owner
- RuntimeClient PImpl 제거 또는 public layout 변경
- callback cleanup을 위한 detached thread나 intentional leak
- unbounded payload copy 또는 owner 없는 cleanup queue
- ROS wire breaking change
- plugin ABI-v1 struct 변경이나 portable ABI 재설계

## 9. 명시적 제외 범위

- GUI alignment feedback authority RAII (`GUI-101`)
- mutable config trusted-root 정책 (`PATH-101`)
- P7-C2 Interactive Alignment와 P7-C3 Transactional Config Form
- 최소 CMake 버전 정리 (`BUILD-101`)
- Goal 09 artifact/SBOM/signing/provenance/promotion
- crash journal, fsync와 startup recovery
- Goal 13 real GPU/driver integration
