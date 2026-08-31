# OpenLMM 프로덕션 준비 잔여 작업

- 상태: H1 완료, 별도 잔여 항목 대기
- 작성일: 2026-08-19
- 기준 브랜치: `develop`
- 기준 커밋: `3deffc7`
- 범위: 현재 package code review에서 확인된 runtime, plugin, GUI, config,
  ROS 및 build-system 잔여 작업

> 최신 통합 구현 계약: THR-101/102, LIFE-101, ROS-101, PLUG-101은
> [`apps/13_production_hardening_implementation_spec.md`](apps/13_production_hardening_implementation_spec.md)의
> H1 순서와 완료 조건을 canonical specification으로 사용한다. H1은 2026-08-28에
> 구현·검증됐으며 아래 내용은 최초 감사 기록과 H1 외 잔여 작업을 함께 보존한다.

## 1. 목적

디렉터리 개편과 package boundary 정리는 완료됐다. 이후 작업은 구조를 다시 바꾸는
것이 아니라, 운영 중 실패 가능성이 있는 수명주기와 호환성 경계를 강화하는 데
집중한다.

우선순위는 다음과 같다.

1. 잘못된 plugin을 process에 생성하기 전에 거부한다.
2. thread 자원 고갈을 process 종료가 아닌 명시적 오류로 처리한다.
3. GUI alignment feedback authority를 정확한 수명으로 관리한다.
4. callback 내부 client 파괴에서 detached cleanup과 leak을 제거한다.
5. mutable config의 신뢰 경계를 확정한다.
6. package 및 ROS의 선언·취소 계약을 실제 동작과 일치시킨다.

## 2. 작업 요약

| ID | 우선순위 | 작업 | 상태 |
|---|---:|---|---|
| PLUG-101 | P1 | Algorithm plugin capability/ABI generation 검증 | **완료 / 검증됨 (H1-4)** |
| THR-101 | P1 | Pipeline thread 생성 실패의 exception-safe rollback | **완료 / 검증됨 (H1-1)** |
| THR-102 | P1 | BoundedExecutor 부분 생성 실패 안전화 | **완료 / 검증됨 (H1-1)** |
| GUI-101 | P1 | Alignment feedback authority RAII화 | 대기 |
| LIFE-101 | P2 | RuntimeClient detached cleanup 및 leak 제거 | **완료 / 검증됨 (H1-2)** |
| PATH-101 | P2/P1 | Mutable config trusted-root 정책 확정 | 정책 결정 필요 |
| ROS-101 | P2 | Action accept 직후 cancellation race 제거 | **완료 / 검증됨 (H1-3)** |
| BUILD-101 | P2 | 실제 최소 CMake 버전 통일 | 대기 |
| CI-101 | P2 | Resource failure와 stale plugin 회귀 gate 추가 | 대기 |

`PATH-101`은 SDK 또는 원격 control plane에서 신뢰되지 않은 입력을 받을 경우 P1으로
상향한다.

## 3. PLUG-101: Algorithm plugin 호환성 세대 검증

### 3.1 문제

Algorithm plugin은 C ABI entry를 사용하지만 실제 instance는 C++ virtual object다.
따라서 `abi_version == 1`과 `plugin_kind`만 같다고 해서 안전하지 않다. compiler,
표준 라이브러리, public interface layout, Eigen 및 PCL ABI가 달라질 경우 create 이후
vtable 호출에서 crash 또는 undefined behavior가 발생할 수 있다.

현재 GUI plugin은 `gui:services-v3` capability를 정확히 검사하지만 descriptor와
dynamic remover는 capability, config schema version 및 build generation을 강제하지
않는다. built-in algorithm plugin의 `build_version`도 현재 package 3.0과 달리
`open-lmm-1.0`으로 남아 있다.

### 3.2 구현 범위

- interface별 exact capability를 정의한다.

```text
descriptor:kdtree-v3
dynamic_remover:online-v3
dynamic_remover:offline-v3
```

- descriptor와 remover의 inspect/load 모두 exact capability를 전달한다.
- host가 지원하는 `config_schema_version`을 명시하고 일치 여부를 검사한다.
- `build_version`은 CMake의 `PROJECT_VERSION`에서 자동 생성한다.
- plugin name과 config에서 요청한 model이 일치하는지 검사한다.
- error에는 path, expected/actual capability, schema 및 build generation을 포함한다.
- GUI의 기존 `gui:services-v3` 검증은 유지한다.
- plugin ABI v1 entry symbol 자체는 변경하지 않는다.

### 3.3 테스트

- 과거 capability를 가진 descriptor plugin을 create 전에 거부한다.
- 과거 capability를 가진 online/offline remover를 create 전에 거부한다.
- kind는 같지만 model name이 다른 plugin을 거부한다.
- schema version 불일치를 거부한다.
- built-in plugin metadata가 package version과 일치한다.
- 정상 built-in plugin의 create/destroy 및 DSO lifetime fixture가 계속 통과한다.
- installed prefix에 과거 DSO를 남긴 upgrade-in-place fixture를 추가한다.

### 3.4 완료 조건

- 호환되지 않는 algorithm plugin은 `create()`가 호출되기 전에 실패한다.
- 모든 built-in plugin metadata가 한 CMake source of truth에서 생성된다.
- plugin ABI, package consumer 및 clean install/upgrade test가 통과한다.

## 4. THR-101: Pipeline thread 생성 실패 rollback

### 4.1 문제

`PipelineController::submit()`은 controller state를 queued로 게시한 뒤 execution thread와
lifecycle thread를 차례로 생성한다.

- execution thread 생성이 실패하면 public `Result` API 밖으로 exception이 전파되고
  queued state가 남을 수 있다.
- lifecycle thread 생성이 실패하면 joinable execution thread의 destructor가
  `std::terminate()`를 호출할 수 있다.

### 4.2 구현 계약

- thread 생성은 injectable factory 또는 작은 thread launcher abstraction을 통한다.
- queued state를 게시하기 전 필요한 allocation을 최대한 완료한다.
- 부분 생성된 thread는 rollback guard가 cancellation을 요청하고 반드시 join한다.
- 실패 시 다음 상태를 원복한다.

```text
job
worker
cancellation token
terminal watermark
feedback publication state
pending event queue
```

- `std::system_error`는 resource exhaustion을 나타내는 typed `Error`로 변환한다.
- 실패한 submit은 job handle과 queued/started event를 외부에 남기지 않는다.
- rollback 자체는 noexcept여야 한다.

### 4.3 테스트

- 첫 번째 thread 생성 실패를 주입한다.
- 두 번째 thread 생성 실패를 주입한다.
- 실패 후 snapshot에 active/queued job이 없는지 확인한다.
- 실패 후 다음 submit이 정상적으로 성공하는지 확인한다.
- failure path에서 `std::terminate`, thread leak 및 callback 잔존이 없는지 확인한다.
- ASAN/UBSAN과 TSAN targeted test를 실행한다.

## 5. THR-102: BoundedExecutor 부분 생성 실패 안전화

### 5.1 문제

`BoundedExecutor` 생성 중 worker 일부를 만든 뒤 다음 `std::thread` 생성이 실패하면
이미 생성된 joinable thread가 stack unwinding 중 파괴돼 process가 종료될 수 있다.

### 5.2 구현 계약

- constructor body에 부분 생성 rollback guard를 둔다.
- 생성 실패 시 `stopping_`을 게시하고 생성된 worker를 모두 깨운 뒤 join한다.
- 생성 실패는 controller/bootstrap 계층에서 typed resource error로 변환한다.
- executor가 성공적으로 생성된 경우에만 task admission을 시작한다.

### 5.3 테스트

- 1번, 중간, 마지막 worker 생성 실패를 각각 주입한다.
- 생성된 worker가 모두 종료되는지 확인한다.
- process termination과 background thread leak이 없는지 확인한다.
- 정상 executor의 queue capacity, cancellation 및 shutdown test를 유지한다.

## 6. GUI-101: Alignment feedback authority RAII화

### 6.1 문제

현재 GUI host가 runtime의 global boolean을 직접 켜고 끈다. 이 방식에는 다음 문제가
있다.

- 실행 중인 `GuiRuntimeHost`에 move assignment하면 기존 host의 `Stop()`이 호출되지
  않아 이전 runtime의 feedback 상태가 true로 남는다.
- 동일 runtime에 둘 이상의 host가 연결되면 한 host의 종료가 다른 host의 authority도
  제거한다.
- GUI가 사라진 뒤 headless runtime이 interactive feedback을 기다릴 수 있다.

### 6.2 목표 API

```cpp
class AlignmentFeedbackAuthority {
 public:
  AlignmentFeedbackAuthority(AlignmentFeedbackAuthority&&) noexcept;
  AlignmentFeedbackAuthority& operator=(AlignmentFeedbackAuthority&&) noexcept;
  ~AlignmentFeedbackAuthority();
};

Result<AlignmentFeedbackAuthority>
RuntimeClient::AcquireAlignmentFeedbackAuthority();
```

- authority 획득과 해제는 reference-counted 또는 token registry로 구현한다.
- 마지막 authority가 해제될 때만 feedback을 비활성화하고 active review를 취소한다.
- `GuiRuntimeHost::Impl`이 authority token을 plugin보다 오래 소유한다.
- custom move assignment는 기존 host를 먼저 정리한 뒤 token을 이동한다.
- 기존 boolean setter를 유지해야 한다면 compatibility wrapper로 제한하고 deprecated
  처리한다.

### 6.3 테스트

- 실행 중인 host로 move assignment했을 때 이전 runtime authority가 해제된다.
- host 두 개 중 하나만 종료하면 authority가 유지된다.
- 마지막 host 종료 시 authority가 제거된다.
- plugin start 실패 시 authority가 rollback된다.
- authority가 없는 headless mode는 interactive wait에 진입하지 않는다.
- terminal callback과 동시에 host를 종료해도 deadlock이 없다.

## 7. LIFE-101: RuntimeClient cleanup ownership

### 7.1 문제

callback 내부에서 마지막 `RuntimeClient`가 파괴되면 self-join 회피를 위해 detached
thread에서 `Impl`을 삭제한다. thread 생성 실패 시 runtime 전체를 의도적으로 leak하고,
cleanup 완료를 기다릴 handle도 제공하지 않는다.

### 7.2 권장 설계

다음 중 하나를 선택해 cleanup ownership을 명시한다.

1. service가 소유한 bounded cleanup executor로 retire를 위임한다.
2. `ShutdownAsync()`가 join 가능한 cleanup handle을 반환한다.
3. callback registry와 controller lifetime을 분리해 callback thread에서도 동기 파괴가
   self-join을 만들지 않도록 한다.

필수 조건은 다음과 같다.

- detached thread를 사용하지 않는다.
- thread 생성 실패를 이유로 runtime을 leak하지 않는다.
- component/DSO unload 전에 cleanup 완료를 확인할 수 있다.
- callback 내부 마지막 owner 파괴가 deadlock 또는 self-join을 만들지 않는다.
- move assignment도 같은 retirement 경로를 사용한다.

### 7.3 테스트

- terminal callback에서 마지막 client owner를 파괴한다.
- callback에서 client move assignment로 기존 runtime을 retire한다.
- cleanup executor admission 실패를 주입한다.
- plugin instance와 controller가 정확히 한 번 파괴되는지 확인한다.
- LeakSanitizer가 가능한 비-container job 또는 별도 heap ownership fixture를 추가한다.

## 8. PATH-101: Mutable config trusted-root 정책

### 8.1 현재 상태

`selected_document`가 config root 외부를 가리켜도 warning만 기록하고 file transaction을
계속 수행한다. 로컬에서 신뢰된 GUI만 사용하는 배포에서는 호환 정책일 수 있지만,
SDK나 원격 입력으로 노출되면 process 권한 범위의 외부 파일을 교체할 수 있다.

### 8.2 결정할 정책

기본 권장 정책은 다음과 같다.

- module config write는 canonical config root 내부만 허용한다.
- absolute path는 기본 거부한다.
- `..`, symlink 및 bind path 결과도 canonical root 안이어야 한다.
- 외부 path가 필요한 배포는 runtime open option의 trusted allowlist로만 허용한다.
- config JSON 자체는 write 권한을 확장할 수 없다.
- root, 다른 module document, temporary 및 backup alias를 거부한다.

호환성 때문에 즉시 강제할 수 없다면 다음 migration을 적용한다.

1. 3.x에서 warning과 telemetry를 수집한다.
2. allowlist option을 추가한다.
3. 다음 minor에서 external path 사용 시 명시적 opt-in을 요구한다.
4. 다음 major에서 기본 거부한다.

### 8.3 테스트

- root 내부 regular file과 새 file을 허용한다.
- `../`, 절대 외부 path, root 밖 symlink를 거부한다.
- 검증 후 commit 전 symlink 교체 race를 거부한다.
- trusted allowlist에 포함된 path만 예외적으로 허용한다.
- 거부 시 disk, runtime state 및 revision이 모두 유지된다.

## 9. ROS-101: Action cancellation 계약

### 9.1 문제

goal이 accept된 뒤 worker가 runtime job handle을 게시하기 전 cancellation이 도착하면
현재 구현은 `active_job_`이 없다는 이유로 cancel을 거부한다. 또한 cancel callback에
전달된 goal handle을 active goal과 비교하지 않는다.

### 9.2 구현 계약

- accepted goal identity를 strong 또는 stable weak handle로 추적한다.
- 해당 goal에 대한 cancellation을 job submit 전에도 pending 상태로 기록한다.
- submit 완료 직후 pending cancellation을 runtime job에 전달한다.
- 다른 goal handle의 cancellation이 현재 job을 취소하지 못하게 한다.
- terminal transition은 admission gate와 active goal/job 상태를 하나의 RAII guard로
  정리한다.

### 9.3 테스트

- accept 직후 submit 전 cancel을 요청한다.
- submit 직후와 wait 중 cancel을 요청한다.
- 이전 goal의 stale cancel이 새 goal에 영향을 주지 않는다.
- submit 실패와 node destruction에서도 admission reservation이 해제된다.
- concurrent goal은 계속 하나만 admission된다.

## 10. BUILD-101: 최소 CMake 버전 정합성

### 10.1 문제

core 및 ROS package는 `cmake_minimum_required(VERSION 3.5)`를 선언하지만 실제로는
`target_link_options`, `FetchContent_MakeAvailable`, C++20 target feature 등 더 최신
CMake 기능을 사용한다. package consumer fixture는 이미 3.16을 요구한다.

### 10.2 작업

- core와 ROS의 최소 CMake 버전을 3.16 이상으로 통일한다.
- README, Docker image 및 release policy의 지원 버전을 일치시킨다.
- package config가 요구하는 명령도 동일한 최소 버전에서 검증한다.
- 더 높은 최소 버전이 필요한 dependency가 있으면 configure 단계에서 명확하게
  실패시킨다.

### 10.3 완료 조건

- 선언된 최소 버전의 clean configure/build가 CI에서 통과한다.
- 그보다 낮은 버전에서는 첫 configure에서 명확한 version error가 발생한다.
- source package와 installed package consumer가 같은 최소 버전을 사용한다.

## 11. CI-101: 잔여 작업 회귀 gate

다음 fixture를 required checks에 포함한다.

- stale descriptor/remover capability rejection
- plugin schema/build generation mismatch rejection
- pipeline first/second thread creation failure
- bounded executor partial construction failure
- GUI authority move/multi-owner lifetime
- callback 내부 마지막 client destruction
- ROS pre-submit cancellation 및 goal identity
- trusted-root path escape와 symlink race

추가 정책:

- sanitizer build 재시도는 최초 compiler failure를 성공 evidence로 덮지 않도록 별도
  flaky 상태로 기록한다.
- LeakSanitizer를 실행할 수 있는 환경을 별도 job으로 제공하거나 ownership counter
  fixture로 leak 경로를 검증한다.
- fault injection test는 정상 compiler matrix와 ASAN/UBSAN 양쪽에서 실행한다.

## 12. 구현 순서

### Milestone 1: 즉시 안정화

1. `PLUG-101` exact capability와 metadata generation
2. `THR-101` pipeline thread rollback
3. `THR-102` executor partial construction rollback
4. 관련 fault injection 및 sanitizer gate

이 milestone은 process crash와 잘못된 DSO 실행을 직접 차단하므로 최우선이다.

### Milestone 2: 수명주기 정리

1. `GUI-101` feedback authority token
2. `LIFE-101` client cleanup ownership
3. GUI move/multi-owner 및 callback destruction stress test

### Milestone 3: 외부 경계 정리

1. `PATH-101` product policy 확정과 migration
2. `ROS-101` cancellation state machine
3. `BUILD-101` minimum version 통일
4. package/ROS/upgrade-in-place 전체 검증

## 13. 권장 커밋 분리

1. `test(plugin): reproduce stale algorithm capability acceptance`
2. `fix(plugin): enforce algorithm interface generations`
3. `test(runtime): inject pipeline thread launch failures`
4. `fix(runtime): roll back partial job thread startup`
5. `fix(runtime): make executor construction failure-safe`
6. `refactor(gui): own feedback authority with an RAII token`
7. `refactor(runtime): replace detached client retirement`
8. `fix(ros): preserve pre-submit action cancellation`
9. `fix(config): confine mutable documents to trusted roots`
10. `build: align the declared minimum CMake version`
11. `ci: gate production lifetime and compatibility faults`

테스트 재현 커밋과 구현 커밋을 분리하고, directory move나 무관한 algorithm 변경을
같은 커밋에 넣지 않는다.

## 14. 전체 검증 계획

- GCC 12 GUI-OFF/ON clean build
- GCC 13 GUI-OFF clean build
- Clang 15 GUI-OFF clean build
- 전체 CTest
- ASAN/UBSAN fault-focused suite
- TSAN lifecycle/callback suite
- installed source-free package consumer
- public header self-containment
- C11 plugin SDK fixture
- clean install 및 upgrade-in-place stale DSO fixture
- ROS headless 및 GUI composition build/test
- test1/test2 ordered replay와 pose tolerance 확인
- config checksum, output directory 및 cache 정책 확인
- architecture/release policy 검사

## 15. 별도 product decision으로 유지할 항목

다음은 이번 code review 수정과 별개로 기존 architecture report에 남아 있는 제품 정책
작업이다.

- 대용량 PCD streaming decode 또는 preview sidecar
- public runtime snapshot의 recovery-required health 표현
- input dataset mutation과 immutable manifest 정책
- power-loss durability를 위한 journal, directory fsync 및 startup recovery

측정된 production 요구가 확인되기 전까지 위 항목을 이번 안정화 작업에 섞지 않는다.

## 16. 최종 완료 조건

- 호환되지 않는 algorithm plugin이 instance 생성 전에 거부된다.
- thread 생성 실패가 exception 또는 process termination이 아닌 typed error로 반환된다.
- 부분 생성된 worker, queued job, callback 및 runtime resource가 남지 않는다.
- GUI feedback availability가 정확한 authority 수명과 일치한다.
- callback 내부 client 파괴가 detached thread와 leak에 의존하지 않는다.
- config write가 확정된 trusted-root 정책을 따른다.
- ROS action은 accept 이후 어느 시점의 cancellation도 올바른 goal에 적용한다.
- 최소 CMake 버전 선언과 실제 package consumer 요구사항이 일치한다.
- normal, sanitizer, package, ROS, plugin, replay 및 architecture gate가 모두 통과한다.
