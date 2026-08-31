# Goal 06 — Python Binding v1 구현 명세서 및 로컬 구현 결과

- 문서 상태: **로컬 구현 완료 / public release 전**
- 최초 작성일: 2026-08-25 UTC
- 구현 검증일: 2026-08-26 UTC
- 기준 브랜치: `develop`
- 기준 HEAD: `27b39f726f79651b827d192d0c23cc5546531daf`
- 검증 대상: 위 HEAD에 Goal 06 변경을 적용한 미커밋 worktree
- architecture freeze: `59e003ebc4b7d44597ced4ddab3436adec310370`
- 상위 요구사항:
  `docs/pose_freeze_goals/open_lmm_post_freeze_goals/06_python_binding_v1_goal.md`
- 결과 문서 경로: 구현 중에는 이 문서를 갱신하고, 완료 시 실제 변경·검증 증거를
  같은 문서에 추가한다.

## 1. 결론

Goal 06의 로컬 설계·구현은 완료됐다. Python binding은 새로운 runtime이나 상태 owner가
아니라 기존 `RuntimeClient`를 소비하는 **leaf adapter**로 구현됐다.

첫 완료 범위는 다음으로 고정한다.

- CPython 3.10 / Ubuntu 22.04 / x86-64의 로컬 platform wheel
- `RuntimeClient` 기반 Open, RunAll, RunStage, RunNode, OptimizeThrough,
  Wait, Cancel, Snapshot, ConfigDocuments, ConfigCandidates, ReplaceRootConfig,
  Visualization, ApplyConfig, event subscription, Close
- Python 예외, immutable snapshot DTO, NumPy point representation
- long-running call의 GIL release와 callback의 안전한 GIL reacquire
- source-free local wheel install/import/E2E

Goal 03–05의 hosted evidence 미완료는 이 로컬 구현을 막지 않는다. 다만 이 Goal의
wheel은 아직 공식 portable release artifact가 아니다. 다중 Python/플랫폼 wheel,
manylinux 적합성, 외부 게시, SBOM·서명·RC 승격은 Goal 09에서 수행한다.

## 2. 현재 코드 기준점

### 2.1 이미 존재하는 공개 경계

`open_lmm/include/open_lmm/server/runtime_client.hpp`의 `RuntimeClient`는 다음 공개
operation을 제공한다.

| C++ operation | Python v1 | 비고 |
|---|---|---|
| `Open` | `Runtime.open` | config directory, label, output root |
| `ReplaceRootConfig` | `Runtime.replace_root_config` | exact expected revision과 open 당시 bootstrap request 사용 |
| `SubmitRunAll` | `Runtime.run_all` | `Job` 반환 |
| `Submit(kStage)` | `Runtime.run_stage` | public `StageId`만 노출 |
| `Submit(kNode)` | `Runtime.run_node` | public `NodeId`, optional exact agent만 전달 |
| `Submit(kOptimizeThrough)` | `Runtime.optimize_through` | authoritative catalog의 agent ID를 전달 |
| `Cancel` | `Runtime.cancel`, `Job.cancel` | 같은 Runtime이 만든 Job만 허용 |
| `Wait` | `Job.wait` | 성공 시 `None`, 실패 시 Python 예외 |
| `Snapshot` | `Runtime.snapshot` | immutable Python DTO |
| `NodeDescriptors` | 미노출 | v1 완료 후 additive 후보 |
| committed config query | `Runtime.config_documents` | canonical JSON, logical selector, revision만 복사 |
| trusted candidate query | `Runtime.config_candidates` | bounded/schema-validated model, canonical JSON, relative selector만 복사 |
| `Visualization` | `Runtime.visualization` | NumPy semantic point array |
| alignment feedback | 미노출 | interactive alignment는 후속 additive 범위 |
| `ApplyConfig` | `Runtime.apply_config` | explicit expected revision 필수 |
| `SubscribeEvents` | `Runtime.subscribe_events` | `Subscription` 수명으로 해제 |
| `Close` | `Runtime.close` | 기본 cancel-and-wait, idempotent |
| `IsOpen` | `Runtime.is_open` | authoritative query의 편의 표현 |

Node, Optimize Through, committed config/candidate query와 root replacement는 초기 v1 완료 후
additive method로 추가됐다. 새 Python-side execution graph, config owner나 scheduler를
만들지 않으며 기존 `RuntimeClient` submit/query/transaction 경계에서 검증·실행된다.
`NodeDescriptors`와 alignment feedback은 계속 후속 범위다.

`Result<T>`는 Python에 노출하지 않는다. 성공 값만 반환하고 `Error`는 한 binding
boundary에서 Python 예외로 변환한다.

### 2.2 보호해야 하는 architecture invariant

| Invariant | Goal 06 적용 규칙 |
|---|---|
| INV-01~04 state/commit | Python이 state, transaction, file commit owner를 만들지 않는다. |
| INV-05 late cancel | `Job.wait()`는 C++ receipt 결과를 그대로 따른다. Python이 late cancel을 실패로 재해석하지 않는다. |
| INV-06~07 reconciliation/health | native `Error`와 snapshot을 보존하며 로그나 Python cache를 authority로 삼지 않는다. |
| INV-08 epoch isolation | Python callback/job이 retired runtime의 결과를 active runtime에 재게시하지 않는다. |
| INV-09~10 presentation | visualization은 immutable derived snapshot이며 runtime state를 소유하지 않는다. |
| INV-13 adapter leaf | binding target은 `open_lmm_client`와 public headers만 소비한다. |
| INV-14 API/PImpl | `RuntimeClient` PImpl 뒤의 타입을 bind하거나 include하지 않는다. |
| INV-16 resource | large NumPy payload의 copy/ownership과 peak memory를 측정한다. |
| INV-17 lifetime | close/cancel/unsubscribe/interpreter shutdown을 deterministic하게 검증한다. |
| INV-18 diagnostics | Python-side 상태나 event log는 다음 runtime 동작의 authority가 아니다. |

다음 타입과 경로는 binding source에서 이름조차 사용하지 않는다.

```text
RuntimeService
PipelineController
StageExecutor
StageCoordinator
RuntimeStateStore
RuntimeTransaction
RuntimeReconfigurer
MapServer
plugins/host/*
src/runtime/*
src/domain/*
PCL public types
```

## 3. 기술 선택

### 3.1 선택: pybind11

초기 binding 기술은 **pybind11**로 고정한다.

선택 이유:

- C++20, STL, Eigen 및 NumPy buffer 작성에 필요한 기능을 제공한다.
- GIL release/reacquire와 C++ exception translator를 작은 adapter layer에서 명시할 수
  있다.
- CMake target 기반으로 `open_lmm_client`에 직접 연결할 수 있다.
- Goal 06은 Python hot-path plugin이 아니라 coarse-grained runtime façade이므로 binding
  호출 overhead보다 lifetime 명료성과 검증 용이성이 우선이다.
- binding generator가 runtime 내부를 자동 노출하지 않고 공개 DTO를 선택적으로 매핑할
  수 있다.

nanobind는 생성 binary와 호출 overhead 측정이 실제 병목으로 확인될 때만 별도 비교한다.
기술 선택 자체를 이유로 동일 API의 두 binding backend를 병렬 유지하지 않는다.

### 3.2 의존성 정책

- `pybind11`과 Python build backend는 Python build requirement이며 C++ Runtime/Development
  install component의 필수 의존성으로 만들지 않는다.
- `OPEN_LMM_BUILD_PYTHON=OFF`가 기본값이다. OFF에서는 adapter가 Python development,
  NumPy, pybind11을 탐색하거나 내려받지 않는다. ROS/ament 자체의 Python 사용은 이
  option의 범위가 아니다.
- wheel build environment의 package 버전은 별도 constraints 파일에 exact pin한다.
- configure 중 암묵적 network fetch는 금지한다. pybind11은 활성 Python build
  environment가 제공하고 CMake는 `find_package(pybind11 CONFIG REQUIRED)`로 찾는다.
- NumPy는 Python runtime dependency다. C++ public header나 ABI dependency로 추가하지
  않는다.

검증한 로컬 기준은 CPython 3.10.12, NumPy 1.21.5, CMake 3.25.3, GCC 12.3,
Clang 15.0.7이다. 일반 CMake build는 배포판 pybind11 2.9.1을 사용했고, local wheel
build venv는 `build-constraints.txt`에 고정한 pybind11 2.13.6과
scikit-build-core 0.10.7을 사용했다.

### 3.3 build backend: scikit-build-core

PEP 517 backend는 **scikit-build-core**를 사용한다. 기존 native target graph를 CMake와
별도로 재작성하지 않고, `pyproject.toml`에서 Python package와 CMake install 결과를 한
wheel staging tree로 조립하기 위함이다. setuptools custom `build_ext`와 CMake wrapper를
동시에 유지하지 않는다.

- `src/adapters/python/pyproject.toml`이 distribution metadata와 Python dependency를
  소유한다.
- native project version은 configure metadata에서 읽어 `open_lmm.__version__`과
  일치시킨다. version을 두 파일에서 수동으로 따로 올리지 않는다.
- build requirement의 exact resolved version은
  `src/adapters/python/build-constraints.txt`에 고정한다.
- backend가 network dependency를 임의 선택하게 두지 않고 격리 build에 constraints를
  적용한다.

## 4. Python public API v1

### 4.1 package identity

```text
distribution: open-lmm
import name:  open_lmm
native name:  open_lmm._native   # private, compatibility contract 아님
API_VERSION:  1
__version__:  native project version과 동일(현재 3.0.0)
```

`_native`의 class/function 이름은 public contract가 아니다. 사용자는 최상위
`open_lmm`에서 re-export된 API만 사용한다.

### 4.2 사용자 API

```python
from os import PathLike
from typing import Callable, Optional

class Runtime:
    def __init__(self, max_agent_tasks: int = 4) -> None: ...

    def open(
        self,
        config_directory: str | PathLike[str],
        *,
        label: str = "",
        output_root: str | PathLike[str] | None = None,
    ) -> None: ...

    def run_all(self) -> Job: ...

    def run_stage(
        self,
        stage: Stage,
        *,
        agent: str | None = None,
    ) -> Job: ...

    def snapshot(self) -> RuntimeSnapshot: ...

    def visualization(
        self,
        agent: str,
        *,
        include_points: bool = True,
        preview_voxel_size_m: float | None = None,
    ) -> VisualizationSnapshot: ...

    def apply_config(
        self,
        domain: ConfigDomain,
        document_json: str,
        *,
        expected: Revision,
        selected_document: str | PathLike[str] | None = None,
    ) -> ConfigApplyReceipt: ...

    def subscribe_events(
        self,
        callback: Callable[[ExecutionEvent], None],
    ) -> Subscription: ...

    def cancel(self, job: Job) -> None: ...
    def close(self, *, cancel_running: bool = True) -> None: ...
    def is_open(self) -> bool: ...
    def __enter__(self) -> Runtime: ...
    def __exit__(self, exc_type, exc, traceback) -> None: ...

class Job:
    @property
    def id(self) -> int: ...
    def wait(self) -> None: ...
    def cancel(self) -> None: ...

class Subscription:
    def close(self) -> None: ...
    def __enter__(self) -> Subscription: ...
    def __exit__(self, exc_type, exc, traceback) -> None: ...
```

기본 UX는 상위 Goal의 예시와 동일하다.

```python
import open_lmm

with open_lmm.Runtime() as runtime:
    runtime.open("config", label="trial-01")
    job = runtime.run_all()
    job.wait()
    snapshot = runtime.snapshot()
```

### 4.3 타입 규칙

- `Stage`, `Node`, `ArtifactType`, `ArtifactState`, `RuntimeStatus`, `JobState`,
  `EventType`, `ConfigDomain`, `CancellationMode`, `ErrorCode`, `ErrorSeverity`는
  Python `Enum`으로 공개한다.
- snapshot/event/receipt는 `@dataclass(frozen=True, slots=True)` 또는 그와 동등한
  immutable value object로 공개한다.
- `AgentId`는 Python 경계에서 non-empty `str`로 표현하고 native `AgentId::Parse`로
  검증한다.
- filesystem 값은 입력에서 `os.PathLike`를 허용하고 출력에서는 `pathlib.Path`를
  사용한다.
- C++ `std::string_view`를 Python object가 직접 참조하게 하지 않는다. 모든 문자열은
  소유 Python `str`로 변환한다.
- `JobHandle`은 독립 public class로 노출하지 않는다. `Job` 내부에 보관하고 `id`만
  read-only로 공개한다.
- 같은 `Runtime`에서 생성하지 않은 `Job`을 `cancel`에 넘기면 C++ 호출 전에
  `ValueError`가 아니라 `OpenLMMInvalidArgumentError`를 발생시킨다.

### 4.4 v1 snapshot 최소 필드

public DTO의 의미를 잃지 않도록 다음 값을 보존한다.

```text
RuntimeSnapshot
  label, status, output_directory, pipeline

PipelineSnapshot
  job, runtime_revision, config_revision, agents, artifacts, recent_events

JobSnapshot
  id, state, active_stage, message, cancellation

ArtifactMetadata
  type, agent, state, revision, producer, detail, external_path, fingerprint

ExecutionEvent
  job_id, type, stage, node, agent, message, sequence,
  progress_current, progress_total, error, cancellation,
  affected_agents, algorithm_progress
```

필드가 비어 있다고 DTO 자체를 생략하지 않는다. 선택 값은 `None`, 반복 값은 immutable
`tuple`로 변환한다. Python snapshot을 수정해도 native runtime에 영향을 줄 수 없어야
한다.

## 5. Error contract

### 5.1 변환 원칙

`Result<T>::Failure(Error)`는 다음 public hierarchy로 변환한다.

```text
OpenLMMError
├── OpenLMMFileNotFoundError
├── OpenLMMParseError
├── OpenLMMInvalidArgumentError
├── OpenLMMPluginLoadError
├── OpenLMMRegistrationError
├── OpenLMMOptimizationError
├── OpenLMMIOError
├── OpenLMMCancelledError
└── OpenLMMAgentExcludedError
```

모든 예외는 다음 read-only attribute를 갖는다.

```python
error.code          # ErrorCode
error.severity      # ErrorSeverity
error.context       # ErrorContext immutable DTO
str(error)          # native message
```

`ErrorContext`는 `runtime_revision`, `stage`, `node`, `agent`, `plugin`, `config`,
`json_pointer`, `expected`, `actual`, `schema_version`을 보존한다. Python이 message
문자열을 parse해 분기하도록 만들지 않는다.

### 5.2 예외 안전성

- native `Result` failure와 예상하지 못한 C++ exception을 구분한다.
- 공개 call을 빠져나온 `std::exception`은 `OpenLMMInternalError`로 변환하고 원문을
  보존한다. unknown exception은 고정 message를 사용한다.
- `KeyboardInterrupt`가 blocking `wait`를 즉시 native cancellation로 바꾼다고 약속하지
  않는다. 취소가 필요한 프로그램은 다른 Python thread에서 `job.cancel()`을 호출한다.
- callback에서 발생한 Python exception은 native worker로 전파하지 않는다.
  `sys.unraisablehook`으로 보고하고 subscription은 유지한다.

## 6. Lifetime와 thread/GIL contract

### 6.1 Runtime/Job ownership

- Python `Runtime` holder만 하나의 `RuntimeClient`를 소유한다.
- `Job`은 native state를 복제하지 않고 owner identity와 `JobHandle`만 가진다.
- operation은 호출 시작 시 native holder의 strong reference를 확보한 뒤 GIL을 놓는다.
  따라서 다른 Python thread가 `Runtime` reference를 삭제해도 진행 중인 C++ call의
  객체가 중간 파괴되지 않는다.
- `Job`이 `Runtime`의 수명을 무기한 연장하지 않는다. owner가 사라진 뒤 job method는
  `OpenLMMInvalidArgumentError`를 발생시킨다.
- Python destructor는 exception을 밖으로 던지지 않는다. 명시적 `close()`가 오류를
  관찰하는 유일한 경로다.
- `Runtime.close()`는 C++ `Close`의 idempotency를 그대로 보존한다.
- context manager 종료는 기본 `cancel_running=True`로 close한다.

Python holder는 runtime state, revision, health, job state를 cache하지 않는다. cross-owner
job 검사와 Python object lifetime을 위한 identity만 보유한다.

### 6.2 GIL policy

| Python call | Native call 동안 GIL | 이유 |
|---|---|---|
| `open` | release | config/plugin preflight와 filesystem 작업 가능 |
| `run_all`, `run_stage` | release | admission/submit이 잠시 block할 수 있음 |
| `Job.wait` | release | long-running primary path |
| `cancel` | release | concurrent job thread와 독립 진행 |
| `snapshot` | release 후 변환 시 reacquire | native vectors 복사 가능 |
| `visualization` | release 후 NumPy 생성 시 reacquire | large derived snapshot |
| `apply_config` | release | validate/file barrier/commit 가능 |
| `subscribe_events`/unsubscribe | native wait 동안 release | callback drain과 GIL deadlock 방지 |
| `close` | release | cancel, wait, callback drain 포함 |
| native event callback | acquire | Python object 접근 전 필수 |

GIL을 놓은 영역에서는 `py::object`, Python C API, Python allocator를 접근하지 않는다.
native `Result`를 소유 C++ 값으로 받은 뒤 GIL을 다시 확보해 Python object로 변환한다.

### 6.3 Callback와 subscription

`Subscription`은 반드시 다음 순서를 구현한다.

1. Python callback과 active flag를 가진 shared callback state를 만든다.
2. native callback 진입 시 GIL을 획득하고 active 여부를 재확인한다.
3. `ExecutionEvent`를 소유 Python DTO로 변환한 후 callback을 호출한다.
4. Python exception은 `sys.unraisablehook`에 전달하고 C++ 경계를 넘기지 않는다.
5. `Subscription.close()`는 active를 false로 전환한다.
6. GIL을 놓고 native `ExecutionEventSubscription::Reset()`을 호출해 in-flight callback을
   drain한다.
7. GIL을 다시 획득한 상태에서 마지막 Python callback reference를 해제한다.

callback 안에서는 `snapshot()` 같은 read-only query를 허용한다. `open`, `close`,
`replace`, 새 job submit처럼 native가 callback thread에서 거부하는 lifecycle command는
그 native error를 그대로 Python 예외로 반환한다. 별도 Python work queue나 두 번째
event authority를 만들지 않는다.

module은 weak runtime/subscription registry와 `atexit` cleanup을 제공한다. interpreter가
정상 종료할 때 subscription을 먼저 끊고 runtime을 close한다. finalization이 이미
시작돼 GIL-safe decref가 불가능한 비정상 fallback에서는 crash나 self-join보다 제한된
reference leak을 선택하며, 이 경로를 subprocess shutdown test로 감시한다.

## 7. NumPy / point-cloud contract

### 7.1 공개 표현

PCL, Eigen container, native pointer는 Python에 노출하지 않는다.

| 값 | NumPy 표현 | dtype | 쓰기 |
|---|---|---|---|
| visualization points | `(N, 4)` = x,y,z,intensity | `float32` | read-only |
| visualization poses | `(N, 4, 4)` homogeneous transform | `float32` | read-only |
| bounds | `(3,)` min/max | `float32` | read-only |
| alignment point 후보 | v1 미노출 | - | - |

`points[:, :3]`가 semantic xyz이고 `points[:, 3]`이 intensity다. intensity가 없는 미래
point type을 NaN이나 임의 값으로 추정하지 않는다. 별도 representation version을
정의한 뒤 additive field로 추가한다.

### 7.2 copy와 ownership

- `RuntimeClient::Visualization()`이 반환한 value snapshot은 이미 committed runtime에서
  분리된 derived copy다.
- binding은 moved `std::vector<VisualizationPoint>`를 capsule/native holder가 소유하게
  하고 NumPy array가 그 holder를 base로 참조하게 한다.
- 따라서 runtime close 후에도 array가 유효하고, Python mutation으로 runtime이 바뀌지
  않는다.
- `VisualizationPoint`의 standard-layout/field offset/size가 4개의 연속 `float` 조건을
  만족하는지 `static_assert`한다. 조건이 깨지면 build를 실패시키며 조용히 잘못된 stride를
  노출하지 않는다.
- array의 writable flag는 false로 설정한다.
- pose matrix는 Eigen storage layout을 직접 노출하지 않고 새 C-contiguous NumPy buffer에
  명시적으로 복사한다.

points의 추가 full payload copy를 만들지 않는 것이 목표다. 다만 기존 C++ visualization
snapshot 생성 자체의 copy는 남는다. small fixture에서 peak RSS와 payload bytes를
측정해 문서화하고, Python 변환 때문에 point payload가 한 번 더 완전 복제되면 완료로
판정하지 않는다.

## 8. Build와 package 구조

### 8.1 구현 directory

```text
open_lmm/
  src/adapters/python/
    CMakeLists.txt
    module.cpp
    runtime_binding.cpp
    runtime_binding.hpp
    model_conversion.cpp
    model_conversion.hpp
    numpy_conversion.cpp
    numpy_conversion.hpp
    pyproject.toml
    build-constraints.txt
    build_local_wheel.sh
    README.md
    python_public_api_v1.txt
    package/open_lmm/
      __init__.py
      _api.py
      _errors.py
      _models.py
      py.typed
    examples/
      basic_runtime.py
      cancellation.py
      events.py
      visualization_numpy.py
  test/adapters/python/
    python_test_fixture.py
    test_public_api.py
    test_runtime.py
    test_config.py
    test_errors.py
    test_callbacks.py
    test_lifetime.py
    test_numpy.py
    test_wheel_install.py
    measure_numpy_memory.py
```

binding C++ source를 `python/` 아래 production-agnostic utility로 숨기지 않는다. 기존
dependency graph의 leaf adapter임을 드러내기 위해 `src/adapters/python`에 둔다.
PEP 517 metadata, pure-Python package, examples도 같은 adapter owner 아래 두고, 검증 코드는
기존 test architecture에 맞춰 `test/adapters/python`에 분리했다.

### 8.2 CMake integration

`cmake/options.cmake`:

```cmake
option(OPEN_LMM_BUILD_PYTHON "Build the optional CPython binding" OFF)
```

`open_lmm/CMakeLists.txt`에서 `open_lmm_client` 생성 이후에만 Python adapter를 추가한다.

```cmake
if(OPEN_LMM_BUILD_PYTHON)
  add_subdirectory(src/adapters/python)
endif()
```

native module target 규칙:

```text
target: open_lmm_python_native
output: open_lmm/_native.*
link: pybind11::module, open_lmm_client
include: open_lmm/include의 installed public surface만
forbidden link: open_lmm_map_server 및 모든 private object target
forbidden source: open_lmm/src/runtime, config, domain, plugin-host production .cpp
RPATH: wheel 안에서는 $ORIGIN/.libs, build tree에서는 기존 target graph
```

adapter target은 production `.cpp`를 다시 compile하지 않는다. `RuntimeClient`를 link하는
하나의 native module만 만든다.

### 8.3 local wheel contract

Goal 06 wheel은 다음 조건의 local platform wheel이다.

- tag: CPython 3.10 / Linux x86-64 platform-specific
- fresh venv에 설치 가능
- source tree가 없는 상태에서 import와 tiny fixture E2E 가능
- OpenLMM-owned runtime DSOs와 활성 built-in plugin DSOs를 wheel의
  `open_lmm/.libs`에 stage
- extension과 bundled OpenLMM DSO의 relative RPATH 검증
- `__version__ == 3.0.0`, license/notice 포함
- 시스템 compiler/stdlib/PCL/GTSAM 계열 compatibility는 현재 release matrix와 동일

이 단계는 manylinux 호환이나 PyPI 게시 가능성을 주장하지 않는다. third-party DSO
closure, auditwheel policy, wheel matrix와 release signing은 Goal 09 책임이다.

## 9. 구현 작업 패키지

각 패키지는 앞 패키지의 contract test가 통과한 뒤 진행한다.

### P00 — baseline과 API lock

- 이 문서의 API signature와 non-goal 승인
- 현재 full CTest, architecture policy, `git diff --check` 기준 기록
- `python_public_api_v1.txt` golden manifest 작성
- public symbol 추가/삭제 시 explicit compatibility review gate 연결

완료 증거: API manifest와 baseline test log.

### P01 — optional build/package skeleton

- `OPEN_LMM_BUILD_PYTHON=OFF` 추가
- `src/adapters/python/pyproject.toml`, package source, typing marker 작성
- pybind11 module 최소 `API_VERSION`, `__version__` 노출
- Python OFF build가 기존 configure 결과를 바꾸지 않는지 검증

완료 증거: OFF clean build PASS, ON import PASS.

### P02 — Result/error와 value conversion

- 모든 현재 `Error::Code` converter와 Python exception hierarchy 구현
- Error context, enum, snapshot/receipt converter 구현
- native `Result<T>`가 module boundary 밖으로 새지 않게 함
- unknown enum/error의 fail-loud test 추가

완료 증거: error code/context table-driven tests.

### P03 — Runtime/Job core

- `Runtime.open`, `run_all`, `run_stage`, `Job.wait`, 양쪽 cancel,
  `snapshot`, `is_open`, `close` 구현
- foreign Job rejection과 owner destruction contract 구현
- context manager와 idempotent close 구현

완료 증거: Open→Run→Wait→Snapshot→Close와 cancellation integration PASS.

### P04 — GIL and shutdown

- operation별 GIL policy 적용
- blocking wait/close 중 다른 Python thread progress test
- Runtime/Job GC, active job close, interpreter shutdown subprocess test
- destructor exception/no-self-join 검증

완료 증거: deterministic L5 tests와 timeout 없는 정상 shutdown.

### P05 — event callback/subscription

- event DTO converter와 subscription holder 구현
- callback GIL reacquire, unsubscribe drain, Python exception reporting 구현
- callback 중 read-only reentry, callback self-unsubscribe, close rejection 검증
- callback reference cycle/GC test 추가

완료 증거: exactly-once terminal event와 post-unsubscribe callback 0건.

### P06 — visualization/NumPy

- query와 immutable NumPy conversion 구현
- point buffer moved ownership/capsule, pose explicit copy 구현
- dtype/shape/stride/read-only/lifetime test 추가
- conversion peak memory 계측과 full payload duplicate 부재 확인

완료 증거: runtime close 후 array access PASS, PCL symbol/type 노출 0건.

### P07 — ApplyConfig

- `ConfigDomain`, `Revision`, `ConfigApplyReceipt` 구현
- expected revision을 필수 keyword로 강제
- stale revision, malformed JSON, committed receipt mapping 검증
- Python이 config/state authority를 cache하지 않음을 확인

완료 증거: successful revision advance와 stale failure 보존 PASS.

### P08 — local wheel staging

- OpenLMM-owned DSO/plugin staging과 relative RPATH 구현
- source-free fresh venv install/import/E2E runner 구현
- wheel 내부 절대 build/source path와 private header 부재 검사
- license/notice/version metadata 추가

완료 증거: unpacked wheel audit와 fresh venv tiny fixture PASS.

### P09 — architecture/test registration

- Python adapter 전체에 public-header-only/leaf-link architecture gate 추가
- Python CTest를 `openlmm_add_test`로 owner/layer/invariant 등록
- 기존 C++ regression suite, package consumer, sanitizer 영향 검증
- fixed sleep/yield race oracle 금지 유지

완료 증거: architecture policy, manifest, full CTest PASS.

### P10 — docs/examples/final evidence

- API reference, four examples, local wheel build/install guide 작성
- supported/unsupported matrix와 known limitation 기록
- 이 문서에 exact commands, test counts, artifact SHA-256, measured memory 추가
- 미완료 hosted/release 항목을 Goal 09 backlog로 연결

완료 증거: examples가 fresh venv에서 실행되고 결과 문서가 실제 HEAD와 일치.

## 10. Test specification

### 10.1 등록 test portfolio

| Test | Layer | Module | Owner | Invariants | 필수 검증 |
|---|---|---|---|---|---|
| `open_lmm_python_public_api_tests` | L2 | adapters.python.api | PythonBinding | INV-13,14 | golden exports, private symbol 부재 |
| `open_lmm_python_error_tests` | L2 | adapters.python.errors | PythonBinding | INV-07,14,18 | code/severity/context mapping |
| `open_lmm_python_runtime_tests` | L3 | adapters.python.runtime | RuntimeClient | INV-05,06,07,08 | open/run/wait/cancel/snapshot/close |
| `open_lmm_python_config_tests` | L3 | adapters.python.config | RuntimeClient | INV-03,04,07 | expected revision/commit/error |
| `open_lmm_python_callback_tests` | L5 | adapters.python.events | PythonSubscription | INV-08,17 | GIL/reentry/unsubscribe/exception |
| `open_lmm_python_lifetime_tests` | L5 | adapters.python.lifetime | PythonRuntimeHolder | INV-08,17 | GC/close/shutdown/subprocess |
| `open_lmm_python_numpy_tests` | L3 | adapters.python.visualization | PythonBufferOwner | INV-09,10,16 | shape/ownership/lifetime/RSS |
| `open_lmm_python_wheel_tests` | L3 | package.python | PythonWheel | INV-13,14 | source-free install/import/E2E |

Python command test도 raw `add_test`를 사용하지 않고 `openlmm_add_test(COMMAND ...)`로
등록한다. `MODULE`, `OWNER`, `LAYER`, `LANES` metadata를 생략할 수 없다.

### 10.2 필수 behavior cases

#### Runtime

- invalid max task count rejection
- open success and double-open failure
- run-all success
- each public stage submit/wait
- pre-completion cancel and cancelled error mapping
- terminal success 후 cancel이 성공 receipt를 뒤집지 않음
- wrong runtime Job rejection
- snapshot revision/config/job/artifact/event conversion
- close with active job: reject mode and cancel-and-wait mode
- repeated close success and reopen policy 확인

late-cancel과 recovery semantics의 exhaustive race는 기존 C++ owner test가 계속 담당한다.
Python test는 binding이 그 result를 재해석하지 않는지만 검증한다.

#### Error/config

- missing config, malformed document, plugin preflight, I/O error
- every native error code의 subclass/code/context preservation
- stale expected revision failure 뒤 committed snapshot 불변
- successful apply receipt와 authoritative snapshot revision 일치

#### GIL/callback/lifetime

- `Job.wait()` 동안 독립 Python thread가 phase signal에 응답
- close callback drain 중 GIL deadlock 없음
- event sequence monotonic, terminal event exactly once
- unsubscribe 반환 후 callback 0건
- callback exception 이후 다음 event 수신 가능
- callback 안 snapshot 성공
- callback 안 close가 즉시 명시적 error이며 self-join 없음
- callback self-unsubscribe 안전
- active job/subscription 상태에서 object GC 및 process exit 정상

이 test들은 `sleep`, polling, 임의 `yield`를 correctness oracle로 사용하지 않는다.
`threading.Event`, `Barrier`, callback signal, subprocess timeout을 사용한다.

#### NumPy

- empty and non-empty point array
- exact `float32`, `(N,4)`, C-compatible strides
- `WRITEABLE == False`
- xyz/intensity field value parity
- pose transform direction/value parity
- runtime/snapshot native owner 삭제 후 array lifetime
- Python array 삭제 시 holder exactly-once destruction
- PCL Python type/capsule/address 노출 0건
- large point conversion에서 추가 full-size point copy 부재

#### Wheel

- wheel file name/version/tag
- fresh venv `pip install --no-deps`
- source directory를 숨긴 상태에서 import
- `_native`, OpenLMM-owned DSOs, built-in plugins의 relative dependency resolution
- basic fixture Open→RunAll→Wait→Snapshot→Close
- wheel 내 `/root/workspace`, build tree, private header 문자열/경로 부재
- license and notice 존재

## 11. Validation commands and lanes

구현 후 최소 로컬 검증 순서는 다음과 같다. 정확한 build directory 이름은 구현 시
결과 문서에 기록한다.

```text
1. clean default configure/build with OPEN_LMM_BUILD_PYTHON=OFF
2. architecture/release policy
3. isolated CPython 3.10 venv + pinned build requirements
4. clean Python-enabled GCC 12 configure/build
5. Python-labeled CTest
6. full C++ CTest
7. local wheel build
8. source-free fresh-venv wheel install/import/E2E
9. applicable ASan+UBSan native bridge tests
10. git diff --check
```

Compiler matrix 목표:

| Compiler | CPython | Goal 06 local gate |
|---|---:|---|
| GCC 12 + libstdc++ | 3.10 | required |
| Clang 15 + documented libstdc++ 12 set | 3.10 | required before completion |
| GCC 13 + libstdc++ | 3.10 | existing release matrix 영향 검증 |

TSan-instrumented CPython execution은 Goal 06 required gate로 주장하지 않는다. native core의
기존 TSan suite는 계속 통과해야 하며, binding callback race는 deterministic Python L5와
ASan+UBSan 가능한 bridge test로 검증한다.

## 12. Supported Python matrix

Python API v1 최초 지원 범위:

| Runtime | Status | 이유 |
|---|---|---|
| CPython 3.10, Ubuntu 22.04 x86-64 | **supported** | release OS 기본 Python이며 local 기준 존재 |
| CPython 3.11/3.12 | provisional | clean wheel build/install matrix가 Goal 09에서 추가된 뒤 승격 |
| CPython free-threaded | unsupported | GIL/lifetime contract 재검토 필요 |
| PyPy | unsupported | extension/lifetime 검증 없음 |
| Windows/macOS | unsupported | native dependency와 plugin loader release matrix 밖 |
| source-only install from PyPI | unsupported | Goal 09 release pipeline 전에는 배포하지 않음 |

지원은 import만 의미하지 않는다. 해당 row에서 wheel install과 public workflow E2E가
required gate여야 supported로 표기한다.

## 13. Compatibility and versioning

- Python distribution version은 OpenLMM project version과 동기화한다.
- `API_VERSION = 1`은 Python surface generation이다. Plugin ABI version과 관계없다.
- public export, signature, enum 의미, exception attribute, NumPy dtype/shape의 breaking
  change는 explicit compatibility review가 필요하다.
- `_native`는 private이며 direct import를 문서화하지 않는다.
- C++ Runtime API의 보장은 기존 release policy의 source/rebuild compatibility를 넘지
  않는다.
- wheel에 native DSO를 포함해도 portable external plugin ABI를 약속하지 않는다.
- additive field는 Python dataclass constructor compatibility와 pattern matching 영향을
  검토한 후 추가한다.

`python_public_api_v1.txt`와 API introspection test는 accidental export를 막는 golden
policy다. production source에서 자동 생성하지 않는다.

## 14. Non-goals and deferred work

Goal 06에서 하지 않는다.

- `RuntimeService`, controller, executor, state store의 direct binding
- Python 구현 algorithm/plugin을 production hot path에서 호출
- PCL/Eigen object를 public Python type으로 노출
- asyncio-native job/event API
- node graph 편집 또는 arbitrary execution request
- interactive alignment feedback API
- root-config runtime replacement API
- pandas/experiment/parameter sweep/replay automation — Goal 07
- manylinux/multi-platform/public wheel/release promotion — Goal 09
- generic point cloud representation 교체 — Goal 10
- portable external plugin SDK 약속 — Goal 15

위 항목은 필요 시 additive follow-up으로 제안한다. frozen invariant를 깨야 한다면 구현에
섞지 않고 architecture change admission을 먼저 수행한다.

## 15. Risk and rollback

| Risk | 방지/검증 | Rollback boundary |
|---|---|---|
| callback/GIL deadlock | unsubscribe 시 GIL release, deterministic L5 | P05 단독 제거 가능 |
| interpreter shutdown UAF | atexit drain, subprocess tests | callback API를 v1 export에서 제외 |
| NumPy hidden copy/peak RSS | moved owner + memory measurement | visualization API만 제외, core 유지 |
| DSO/plugin resolution failure | `.libs` relative RPATH source-free test | local wheel staging P08 rollback |
| private type leak | include/link/symbol/API golden gate | adapter target 변경만 rollback |
| C++ behavior 재해석 | Result translator 단일화 | converter commit rollback |
| default build dependency 증가 | option OFF, no Python discovery | P01 option/subdirectory rollback |

구현 commit은 P01~P10 경계를 가능한 한 유지한다. Python adapter 제거가 runtime core,
public C++ header, plugin ABI에 영향을 주지 않아야 한다.

## 16. Completion conditions

다음 조건은 2026-08-26의 Goal 06 worktree에서 모두 충족됐다. 세부 실행 근거는
§18에 고정한다.

- [x] pybind11 선택과 pin된 build environment 기록
- [x] Python public API와 golden export manifest 구현
- [x] RuntimeClient/Job/error/snapshot 최소 binding 구현
- [x] Open/RunAll/RunStage/Wait/Snapshot/Cancel/Close PASS
- [x] ApplyConfig expected-revision contract PASS
- [x] GIL responsiveness PASS
- [x] callback GIL, unsubscribe, exception, destruction PASS
- [x] NumPy `(N,4)` semantic representation과 ownership/lifetime PASS
- [x] PCL/private runtime type public 노출 0건
- [x] Python adapter forbidden-edge gate PASS
- [x] CPython 3.10 GCC 12 및 Clang 15 local clean validation PASS
- [x] source-free fresh-venv local wheel install/import/E2E PASS
- [x] existing C++ full CTest/package/policy regression PASS
- [x] examples와 supported/unsupported matrix 문서화 및 실행 확인
- [x] exact baseline HEAD, command, test count, artifact hash, memory measurement 기록

Goal 06 완료 후 Goal 07의 experiment toolkit 설계를 시작할 수 있다. 그러나 stable/public
wheel 선언은 Goal 03–05 hosted blocker와 Goal 09 packaging/release gate가 닫힌 뒤에만
가능하다.

## 17. 구현 완료 판정

**PASS — Goal 06 local implementation complete.**

public `RuntimeClient` PImpl, runtime lifecycle/epoch owner, test ownership 체계와 package
target을 그대로 사용해 Python leaf adapter를 구현했다. private runtime include/link,
두 번째 lifecycle/state owner, Python callback 기반 algorithm hot path, public PCL exposure는
추가하지 않았다. 따라서 이 범위에는 architecture change proposal이 필요하지 않았다.

이 판정은 local CPython 3.10 Linux wheel에 한정된다. portable/public wheel과 release
promotion은 Goal 09가 소유한다.

## 18. 실제 구현 및 검증 증거

### 18.1 변경 범위와 owner

| 영역 | 실제 구현 | 경계 판정 |
|---|---|---|
| native leaf adapter | `src/adapters/python/{module,runtime_binding,model_conversion,numpy_conversion}.*` | `open_lmm_client`만 link |
| Python public surface | `package/open_lmm/{__init__,_api,_errors,_models}.py`, `py.typed` | `_native`는 private |
| API lock | `python_public_api_v1.txt`, `test_public_api.py` | 46개 public export golden 비교 |
| optional build | `OPEN_LMM_BUILD_PYTHON=OFF`, adapter CMake | OFF에서 adapter NumPy/pybind11 discovery와 binding compile 0건 |
| local packaging | `pyproject.toml`, constraints, wheel helper, `cmake/python_wheel.cmake` | DSO staging은 packaging owner, adapter link는 leaf 유지 |
| executable contracts | Python CTest 7개 + installed-wheel CTest 1개 | L2/L3/L5와 INV metadata 등록 |
| docs/examples | README와 4개 example | fresh wheel venv에서 4개 workflow 실행 PASS |

wheel helper는 ScanContext와 FreeDOM을 포함한 최소 local runtime closure를 stage한다.
Python adapter source/CMake에는 `open_lmm_map_server`나 private object target dependency를
추가하지 않았다. architecture gate는 Python ON일 때 각 binding production source가 정확히
한 번 compile되고, OFF일 때 0번 compile되는 것도 검사한다.

### 18.2 compiler, test, sanitizer 결과

| 구성 | 결과 |
|---|---|
| GCC 12.3, Release, Python 기본 OFF clean build | 전체 build PASS, CTest **81/81 PASS**; installed package consumer 포함 |
| GCC 12.3, Release, Python ON | 전체 build PASS, CTest **88/88 PASS**; Python 7 + wheel 1 포함 |
| Clang 15.0.7 + libstdc++ 12, Release, Python ON clean build | native module build PASS, Python CTest **7/7 PASS** |
| GCC 12.3, Debug, ASan+UBSan, Python ON clean build | Python CTest **7/7 PASS** |
| architecture/release policy | `scripts/ci/check_architecture_policy.sh` PASS |
| formatting | `git diff --check` PASS |

ASan 실행은 non-instrumented CPython에 sanitizer runtime과 C++ exception runtime을 먼저
제공하기 위해 다음 preload 순서를 사용했다. 이 조건이 없으면 ASan 자체의
`real___cxa_throw` interceptor 초기화 검사가 실패하며, 이는 binding memory defect 판정이
아니다.

```bash
cmake -S open_lmm -B /tmp/openlmm-goal06-asan-build \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
  -DCMAKE_BUILD_TYPE=Debug -DBUILD_TESTING=ON \
  -DOPEN_LMM_BUILD_PYTHON=ON -DOPEN_LMM_ENABLE_ASAN_UBSAN=ON \
  -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF
cmake --build /tmp/openlmm-goal06-asan-build \
  --target open_lmm_python_native -j2

env \
  LD_PRELOAD=/usr/lib/gcc/x86_64-linux-gnu/12/libasan.so:/usr/lib/x86_64-linux-gnu/libstdc++.so.6 \
  ASAN_OPTIONS=detect_leaks=0:halt_on_error=1 \
  UBSAN_OPTIONS=halt_on_error=1:print_stacktrace=1 \
  ctest --test-dir /tmp/openlmm-goal06-asan-build \
    -R '^open_lmm_python_(public_api|error|runtime|config|callback|numpy|lifetime)_tests$' \
    --output-on-failure
```

주요 재현 명령은 다음과 같다.

```bash
# Python 기본 OFF clean regression
cmake -S open_lmm -B /tmp/openlmm-goal06-gcc12-off-build \
  -DCMAKE_C_COMPILER=/usr/bin/gcc-12 \
  -DCMAKE_CXX_COMPILER=/usr/bin/g++-12 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
cmake --build /tmp/openlmm-goal06-gcc12-off-build -j2
ctest --test-dir /tmp/openlmm-goal06-gcc12-off-build --output-on-failure -j2

# GCC Python-enabled 전체 regression
cmake --build /root/workspace/build/open_lmm -j2
ctest --test-dir /root/workspace/build/open_lmm --output-on-failure -j2

# Clang 15 Python binding
cmake -S open_lmm -B /tmp/openlmm-goal06-clang15-build \
  -DCMAKE_C_COMPILER=/usr/bin/clang-15 \
  -DCMAKE_CXX_COMPILER=/usr/bin/clang++-15 \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON \
  -DOPEN_LMM_BUILD_PYTHON=ON -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF \
  -DOPEN_LMM_BUILD_DESCRIPTOR_SOLID=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD=OFF \
  -DOPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR=OFF \
  -DCMAKE_CXX_FLAGS='-nostdinc++ -isystem /usr/include/c++/12 -isystem /usr/include/x86_64-linux-gnu/c++/12 -isystem /usr/include/c++/12/backward'
cmake --build /tmp/openlmm-goal06-clang15-build \
  --target open_lmm_python_native -j2
ctest --test-dir /tmp/openlmm-goal06-clang15-build \
  -R '^open_lmm_python_(public_api|error|runtime|config|callback|numpy|lifetime)_tests$' \
  --output-on-failure

bash scripts/ci/check_architecture_policy.sh
git diff --check
```

### 18.3 wheel artifact와 source-free 실행

최종 로컬 artifact:

```text
file: open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl
size: 7,685,793 bytes
sha256: fb3417b3104a887c45e9c7858fb3aec399e0c80b7806360f481ed08ed92a2db9
```

생성 및 격리 검증 명령:

```bash
env -u ROS_VERSION -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u PYTHONPATH \
  SKBUILD_BUILD_DIR=/tmp/openlmm-goal06-pep517-build \
  open_lmm/src/adapters/python/build_local_wheel.sh \
  /tmp/openlmm-goal06-build-venv-v2/bin/python \
  /tmp/openlmm-goal06-wheelhouse-v3

/tmp/openlmm-goal06-install-venv-v3/bin/python -m pip install \
  numpy==1.21.5 \
  /tmp/openlmm-goal06-wheelhouse-v3/open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl

env -u LD_LIBRARY_PATH -u PYTHONPATH -u ROS_VERSION \
  -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
  /tmp/openlmm-goal06-install-venv-v3/bin/python -I \
  /root/workspace/src/open-lmm/open_lmm/test/adapters/python/test_wheel_install.py
```

fresh venv의 isolated mode에서 3/3이 통과했고 Open→RunAll→Wait→Snapshot→Visualization을
실행했으며, 같은 환경에서 ROS/Python path를 제거한 `pip check`도 PASS했다. unpack audit
결과 `.a`, `/root/workspace`, Goal 06 임시 build path는 0건이었다.
native module RUNPATH는 `$ORIGIN/.libs`, bundled `libopen_lmm_client.so.3.0.0` RUNPATH는
`$ORIGIN`이다. wheel에는 `LICENCE`와 `THIRD_PARTY_NOTICES.md`가 포함된다.

### 18.4 NumPy ownership와 memory evidence

20,088개 point의 `(N,4)` float32 visualization에서 얻은 값:

```json
{
  "displayed_point_count": 20088,
  "numpy_owner_bytes": 321408,
  "max_rss_before_kib": 84308,
  "max_rss_after_kib": 84308,
  "max_rss_delta_kib": 0
}
```

`321408 == 20088 * 4 * sizeof(float)`이므로 capsule이 소유한 payload는 semantic point
buffer 정확히 하나다. test는 dtype, shape, stride, read-only flag, 값 parity, base owner,
runtime close 후 lifetime을 함께 검증한다. `ru_maxrss`의 관측 delta 0은 보조 계측이며,
그 값만으로 peak copy 부재를 추론하지 않는다.

### 18.5 기준 commit과 남은 release 범위

검증한 source 기준점은 `develop`의
`27b39f726f79651b827d192d0c23cc5546531daf`이고, Goal 06 구현은 그 위의 미커밋
worktree다. 따라서 이 문서는 존재하지 않는 구현 commit SHA를 만들지 않는다. 이후
commit할 때에는 이 기준점과 Goal 06 diff를 분리해 추적한다.

남은 항목은 구현 결함이 아니라 Goal 09 release scope다.

- manylinux/auditwheel 및 third-party portable DSO closure
- CPython 3.11/3.12와 다중 OS/architecture matrix
- public index 게시, SBOM, provenance, signing, RC promotion
- external plugin SDK의 portable ABI 판단
