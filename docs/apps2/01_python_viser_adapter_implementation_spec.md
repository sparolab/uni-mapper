# Python Viser Compatibility Adapter 구현명세서

- 상태: **IMPLEMENTED / VERIFICATION COMPLETE**
- 구현 기준일: **2026-08-28**
- 목표 구현 위치: `applications/python/viser`
- Python distribution: `open-lmm-viser`
- console entry: `open-lmm-viser`
- 기준 OpenLMM Python API: v3.0.0 / CPython 3.10
- 기준 viser API: v1.1 release family

구현은 `applications/python/viser`의 독립 wheel에 한정되며 `open_lmm/core`,
public C++ API/ABI, plugin ABI는 수정하지 않았다. P3 console owner cutover는
`applications/python/experiment`의 별도 package owner로 먼저 분리했다.

## 1. 목적과 완료 경계

이 작업은 OpenLMM의 committed `VisualizationSnapshot`을 Python에서 받아 viser의
point-cloud와 line-segment scene으로 표시하는 독립 application adapter를 추가한다.

첫 delivery는 다음을 완료한다.

- agent별 committed point cloud 표시
- agent별 committed trajectory 표시
- artifact/stage/job commit 뒤 자동 refresh
- 여러 event가 몰릴 때 agent별 최신 refresh로 coalescing
- stale revision 차단
- 새 presentation 준비 전까지 기존 visible presentation 유지
- `open-lmm-viser` CLI와 재사용 가능한 `ViserAdapter` 제공
- installed `open-lmm` wheel만 소비하는 독립 Python wheel 구성

다음은 첫 delivery에 포함하지 않는다.

- GSAT 실행, GSAT score 또는 traversability overlay
- `DataLoader`, `DynamicRemover`, `Descriptor` 같은 알고리즘 직접 Python binding
- loop edge와 pose-axis 표시
- raw scan의 frame-rate streaming 또는 playback timeline
- config 편집, stage 실행 버튼, cancellation, alignment feedback 같은 runtime 제어 UI
- remote runtime attach 또는 별도 RPC protocol
- browser authentication, TLS termination 또는 public deployment
- core public API/ABI, plugin ABI, Python binding API 변경

## 2. 선행 결정과 P3 경계

기존 Python application extraction은 `docs/apps/04_python_application_extraction.md`에서
`DEFERRED / OPTIONAL`로 유예돼 있다. Viser application은 다음 두 재개 조건을 동시에
충족한다.

- Python user-facing application이 둘 이상 생긴다.
- SDK와 다른 크기의 optional dependency가 추가된다.

따라서 Viser implementation에 앞서 P3를 별도 작업으로 재개했다. P3는
`open-lmm-experiment`의 console application owner를
`applications/python/experiment`로 이동하는 독립 명세와 commit으로 수행한다.

Viser package 자체는 experiment code나 entry point를 참조하지 않는다. Source control
반영 시 P3 owner cutover와 Viser application은 각각 독립 review 단위로 나눠야 한다.

## 3. Architecture와 ownership

목표 dependency 방향은 다음과 같다.

```text
RuntimeStateStore
      ↓
RuntimeService
      ↓
RuntimeClient
      ↓
bindings/python: open_lmm.Runtime
      ↓
applications/python/viser: ViserAdapter + CLI
      ↓
viser server/browser
```

`open_lmm.Runtime`과 그 native bridge가 authoritative runtime owner다. `ViserAdapter`는
다음 derived presentation 정보만 소유한다.

- event subscription
- bounded refresh intent
- agent별 requested generation과 displayed revision
- candidate NumPy payload
- viser scene handle

Adapter가 runtime snapshot, artifact 또는 algorithm output의 두 번째 canonical owner가
되어서는 안 된다. 실패 뒤 retry 여부와 agent 존재 여부는 항상 public Runtime query로
재확인한다. 로그만으로 future behavior를 바꾸지 않는다.

다음 dependency는 금지한다.

- core 또는 `bindings/python`에서 `viser` import/의존
- core build/install에서 `applications/python/viser` source 참조
- Viser package에서 `open_lmm._native` 직접 import
- Viser package에서 core private header/source/plugin host 접근
- Viser package가 native runtime/plugin DSO를 다시 bundle
- 별도 `RuntimeStateStore`, runtime repository 또는 plugin composition owner 생성

## 4. 목표 package 구조

```text
applications/python/viser/
├── pyproject.toml
├── README.md
├── packaging/
│   ├── RELEASE_POLICY.md
│   └── THIRD_PARTY_NOTICES.md
├── package/open_lmm_viser/
│   ├── __init__.py
│   ├── __main__.py
│   ├── adapter.py
│   ├── conversion.py
│   └── cli.py
└── test/
    ├── test_adapter.py
    ├── test_conversion.py
    ├── test_cli.py
    └── test_package.py
```

Production package의 canonical owner는 이 directory 하나다. 예제나 forwarding copy를
`bindings/python/examples`에 중복해서 두지 않는다. Binding README에는 별도 viewer
package를 가리키는 짧은 링크만 추가할 수 있다.

## 5. Distribution과 dependency 계약

`pyproject.toml`은 다음 package contract를 선언한다.

```toml
[project]
name = "open-lmm-viser"
version = "3.0.0"
requires-python = ">=3.10,<3.11"
dependencies = [
  "open-lmm==3.0.0",
  "viser>=1.1,<2",
]

[project.scripts]
open-lmm-viser = "open_lmm_viser.cli:main"
```

지원 범위는 기존 Python binding v1과 같은 CPython 3.10, Ubuntu 22.04 x86-64,
same-image local wheel이다. Public PyPI wheel, manylinux portability, Python 3.11+, Windows,
macOS 지원을 새로 주장하지 않는다.

Viser는 application wheel의 Python dependency일 뿐 OpenLMM runtime closure에 포함하지
않는다. Viser license와 transitive dependency notice는 package-local
`THIRD_PARTY_NOTICES.md`에 기록한다.

## 6. Public Python API

`open_lmm_viser`의 v1 public surface는 다음 하나로 제한한다.

```python
class ViserAdapter:
    def __init__(
        self,
        runtime: open_lmm.Runtime,
        server: viser.ViserServer,
        *,
        preview_voxel_size_m: float | None = None,
        point_size: float = 0.03,
        trajectory_thickness: float = 0.03,
    ) -> None: ...

    def start(self) -> None: ...
    def close(self) -> None: ...
    def __enter__(self) -> "ViserAdapter": ...
    def __exit__(self, exc_type, exc, traceback) -> None: ...
```

계약은 다음과 같다.

- `runtime`은 이미 `open()`이 성공한 상태여야 한다.
- Adapter는 전달받은 runtime과 server를 close하지 않는다.
- `start()`는 최초 authoritative snapshot 조회, event subscription, refresh worker 시작을
  완료한 뒤 반환한다.
- 같은 instance에서 두 번째 `start()`는 `RuntimeError`다.
- `close()`는 idempotent하며 subscription close, worker stop/join, owned scene handle 제거
  순서로 수행한다.
- close한 instance는 restart할 수 없다.
- voxel size, point size, trajectory thickness는 finite positive value만 허용한다.
- application package는 public semantic-version compatibility를 v1 범위 이상으로
  확대해 주장하지 않는다.

## 7. CLI 계약

```text
open-lmm-viser CONFIG_DIR
  [--label TEXT]
  [--output-root PATH]
  [--host HOST]
  [--port PORT]
  [--preview-voxel-size-m FLOAT]
```

기본값은 다음과 같다.

| Option | Default | 의미 |
|---|---:|---|
| `--label` | `viser` | Runtime label |
| `--output-root` | 미지정 | Runtime 기본 output root 사용 |
| `--host` | `127.0.0.1` | loopback 전용 bind |
| `--port` | `8080` | viser HTTP/WebSocket port |
| `--preview-voxel-size-m` | 미지정 | Runtime visualization 기본값 사용 |

CLI lifecycle은 다음 순서를 고정한다.

1. argument를 검증한다.
2. `ViserServer`를 시작한다.
3. `Runtime.open()`을 수행한다.
4. `ViserAdapter.start()`로 초기 refresh와 event subscription을 설치한다.
5. `Runtime.run_all()`을 제출하고 `Job.wait()`한다.
6. 성공 뒤 server를 유지해 browser가 결과를 볼 수 있게 한다.
7. SIGINT에서 adapter, runtime, server 순서로 정상 종료한다.

Exit code는 다음과 같다.

| Exit | 의미 |
|---:|---|
| `0` | pipeline 성공 뒤 사용자 종료 |
| `1` | server bootstrap, runtime open, pipeline 또는 adapter failure |
| `2` | usage 또는 argument validation failure |

CLI는 browser를 자동으로 열지 않는다. SSH와 headless 환경에서 URL을 stdout에 한 번
표시한다.

## 8. Refresh lifecycle과 concurrency

### 8.1 최초 refresh

`start()`는 `runtime.snapshot()`의 `pipeline.agents`를 authoritative catalog로 사용한다.
각 agent에 latest generation을 할당하고 refresh intent를 enqueue한다. 최초 query가
실패한 agent는 scene을 만들지 않고 다음 committed trigger를 기다린다.

### 8.2 Event trigger

다음 event만 visualization refresh trigger로 사용한다.

- `ARTIFACT_COMMITTED`
- `ARTIFACT_INVALIDATED`
- `STAGE_COMPLETED`
- `JOB_COMPLETED`

`PROGRESS_UPDATED`, node start, stage start만으로 full point refresh를 수행하지 않는다.
Event callback은 `affected_agents`, 단일 `agent`, 또는 authoritative catalog refresh가
필요하다는 작은 intent만 기록한다. Callback thread에서 다음 작업을 금지한다.

- `runtime.visualization()` 호출
- point array 복사와 color conversion
- viser scene API 호출
- file read
- blocking wait

### 8.3 Bounded worker

Adapter는 worker thread 하나와 agent별 latest-generation slot 하나를 사용한다. 같은
agent의 대기 요청은 새 generation이 이전 generation을 덮어쓴다. Pending state는 현재
authoritative agent 수보다 커질 수 없다.

Worker는 agent별로 다음을 수행한다.

1. 요청 generation을 읽는다.
2. `runtime.visualization(agent, preview_voxel_size_m=...)`을 조회한다.
3. points, colors, trajectory candidate를 완전히 변환하고 검증한다.
4. generation과 snapshot revision이 여전히 최신인지 재확인한다.
5. `server.atomic()` 안에서 같은 scene name으로 candidate를 교체한다.
6. 성공 뒤에만 displayed revision과 handles를 갱신한다.

조회와 conversion 도중 더 최신 generation이 생기면 결과를 폐기한다. Snapshot revision이
displayed revision보다 작으면 stale result로 폐기한다.

## 9. Presentation commit 정책

정상 갱신은 다음 순서를 지킨다.

```text
visible scene
    ↓
candidate snapshot query
    ↓
NumPy conversion + validation
    ↓
generation/revision validation
    ↓
viser atomic same-name replacement
    ↓
new visible scene
```

Candidate가 준비되기 전에 기존 scene을 clear/remove하면 안 된다. 다음 상황에서는 이전
valid scene을 유지한다.

- runtime query failure
- points 또는 poses shape mismatch
- non-finite coordinate
- conversion exception
- stale revision
- superseded generation
- temporary `points_available == false`

Authoritative `runtime.snapshot().pipeline.agents`에서 agent 제거가 확인된 경우에만 해당
agent의 owned scene handles를 제거한다. Query failure를 agent removal로 해석하지 않는다.

## 10. Visualization 변환 계약

### 10.1 Scene coordinate

OpenLMM point와 pose를 runtime global/map frame 값으로 해석하며 추가 transform을 적용하지
않는다. Viser scene의 up direction은 `+Z`로 설정한다.

Agent ID는 다음 방식으로 scene-safe key로 변환한다.

```python
urllib.parse.quote(agent, safe="-_.~")
```

Scene name은 고정한다.

```text
/open_lmm/agents/<agent-key>/points
/open_lmm/agents/<agent-key>/trajectory
```

### 10.2 Point cloud

입력 계약은 기존 Python binding을 그대로 사용한다.

```text
dtype: float32
shape: (N, 4)
columns: x, y, z, intensity
ownership: read-only NumPy array
```

변환은 다음과 같다.

- `points[:, :3]`을 contiguous `float32 (N, 3)` array로 만든다.
- XYZ에 NaN/Inf가 있으면 candidate 전체를 거부한다.
- intensity는 finite min/max로 `[0, 255]`에 선형 정규화한다.
- intensity가 상수면 모든 point를 `(192, 192, 192)`로 표시한다.
- 결과 color는 contiguous `uint8 (N, 3)` grayscale이다.
- viser `precision="float32"`를 사용한다.
- `point_size`는 adapter option을 사용한다.

빈 `(0, 4)` point array는 valid candidate다. 이전 point scene이 있고 authoritative snapshot이
완전한 empty result를 나타낼 때 empty point scene으로 교체한다.

### 10.3 Trajectory

입력 pose는 `float32 (M, 4, 4)`로 검증한다. Translation은 `poses[:, :3, 3]`에서 꺼내
contiguous array로 만든다.

```python
segments = np.stack([positions[:-1], positions[1:]], axis=1)
```

결과 shape은 `(M - 1, 2, 3)`이다. `M < 2`이면 trajectory node를 만들지 않으며 기존 node는
authoritative empty trajectory가 확인된 경우에만 제거한다. Trajectory color는
`(255, 160, 30)`, viser `thickness_units="world"`를 고정한다.

## 11. Failure, shutdown, diagnostics

Refresh failure는 worker를 종료시키지 않는다. Agent, requested generation, queried revision,
exception category를 stderr에 남기고 다음 committed trigger를 기다린다. 대형 NumPy payload
전체나 filesystem 민감 경로를 로그에 출력하지 않는다.

Shutdown 순서는 다음과 같다.

1. closing flag를 설정한다.
2. event subscription을 close한다.
3. worker stop을 요청하고 condition을 깨운다.
4. worker를 join한다.
5. adapter가 만든 scene handles만 제거한다.
6. CLI owner가 runtime을 close한다.
7. CLI owner가 viser server를 stop한다.

Worker join 전에 runtime 또는 server를 파괴하면 안 된다. Late callback은 closing flag를
확인하고 intent를 버린다.

## 12. Resource와 security 정책

- High-frequency progress event마다 point cloud를 WebSocket으로 전송하지 않는다.
- Point count 제어는 별도 sampling owner를 만들지 않고
  `preview_voxel_size_m`을 Runtime query에 전달한다.
- Viser 전송에 필요한 contiguous XYZ와 RGB copy는 candidate 하나만 유지한다.
- Agent별 과거 revision payload를 누적 보관하지 않는다.
- Scene object는 agent당 point node 하나와 trajectory node 하나로 batching한다.
- 기본 bind는 `127.0.0.1`이며 `0.0.0.0` 공개는 사용자의 명시적 `--host` 입력이 있어야 한다.
- V1은 authentication/TLS를 제공한다고 주장하지 않으며 public network 노출을 권장하지
  않는다.

## 13. Test 명세

### 13.1 Conversion unit tests

- `(N, 4)` points가 contiguous `(N, 3)` XYZ와 `(N, 3)` RGB로 변환된다.
- constant intensity가 고정 gray color를 만든다.
- empty points가 정상 처리된다.
- 잘못된 rank/shape/dtype과 non-finite XYZ가 fail-closed한다.
- `(M, 4, 4)` poses가 `(M - 1, 2, 3)` segment로 변환된다.
- zero/one-pose trajectory가 안전하게 처리된다.
- agent-key encoding이 deterministic하고 collision-free다.

### 13.2 Adapter lifecycle tests

- `start()`가 initial agent refresh를 한 번 요청한다.
- committed event burst가 agent별 latest request 하나로 합쳐진다.
- progress event는 point refresh를 만들지 않는다.
- stale revision과 superseded generation이 scene을 교체하지 못한다.
- query/conversion failure가 기존 scene을 제거하지 않는다.
- 성공 candidate만 atomic replacement와 displayed revision 갱신을 수행한다.
- authoritative agent removal만 owned scene을 제거한다.
- duplicate `start()`는 실패하고 repeated `close()`는 안전하다.
- close가 subscription 해제와 worker join을 완료한다.

### 13.3 CLI/package tests

- invalid option과 invalid voxel/port가 exit `2`다.
- runtime open, server bind, pipeline failure가 exit `1`이다.
- 기본 host가 loopback이다.
- source/PYTHONPATH 없이 fresh CPython 3.10 venv에서 두 wheel만으로 import된다.
- application wheel에 native OpenLMM DSO가 중복 포함되지 않는다.
- application source가 `open_lmm._native` 또는 core private path를 참조하지 않는다.
- core와 binding dependency metadata에 viser가 추가되지 않는다.
- one-agent fixture에서 point와 trajectory scene이 생성되는 installed-wheel smoke를 통과한다.

Browser/WebGL pixel golden은 v1 required gate로 두지 않는다. Viser server API에 대한 fake
scene unit test와 실제 server bootstrap smoke를 required gate로 사용한다.

## 14. 구현 순서

1. 별도 P3 implementation spec과 owner cutover를 완료한다.
2. `applications/python/viser` package skeleton과 dependency policy를 만든다.
3. pure NumPy conversion과 unit tests를 구현한다.
4. bounded refresh worker와 fake scene/runtime lifecycle tests를 구현한다.
5. Viser scene commit과 CLI lifecycle을 연결한다.
6. installed-wheel/package/forbidden-edge tests를 추가한다.
7. README, release policy, third-party notices를 작성한다.
8. fresh environment smoke와 architecture regression을 통과시킨다.

## 15. 완료 조건

- `open-lmm-viser CONFIG_DIR`가 pipeline을 실행하고 browser에서 agent별 committed point
  cloud와 trajectory를 표시한다.
- Stage/job commit 뒤 stale scene overwrite 없이 자동 갱신된다.
- 실패한 candidate가 기존 visible presentation을 지우지 않는다.
- Adapter의 pending work와 retained payload가 agent 수와 최신 candidate 수로 bounded다.
- `open-lmm` binding wheel의 public API, dependency, runtime closure가 변하지 않는다.
- Core에 application reverse dependency가 생기지 않는다.
- GSAT, direct algorithm binding, runtime control UI가 구현에 섞이지 않는다.
- Required unit, lifecycle, package, installed-wheel smoke가 모두 통과한다.

## 16. 외부 API 근거

- Viser repository: <https://github.com/viser-project/viser>
- Viser Python/package compatibility:
  <https://raw.githubusercontent.com/viser-project/viser/main/pyproject.toml>
- Point-cloud example: <https://viser.studio/main/examples/scene/point_clouds/>
- Line example: <https://viser.studio/main/examples/scene/lines/>
- Server API: <https://viser.studio/main/api/core/server/>
- Performance guidance: <https://viser.studio/main/performance_tips/>

## 17. 구현 및 검증 결과

- P3 owner cutover 뒤 SDK wheel은 `open-lmm-experiment` entry point를 더 이상 소유하지
  않으며, Experiment/Viser application wheel이 각 command를 독립 소유한다.
- Viser adapter는 단일 worker, agent별 latest-generation slot, displayed revision guard를
  사용하며 event callback에서는 refresh intent만 기록한다.
- conversion, lifecycle, stale/superseded failure, CLI exit, forbidden dependency, wheel
  content test를 추가했다.
- fresh CPython 3.10 환경에서 `open-lmm==3.0.0`, `open-lmm-experiment==3.0.0`,
  `open-lmm-viser==3.0.0`, `viser==1.1.0` 설치와 실제 server/runtime smoke를 확인했다.
- `open_lmm/core`, public C++ header, plugin ABI, native binding source 및 C++ CLI는 변경하지
  않았다.
