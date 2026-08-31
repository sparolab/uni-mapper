# OpenLMM Application / Binding 재구성 구현 로드맵

- 상태: P0–P2·P4·P5 완료, P3 유예, P6 `SKIPPED / NOT REQUIRED`,
  P7 read-only 및 P7-C1 `IMPLEMENTED / GO`, P7-C2·C3 `PLANNED`
- 기준일: 2026-08-28 UTC
- 상위 제안: [application_adapter_reorganization.md](application_adapter_reorganization.md)
- 적용 원칙: architecture boundary를 먼저 증명하고 물리적 이동은 나중에 수행한다.

## 1. 최종 결정

전체 변경은 한 번의 directory move로 수행하지 않는다. 다음 순서를 따른다.

```text
P0 public-boundary preflight
  ↓
P1 C++ CLI application 추출
  ↓
P2 Python binding 추출
  ├── P3 Python executable application 추출 — DEFERRED/OPTIONAL (v3에서는 생략)
  ↓
P4 standalone GUI package 추출
  ↓
P5 core build/install ownership 정리
  ↓
P6 ROS relocation — SKIPPED / NOT REQUIRED (`ros/` 현위치 유지)
  ↓
P7 ROS GUI 제거 + read-only RViz visualization — IMPLEMENTED / GO
  ↓
P7-C1 ROS late-cancel + RViz Control Panel — IMPLEMENTED / GO
  ↓
P7-C2 Interactive Alignment — PLANNED
  ↓
P7-C3 Transactional Config Form — PLANNED
```

위 application phase와 별도로 production hardening은 다음 순서로 진행한다.

```text
H1 thread safety
  → RuntimeClient cleanup ownership
  → ROS early-cancel
  → algorithm plugin generation validation — IMPLEMENTED / GO
```

최종 목표 구조는 다음과 같다.

```text
open-lmm/
├── open_lmm/                       # C++ core/runtime/public contracts
├── bindings/
│   └── python/                     # Python SDK + pybind11 native binding
├── applications/
│   ├── cli/                        # open_lmm_batch
│   └── gui/                        # standalone GUI
├── ros/                            # ROS leaf adapter의 최종 유지 위치
├── distribution/                   # private artifact composer/verifier
└── scripts/
```

## 2. 고정된 dependency 방향

```text
applications/cli ────────────────┐
applications/gui ────────────────┤
ros/ ────────────────────────────┤──> installed open_lmm::client
bindings/python/native ──────────┘

open_lmm core ────────X──> applications 또는 bindings
```

v3의 `open-lmm-experiment` console entry는 `bindings/python` wheel이 소유한다. 향후 P3
재개 조건이 충족되면 `applications/python/experiment`가 installed OpenLMM Python SDK를
소비하는 선택적 leaf로 추가될 수 있으며, binding이 application을 역으로 의존하지 않는
규칙은 그대로 유지한다.

각 application은 process entry, 사용자 입력 변환, presentation, protocol adaptation과
자기 artifact packaging만 소유한다. runtime state, revision, lifecycle, transaction,
cancellation, recovery health, resource admission의 authority를 소유하지 않는다.

## 3. 분류 규칙

### Binding

다른 Python 코드가 `import open_lmm`으로 재사용하는 API와 그 구현이다.

- pybind11 native module
- Python Runtime/Job/model/error API
- NumPy conversion과 ownership
- 재사용 가능한 `open_lmm.experiments` library
- library 내부 subprocess worker

### Application

사용자가 직접 실행하고 자체 exit code, config, signal, deployment lifecycle을 갖는
프로그램이다.

- `open_lmm_batch`
- standalone GUI
- ROS node/component
- `open-lmm-experiment` console program. 다만 v3에서는 별도 artifact로 분리하지 않고
  Python binding wheel이 함께 제공한다.

library 구현을 위해 내부적으로 실행되는 private worker는 user-facing application으로
분류하지 않는다.

## 4. 단계별 문서

| Phase | 문서 | 핵심 결과 |
|---|---|---|
| P0 | [01_boundary_preflight.md](01_boundary_preflight.md) · [구현명세](01_boundary_preflight_implementation_spec.md) · [결과](results/01_boundary_preflight_result.md) | 완료 — private adapter dependency 제거와 독립 build proof |
| P1 | [02_cli_application_extraction.md](02_cli_application_extraction.md) · [구현명세](02_cli_application_extraction_implementation_spec.md) · [결과](results/02_cli_application_extraction_result.md) | 완료 — CLI를 installed client consumer로 추출 |
| P2 | [03_python_binding_extraction.md](03_python_binding_extraction.md) · [구현명세](03_python_binding_extraction_implementation_spec.md) | SDK/wheel source를 `bindings/python`으로 추출 |
| P3 | [04_python_application_extraction.md](04_python_application_extraction.md) | **DEFERRED/OPTIONAL** — v3에서는 `open-lmm-experiment`를 binding wheel에 유지 |
| P4 | [05_gui_application_extraction.md](05_gui_application_extraction.md) | GUI owner/ABI/loader 결정 후 standalone package화 |
| P5 | [06_core_package_cleanup.md](06_core_package_cleanup.md) | core의 adapter build/install ownership 제거 |
| P6 | [07_ros_relocation_final.md](07_ros_relocation_final.md) | **SKIPPED / NOT REQUIRED** — root `ros/`를 최종 위치로 유지 |
| P7 | [08_deferred_ros_gui_rviz.md](08_deferred_ros_gui_rviz.md) · [구현명세](08_ros_rviz_visualization_implementation_spec.md) · [결과](results/08_ros_rviz_visualization_result.md) | **IMPLEMENTED / GO** — ROS GUI composition 제거와 read-only RViz Map/Path/Loop bridge |
| P7-C1 | [구현명세](10_rviz_control_panel_implementation_spec.md) · [결과](results/10_rviz_control_panel_result.md) | **IMPLEMENTED / GO** — committed-success late-cancel semantics와 ROS-only RViz Control Panel |
| P7-C2 | [구현명세](11_ros_interactive_alignment_implementation_spec.md) | **PLANNED** — lease 기반 alignment review와 InteractiveMarker |
| P7-C3 | [구현명세](12_ros_transactional_config_implementation_spec.md) | **PLANNED** — expected-revision 기반 주요 config form |
| DX-1 | [09_core_gui_developer_entrypoint_implementation_spec.md](09_core_gui_developer_entrypoint_implementation_spec.md) | **GO** — `make core-build`, `make cli`, `make gui` C++ 개발 진입점; P7/Goal 09와 독립 |
| DX-2 | [Python developer entrypoint](../../bindings/python/README.md#local-wheel-build) | **GO** — 전용 wheel-profile core와 venv를 보존하는 `make python*` 개발 진입점 |
| H1 | [13_production_hardening_implementation_spec.md](13_production_hardening_implementation_spec.md) · [결과](results/13_production_hardening_result.md) | **IMPLEMENTED / GO** — thread rollback, client retirement, ROS early-cancel, stale plugin generation 차단 |
| DX-3 | [14_python_viser_interactive_alignment_implementation_spec.md](14_python_viser_interactive_alignment_implementation_spec.md) · [결과](results/14_python_viser_interactive_alignment_result.md) | **IMPLEMENTED / GO** — Python alignment API와 exact-job Viser review, ordered agent 색상 |

## 5. 전역 admission rule

각 phase는 다음 조건을 만족해야 다음 phase로 진행한다.

1. 실제 dependency 또는 package ownership 문제를 명시한다.
2. public API/ABI, executable, package component 변화가 있으면 compatibility review를
   먼저 승인한다.
3. 기존 runtime owner와 transaction/presentation/resource invariant를 변경하지 않는다.
4. source-free installed-prefix consumer를 추가한다.
5. 기존 behavior parity와 rollback 경계를 문서화한다.
6. phase를 독립 commit/PR로 유지한다.
7. compiler, package, ROS, sanitizer, architecture policy gate를 통과한다.

## 6. 절대 섞지 않는 변경

- ROS late-cancel 수정과 ROS directory relocation
- ROS GUI 제거와 ROS directory relocation
- RViz bridge 추가와 ROS directory relocation
- GUI standalone 추출과 portable plugin ABI redesign
- Python binding 이동과 Python API breaking change
- core ownership 정리와 runtime state/lifecycle refactor
- directory move와 알고리즘/성능 최적화

## 7. 호환성 원칙

- `open_lmm_batch` executable 이름, 인자, exit code, 설치 경로는 유지한다.
- Python import root `open_lmm`과 public API golden은 유지한다.
- v3에서는 `open-lmm-experiment` console entry와 executable behavior를 Python binding
  wheel이 계속 소유한다. P3 유예는 P4/P5 진입을 막지 않는다.
- ROS package 이름, action/service/message type과 node/component 이름을 유지한다.
- `open_lmm::gui` 제거는 v3에서 즉시 수행하지 않는다. forwarding/deprecation
  또는 major-version 계획 없이는 P4/P5를 완료 처리하지 않는다.
- plugin ABI v1의 same-toolchain 정책을 확대하지 않는다.

## 8. 공통 검증 matrix

각 phase의 targeted test 외에 다음을 유지한다.

```text
clean GCC 12 GUI OFF
clean GCC 12 GUI ON
clean GCC 13 GUI OFF
clean Clang 15 GUI OFF
full CTest
ROS CTest
source-free package consumers
Python wheel install/import/E2E
ASan+UBSan applicable manifest
TSan applicable manifest
architecture/release policy
git diff --check
```

## 9. Rollback 원칙

- 한 phase 실패 시 그 phase만 revert할 수 있어야 한다.
- old/new target을 동시에 장기간 canonical owner로 두지 않는다.
- compatibility forwarding target은 명시적으로 deprecated이며 runtime authority를
  추가하지 않는다.
- 새 artifact가 source-free consumer를 통과하기 전 기존 install owner를 삭제하지 않는다.
- relocation commit은 semantic modification과 분리한다.

## 10. 전체 완료 조건

- core source/build/install이 application/binding source를 소유하지 않는다.
- 모든 leaf는 installed public surface만 소비한다.
- public/private forbidden edge가 repository-wide policy로 보호된다.
- CLI, Python, GUI, ROS의 기존 public behavior가 유지된다.
- P3는 전체 완료의 필수 조건이 아니며, 유예 상태에서는 Python binding artifact가
  `open-lmm-experiment`의 유일한 file/entry-point owner다.
- ROS adapter는 root `ros/`에 유지한다. P6 relocation 생략은 architecture 또는 package
  completeness를 후퇴시키지 않는다.
- P7 제품 결정 `REMOVE_ROS_GUI + RVIZ_READ_ONLY`와 P7-C1 ROS-only Control Panel은
  완료됐다. Interactive alignment와 transactional config form은 각각 P7-C2와 C3로
  유지하며 실제 GPU/driver lifecycle은 Goal 13 lane이다.
