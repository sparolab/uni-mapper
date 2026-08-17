# OpenLMM 문서 안내

이 디렉터리는 현재 `feat/refactor` 구현을 기준으로 유지하는 문서의 진입점이다.
기준일은 2026-08-15이며, GUI 필수 범위 완료 커밋은 `72990a7`이다.

## 현재 문서

| 문서 | 용도 |
|---|---|
| [architecture.md](architecture.md) | 현재 코드 구조와 실행 데이터 흐름 |
| [implementation-status.md](implementation-status.md) | 설계 항목별 구현·부분 구현·미구현 상태 |
| [roadmap.md](roadmap.md) | 남은 작업의 필수도와 권장 순서 |
| [remaining-work.md](remaining-work.md) | 현재 잔여 구현의 우선순위, 범위와 완료 기준 |
| [verification.md](verification.md) | 빌드, 테스트, 회귀 검증 기준 |
| [baseline.md](baseline.md) | test1/test2 기준 산출물과 성능 기준 |
| [specs/tracy-profiling.md](specs/tracy-profiling.md) | Tracy 선택 계측 구현 명세 |
| [specs/parallel-processing.md](specs/parallel-processing.md) | DataLoad 및 agent 병렬 처리 구현 명세 |
| [specs/gui-stage-pipeline.md](specs/gui-stage-pipeline.md) | GUI 모듈 선택 실행을 위한 Stage/Artifact 명세 |
| [specs/python-wheel-bindings.md](specs/python-wheel-bindings.md) | Pimpl SDK·nanobind 기반 Python API와 binary wheel 배포 명세 |
| [specs/compiler-architecture-hardening.md](specs/compiler-architecture-hardening.md) | 반복되는 GCC ICE 방지를 위한 컴파일 경계·헤더 의존성·CMake 개선 명세 |
| [gui-remaining-work.md](gui-remaining-work.md) | GUI 필수 완료 범위와 선택 후속 작업 |

## 문서 관리 원칙

- 현재 코드에서 확인할 수 있는 사실과 향후 계획을 구분한다.
- 구현 상태는 `완료`, `부분 완료`, `미구현` 세 단계로 표시한다.
- 미구현 항목은 다시 `필수`와 `선택`으로 구분한다.
- 완료 판정은 파일 존재가 아니라 빌드, 테스트, 산출물 비교까지 포함한다.
- 구현이 변경되면 코드와 같은 커밋에서 관련 문서를 갱신한다.

## Legacy 문서

이전 설계안, 분석 보고서, 성능 조사 및 참조 코드는 [legacy](legacy/)에 보관한다.
Legacy 문서는 결정 배경과 과거 분석을 확인하기 위한 자료이며 현재 구현 상태의
기준으로 사용하지 않는다.

완료된 Iridescence adapter 구현 명세는
[legacy/iridescence-gui-adapter.md](legacy/iridescence-gui-adapter.md)에 보관한다.
