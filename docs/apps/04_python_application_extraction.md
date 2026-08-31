# Phase 3 — Python Executable Application Extraction

- 상태: **REOPENED / IMPLEMENTED**
- 재개 근거: 두 번째 Python application인 `open-lmm-viser`와 별도 optional dependency가
  추가되어 아래 재개 조건 두 개를 충족했다.
- 현재 결정: `open-lmm-experiment` console owner는
  `applications/python/experiment`의 독립 wheel이다.

## 0. 유예 결정과 재개

현재 `open-lmm-experiment`는 SDK와 같은 release family, dependency set, 설치 lifecycle을
사용하는 작은 console entry다. 이를 지금 별도 wheel로 분리하면 독립 artifact 두 개의
version compatibility, console-script forwarding/deprecation, 설치 UX와 SBOM/release
ownership이 늘어나지만 당시에는 확인된 제품 요구가 없어 유예했다. 이후 Viser
application 요구가 확인되어 2026-08-28에 P3를 재개하고 owner cutover를 수행했다.

P3는 다음 중 하나가 실제 요구로 확인될 때만 재개한다.

- Python user-facing application이 둘 이상 생긴다.
- CLI에 SDK와 다른 대형 또는 optional dependency가 추가된다.
- SDK와 CLI의 release cadence를 분리해야 한다.
- SDK-only 설치 요구가 제품/사용자 evidence로 확인된다.
- CLI에 별도 보안, 권한 또는 배포 정책이 필요하다.

재개 후 argument parser, exit-code mapping, deployment entry point와
`open-lmm-experiment` console script의 canonical owner는
`applications/python/experiment` 하나다. Reusable experiment model/runner와 public
validation entry point는 `bindings/python`에 남는다.

## 1. 재개 시 목적

사용자가 직접 실행하는 Python program을 reusable binding에서 분리한다. 현재 대상은
`open-lmm-experiment` console application이다.

## 2. 재개 시 분류 결정

다음은 `bindings/python`에 남는다.

- `open_lmm.experiments.Experiment` public library
- manifest/planner/metric/export/model
- library가 trial isolation을 위해 사용하는 private worker protocol

다음은 `applications/python/experiment`으로 이동한다.

- argument parser
- user-facing command/usage
- process exit code mapping
- deployment/package entry point

## 3. 재개 시 목표 구조

```text
applications/python/experiment/
├── pyproject.toml
├── package/open_lmm_experiment/
│   ├── __init__.py
│   ├── __main__.py
│   └── cli.py
├── test/
└── README.md
```

application distribution은 installed `open-lmm` Python SDK에 의존한다.
`open_lmm._native`를 직접 import하지 않는다.

## 4. 재개 시 구현 작업

### P3.1 Thin CLI

- 기존 parser와 `validate`/`run` subcommand를 보존한다.
- 실제 workflow는 `open_lmm.experiments` public API에 위임한다.
- manifest validation이나 runtime lifecycle을 재구현하지 않는다.

### P3.2 Exit contract

기존 의미를 고정한다.

| Exit | 의미 |
|---:|---|
| 0 | validation 또는 required experiment 성공 |
| 1 | experiment result failure |
| 2 | usage/input/schema error |
| 77 | dataset/config asset unavailable |

### P3.3 Package/version

- application package와 SDK release-family compatibility를 명시한다.
- official release에서는 함께 검증된 exact artifact set을 사용한다.
- application wheel이 core native DSO를 다시 bundle하지 않는다.

### P3.4 Transition

- 일정 기간 binding wheel의 old console entry가 새 application으로 forwarding할 수 있다.
- 두 distribution이 같은 console script를 동시에 소유하지 않는다.
- forwarding 제거 시 release note/deprecation policy를 따른다.

## 5. 재개 시 검증

- SDK wheel만 설치한 venv에서 library import PASS
- SDK + application wheel을 설치한 fresh venv에서 command PASS
- source/PYTHONPATH 없이 validate/run
- missing dataset exit 77
- invalid manifest exit 2
- failed experiment exit 1
- successful experiment exit 0
- output/evidence hash가 기존 CLI와 동등

## 6. 재개 시 완료 조건

- user-facing console owner가 application package 하나뿐이다.
- reusable experiment logic은 binding library 하나뿐이다.
- application은 public Python API만 소비한다.
- worker/runtime state owner가 중복되지 않는다.
