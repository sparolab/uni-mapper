# Goal 03 구현 명세서: 실제 데이터셋 E2E / 재현 가능한 Replay

상태: **COMPLETE FOR OWNER-CONTROLLED INTERNAL REPLAY**
작성일: 2026-08-20 (완료 증거 갱신: 2026-08-25)
기준 브랜치/커밋: `develop` / verified source `b168b3db89096ca7010afc15707abb223bcd0c71`
상위 목표: `docs/pose_freeze_goals/open_lmm_post_freeze_goals/03_real_dataset_e2e_replay_goal.md`

2026-08-25 완료 기준:

- v1 case/report/baseline 계약, SHA-256/input-lock preflight와 fail-closed
  comparator가 구현되었다.
- 공개 `RuntimeClient`만 사용하는 case runner와 정상/commit 전 실패 L4
  synthetic-representative replay가 구현되었다.
- 지원 fetch/run/candidate-record/CI 스크립트가 추가되었다.
- self-collected `test1`/`test2`를 owner-controlled internal dataset으로
  승인하고 source/file/archive SHA-256 lock을 고정했다.
- deterministic generator로 tiny/small/representative/failure tier를 만들고
  실제 공개 `RuntimeClient` workflow로 모두 검증했다.
- tiny를 clean source에서 5회 실행해 exact/tolerance/range를 분리한
  `tiny-v1` baseline을 생성하고 5개 report 모두 comparator PASS를 확인했다.
- internal push/manual tiny+small+failure와 scheduled representative workflow를
  추가했다. 실제 repository secret/variable과 immutable image 등록은 운영자가
  수행하는 외부 configuration이며, 누락 시 workflow는 fail closed한다.
- 공개 runtime failure event가 agent context를 제공하지 않는 경우 v1 report는
  `agent: null`을 보존한다. manifest의 failure `agent`는 공개 receipt/event가
  제공할 때만 선언하고 비교하며, progress event로 추정하지 않는다.

## 1. 목적과 완료 정의

이 작업은 합성 단위 테스트를 넘어, 고정된 입력 데이터와 설정으로 OpenLMM의 실제 실행 경로를 반복 실행하고 결과를 자동 판정하는 replay 체계를 만든다. 정상 케이스는 다음 전체 흐름을 검증한다.

`Open → DataLoad → Alignment → MapUpdate → Save → Snapshot/Artifacts → (선택적 Config Change/Rerun) → Close`

실패 케이스는 일부 작업이 진행된 뒤 실패하더라도 commit 이전 상태가 보존되고, 부분 산출물이 authoritative state로 승격되지 않으며, 런타임을 정상적으로 닫을 수 있음을 검증한다.

Goal 03은 다음 조건을 모두 만족할 때 완료된다.

1. 소유자가 명시적으로 승인한 배포 범위에서 사용할 정상 baseline이 최소 1개 존재한다.
2. commit 전 실패와 rollback/부분-stage 의미를 검증하는 failure replay가 최소 1개 존재한다.
3. 입력, 설정, baseline, 실행기 버전을 잠근 재현 명령이 문서화된다.
4. tiny/small replay의 trusted main/manual CI와 representative 정기 실행 정책이 존재한다.
5. exact 비교 대상과 tolerance/range 비교 대상이 manifest에 명시된다.
6. baseline 갱신은 자동 덮어쓰기가 아니라 검토 가능한 별도 절차로만 수행된다.

## 1.1 완료 증거 요약

| 항목 | 결과 |
|---|---|
| Canonical source | `/root/dataset_root/example/test1`, `test2`; owner declaration과 unknown acquisition fields를 명시적으로 기록 |
| Tiny | 40 + 40 frames, archive `c8135600db68381bff5b5f3ba70b3aaac928f450bf8a9eda30a2e852c324b4b7` |
| Small | 80 + 80 frames, archive `95d1f8c0788e768353b7f43ff0a04b06f8bb4780aa5c2c7ccd72a7492c8b27b8` |
| Representative | 310 + 358 frames, archive `606622af3daea27ab09cc9cc2ec5c63f02132855b30a430b3b13856e8ce9d611` |
| Failure | tiny의 `test2` frame 120 PCD를 64 bytes로 truncate, archive `6e43080817a4cd6adc57f9ae1719efb71a739ad9452d020724a8fd17026e1b16` |
| Tiny calibration | 5/5 clean reports PASS; exact metadata/poses + numeric map point/bounds tolerances |
| Tiny baseline | `open_lmm/test/replay/baselines/tiny-v1.json`, SHA-256 `cc230fba81ad3dc2fc2652b4ceb634ff690ee197f42223e424bcaacecf484e1a` |
| Small rerun | DataLoad→Alignment→MapUpdate→Save→ApplyConfig→same four stages, revision 1→10, PASS |
| Representative | full two-agent workload, revision 1→5, final ready/Close PASS |
| Failure semantics | DataLoad `io_error`, revision 1→1, no partial Save/map output, Close PASS |
| CI policy | `.github/workflows/replay.yml`: internal push/manual tiny+small+failure, scheduled/manual representative, immutable image/archive checks |

The dataset is intentionally `LicenseRef-OpenLMM-Internal` and
`redistributable=false`. This completes the owner-controlled internal replay
scope; it does not authorize public fork-PR download or public dataset release.
The workflow therefore runs on trusted push/manual/schedule events and uses
repository secrets for immutable private artifact URLs.

## 1.2 Reproducible local commands

Tiny baseline verification:

```bash
scripts/replay/run_replay_case.sh \
  --runner /root/workspace/build/open_lmm/test/open_lmm_replay_runner \
  --compare /root/workspace/build/open_lmm/test/open_lmm_replay_compare \
  --case open_lmm/test/replay/manifests/tiny-v1.json \
  --data-root /root/dataset_root/openlmm-replay-artifacts/tiny-v1/data \
  --config-root open_lmm/test/replay/configs/test1-test2-v1 \
  --output-root /tmp/openlmm-tiny-output \
  --evidence-root /tmp/openlmm-tiny-evidence \
  --container-digest sha256:aabcc53791995ce4ddf9606f9710cbffa730bb555646f01b61844c7c6724eb6c \
  --baseline open_lmm/test/replay/baselines/tiny-v1.json
```

The shown digest is the owner-controlled immutable OCI manifest published as
`hwan0806/open-lmm:calibration`. CI must use this digest-qualified image rather
than the mutable tag through `OPEN_LMM_REPLAY_IMAGE`.

## 1.3 CI configuration contract

Required repository settings are:

```text
vars.OPEN_LMM_REPLAY_IMAGE = image@sha256:<immutable OCI manifest digest>
secrets.OPEN_LMM_REPLAY_TINY_URL
vars.OPEN_LMM_REPLAY_TINY_SHA256
secrets.OPEN_LMM_REPLAY_SMALL_URL
vars.OPEN_LMM_REPLAY_SMALL_SHA256
secrets.OPEN_LMM_REPLAY_FAILURE_URL
vars.OPEN_LMM_REPLAY_FAILURE_SHA256
secrets.OPEN_LMM_REPLAY_REPRESENTATIVE_URL
vars.OPEN_LMM_REPLAY_REPRESENTATIVE_SHA256
```

Missing, tag-only, or hash-mismatched inputs fail before replay. CI never
generates or widens a baseline.

## 2. 현재 자산과 확인된 공백

### 2.1 재사용할 자산

| 자산 | 현재 기능 | 재사용 방향 |
|---|---|---|
| `open_lmm_replay_verify` | `test1`/`test2` DataLoad, Alignment, LoopDetect, Optimize 실행과 revision/artifact/edge/pose 일부 검증 | 공개 `RuntimeClient` 기반 범용 case runner로 대체 |
| `open_lmm_artifact_compare` | pose 최대 이동/회전 오차, 양방향 PCD 최근접점 RMS/최대값, agent manifest 매핑 | versioned replay report/baseline 비교기로 확장 |
| `agent_manifest.json` v1 | agent ID와 파일 symbol 매핑 | Save 산출물 검색의 canonical mapping으로 유지 |
| `RuntimeClient` | Open, Submit, Wait, Snapshot, Visualization, ApplyConfig, SubscribeEvents, Close 제공 | replay runner가 사용하는 유일한 런타임 진입점 |
| `OpenLmmTest.cmake` | L1–L6와 lane 메타데이터 지원 | replay를 L4 workflow test로 등록하도록 확장 |
| compiler/sanitizer/policy CI | 7개 안정된 required check | 이름을 변경하지 않고 별도 replay job 추가 |

### 2.2 착수 시 기존 도구의 한계와 현재 처리

- 기존 `open_lmm_replay_verify`의 agent 이름·실행 순서 하드코딩과 private
  `MapServer` 진입은 `open_lmm_replay_runner`로 대체했다. 신규 runner는 public
  `RuntimeClient`만 사용한다.
- 기존 verifier에 없던 MapUpdate, Save, Snapshot, optional config rerun, Close와
  manifest-driven 정상/실패 판정을 신규 runner에 구현했다.
- `open_lmm_artifact_compare`의 제한된 비교는 closed report/baseline schema,
  exact/tolerance/range 비교와 map bounds/point 통계를 지원하는
  `open_lmm_replay_compare`로 대체했다.
- contract/validator/comparator/subset builder는 L2, public façade runner E2E는 L4
  CTest로 등록했다.
- 현재 runtime input fingerprint는 경로, 파일 크기, mtime 등에 기반한 런타임 캐시 식별자다. 데이터 공급 무결성을 증명하는 content digest로 사용하면 안 된다.
- 저장 artifact의 FNV fingerprint는 진단 정보일 뿐, 컴파일러와 플랫폼을 넘는 numeric golden 판정 기준으로 사용하면 안 된다.

### 2.3 채택 데이터와 사용 범위

소유자가 직접 수집한 `/root/dataset_root/example/test1`과 `test2`를 Goal 03의
canonical source로 채택했다. 소유자는 이 데이터를 OpenLMM 내부 replay와
private CI에서 사용할 권한을 명시적으로 승인했다. 공개 재배포는 승인하지
않았으므로 `LicenseRef-OpenLMM-Internal`, `redistributable=false`로 고정한다.
수집 시각·센서 모델처럼 현재 확인할 수 없는 acquisition field는 추정하지 않고
provenance 문서에 `not recorded`로 남긴다.

| source | 관찰된 규모 | 채택 용도 | 상태 |
|---|---:|---|---|
| `/root/dataset_root/example/test1` | 310 scans/poses | agent A 및 tier source | 내부 사용 승인·SHA-256 lock 완료 |
| `/root/dataset_root/example/test2` | 358 scans/poses | agent B 및 tier source | 내부 사용 승인·SHA-256 lock 완료 |
| `/root/dataset_root/example/agent1..agent7` | 다중 agent, 수 GB 이상 | 향후 확장 후보 | Goal 03 v1 범위 밖 |

절대 로컬 경로는 case manifest나 baseline에 기록하지 않는다. deterministic
subset bundle에는 source-relative path, file digest, attribution 및 내부 사용
범위만 기록한다. 공개 dataset release가 필요해지는 경우에는 별도 권리·라이선스
검토를 거쳐야 하며, 이는 owner-controlled internal replay 완료 조건이 아니다.

### 2.4 구현 시 기준으로 삼을 코드 위치

- 공개 실행 façade: `open_lmm/include/open_lmm/server/runtime_client.hpp`
- stage/node/artifact vocabulary: `open_lmm/include/open_lmm/common/runtime_contracts.hpp`
- 기존 replay 실행기: `open_lmm/test/tools/replay/replay_verify.cpp`
- 기존 artifact 비교기: `open_lmm/test/tools/replay/artifact_compare.cpp`
- 테스트 등록 helper: `open_lmm/test/cmake/OpenLmmTest.cmake`
- compiler/full CTest 진입점: `scripts/ci/build_and_test.sh`
- 현재 required workflow: `.github/workflows/compiler-matrix.yml`
- artifact/schema compatibility 정책: `RELEASE_POLICY.md`

구현 중 이름이나 위치가 바뀌면 manifest와 CI가 private implementation 경로를 새로 참조하도록 임시 우회하지 말고, 위 public contract와 canonical test owner를 기준으로 한 번에 갱신한다.

## 3. 아키텍처 범위와 불변조건

### 3.1 변경 범위

주 변경은 테스트와 CI 영역에 한정한다.

```text
open_lmm/test/replay/
  schema/
  manifests/
  configs/
  baselines/
  runner/
  compare/
  fixtures/
scripts/replay/
scripts/ci/
.github/workflows/
```

production API가 이미 제공하는 기능만 사용한다. 구현을 위해 runtime state, transaction, lifecycle, presentation owner를 새로 만들거나 변경하지 않는다.

### 3.2 반드시 지킬 경계

- runner는 설치 가능한 공개 `RuntimeClient`와 public contracts만 사용한다.
- private `src/runtime/**`, `RuntimeStateStore`, `RuntimeService`, `MapServer`를 include하거나 링크하지 않는다.
- Snapshot과 event는 관찰값이며 두 번째 runtime state owner가 아니다.
- stage 성공/실패는 로그 문자열이 아니라 receipt와 authoritative Snapshot으로 판정한다.
- numeric output 비교 실패가 런타임 상태를 수정하거나 baseline을 자동 갱신하지 않는다.
- 큰 PCD는 필요할 때만 스트리밍/샘플링하여 비교하고, runner가 전체 포인트클라우드 복사본을 중복 보유하지 않는다.
- public API/ABI와 plugin ABI 변경은 이 Goal의 범위 밖이다. 필요성이 발견되면 별도 architecture proposal로 분리한다.

### 3.3 제외 범위

- 장시간 soak/fault campaign은 Goal 04가 담당한다.
- 성능 및 hard RSS budget은 Goal 05가 담당한다. 이 Goal의 timeout/용량 제한은 CI 운영 안전장치이지 성능 합격선이 아니다.
- GPU, ROS, GUI 전용 E2E와 portable plugin ABI 확대는 각각 해당 후속 Goal이 담당한다.

## 4. 데이터셋 계층

| Tier | 필수 내용 | 목표 규모/시간 | 실행 정책 |
|---|---|---|---|
| `tiny` | 2 agents, 실제 파일 파이프라인, non-empty cross-agent alignment/loop, 전체 정상 흐름 | 40 frames/agent | trusted `develop` push와 manual |
| `small` | tiny보다 긴 구간, Save와 config rerun, 수치 안정성 검증 | 80 frames/agent | trusted `develop` push와 manual |
| `representative` | canonical source 전체 두 session과 운영 stage 조합 | 310 + 358 frames | scheduled/manual nightly |
| `failure` | tiny에서 결정적으로 파생한 손상 PCD | tiny 이하 | trusted `develop` push와 manual |

프레임 수는 목표치일 뿐 고정 선택 규칙이 아니다. tiny 구간은 실제로 non-empty loop/alignment가 재현되는 window를 discovery 실행으로 고른 뒤 manifest에 정확한 frame ID 목록으로 고정한다.

`failure`의 첫 필수 케이스는 정상 tiny bundle을 복사해 만드는 별도 큰 파일이 아니라, manifest transform으로 결정적으로 파생한다. 예를 들어 뒤쪽 scan 한 개의 content digest가 맞지만 decode가 실패하도록 고정된 fixture를 대체하거나 pose/scan cardinality를 불일치시킨다. 기대 결과는 DataLoad commit 전 실패, 이전 runtime revision/artifact 유지, 최종/임시/backup Save 파일 부재, 정상 Close다.

## 5. 데이터 공급과 무결성

### 5.1 저장소에 포함하는 것

- JSON schema
- case manifest와 canonical config
- 파일별 SHA-256 및 bundle SHA-256
- 작은 baseline metadata/report
- 데이터 출처, 라이선스, 재배포 허용 여부
- fetch/run/compare 스크립트

대형 scan, PCD, pose 결과 bundle은 Git 저장소에 직접 넣지 않는다.

### 5.2 외부 artifact 정책

- 데이터와 대형 baseline artifact는 immutable release/object artifact로 배포하고 SHA-256으로 주소화한다.
- CI cache는 다운로드 최적화 수단일 뿐 source of truth가 아니다.
- v1 데이터는 internal-only이므로 tiny/small/failure/representative 모두 owner-approved
  private immutable URL과 repository secret을 사용한다. public fork PR에는 데이터를
  노출하거나 외부 contributor의 secret 접근을 허용하지 않는다.
- 모든 tier는 같은 archive SHA-256, file lock, path/symlink 검증을 통과해야 한다.
- 로컬 override는 `OPEN_LMM_REPLAY_DATA_ROOT`만 허용하고, 파일 digest가 manifest와 다르면 즉시 실패한다.
- scheduled representative job에서 필수 데이터가 없으면 skip으로 녹색 처리하지 않고 preflight 실패로 보고한다. 로컬 개발 실행만 exit 77 `NOT_AVAILABLE`을 허용한다.

### 5.3 provenance 필수 필드

데이터를 baseline으로 승격하려면 최소 다음 항목이 있어야 한다.

- 원 출처 URL 또는 내부 asset 식별자
- 취득일과 원 버전
- SPDX license identifier와 attribution 파일
- 원본 변경 내역: agent/frame 선택, 변환, downsample, anonymization
- 원본 및 배포 bundle SHA-256
- 공개 재배포 가능 여부

archive 생성은 정렬된 path, 고정 mtime/uid/gid, 고정 압축 옵션을 사용한다.

## 6. Versioned case manifest

`replay_case.schema.json` v1은 아래 의미를 갖는다. 실제 schema는 모든 필수 필드, enum, 숫자 범위를 닫힌 형태로 검증하고 unknown key를 거부한다.

```json
{
  "schema_version": 1,
  "case_id": "tiny-public-v1",
  "tier": "tiny",
  "dataset": {
    "bundle_id": "open-lmm-replay-tiny-v1",
    "bundle_sha256": "<sha256>",
    "source": "<stable source identifier>",
    "license": "<SPDX>",
    "redistributable": true,
    "agents": [
      {
        "id": "agent-a",
        "frames": [100, 101, 102],
        "pose_file": {"path": "agent-a/optimized_poses.txt", "sha256": "<sha256>"},
        "scan_index": {"path": "agent-a/scans.sha256", "sha256": "<sha256>"}
      }
    ]
  },
  "config": {
    "root": "configs/tiny/config.json",
    "files": [{"path": "configs/tiny/config.json", "sha256": "<sha256>"}],
    "plugins": [{"kind": "descriptor", "id": "<id>", "capability": "<capability>"}]
  },
  "workflow": [
    {"stage": "DataLoad", "agents": "all"},
    {"stage": "Alignment", "agents": "all"},
    {"stage": "MapUpdate", "agents": "all"},
    {"stage": "Save", "agents": "all"}
  ],
  "expected": {
    "result": "success",
    "baseline_id": "tiny-public-baseline-v1"
  },
  "tolerances": {
    "pose_translation_m": 0.001,
    "pose_rotation_rad": 0.001,
    "map_symmetric_rms_m": 0.02
  }
}
```

Threshold 숫자는 예시다. 실제 값은 9절의 calibration 절차로 결정한다.

manifest에는 절대 경로, 임시 job ID, 실행 시각, 현재 workspace 이름을 넣지 않는다. case와 baseline schema version은 서로 독립적으로 증가시킨다. incompatible schema 변경은 새 major version과 migration/compatibility note가 필요하다.

## 7. 구현 구성요소

### 7.1 `open_lmm_replay_runner`

기존 `open_lmm_replay_verify`를 case-driven 실행기로 대체한다.

책임:

1. manifest/schema와 모든 입력 SHA-256을 preflight한다.
2. data를 read-only로 취급하고 매 실행마다 비어 있는 output/cache 디렉터리를 만든다.
3. `RuntimeClient::Open` 후 event를 구독한다.
4. workflow의 stage를 하나씩 Submit/Wait하고 각 barrier 뒤 Snapshot을 수집한다.
5. agent ordering, revision delta, artifact state, loop/alignment identity를 canonical report로 만든다.
6. Visualization과 Save 산출물에서 pose/map 통계를 계산한다.
7. 선택적 ApplyConfig와 affected-stage rerun을 수행한다.
8. 성공과 실패 모두에서 Close를 시도하고 Close 결과를 report에 기록한다.
9. failure case에서도 RAII cleanup 뒤 유효한 report를 남긴다.

runner는 baseline을 판정하거나 수정하지 않는다. 실행 사실을 `replay_report.schema.json` v1 형식으로 기록하는 역할만 가진다.

CLI 초안:

```text
open_lmm_replay_runner \
  --case <case-manifest.json> \
  --data-root <verified-bundle-root> \
  --output-root <empty-directory> \
  --report <report.json>
```

### 7.2 `open_lmm_replay_compare`

기존 `open_lmm_artifact_compare`를 manifest-driven 비교기로 확장한다.

책임:

- report schema와 baseline schema 검증
- exact field 비교
- tolerance/range metric 계산 및 비교
- agent mapping과 pose index의 완전성 검사
- PCD symmetric nearest-neighbor mean/RMS/p95/max, point-count ratio, AABB/centroid, 선택적 voxel occupancy 비교
- 불일치별 expected/actual/delta/limit을 담은 JSON 및 사람이 읽는 요약 생성
- 입력이 없거나 손상되었을 때 fail closed

CLI에서 개별 positional threshold를 받지 않는다. 모든 threshold는 검토된 case/baseline manifest에서만 읽는다. 임시 실험 override가 필요하면 결과를 `non_baseline_run: true`로 표시하고 CI 합격 판정에는 사용할 수 없게 한다.

### 7.3 이름 전환

두 기존 도구는 설치 public API가 아니다. 신규 case-driven 경로는 다음 이름으로
구현했으며, 기존 진단 도구는 호환·개발 보조 목적으로만 남긴다.

| 기존 | 신규 |
|---|---|
| `open_lmm_replay_verify` | `open_lmm_replay_runner` |
| `open_lmm_artifact_compare` | `open_lmm_replay_compare` |

외부 사용자가 확인되면 한 release 동안 deprecated wrapper target을 제공하되, 중복 구현은 만들지 않는다.

### 7.4 지원 스크립트

- `scripts/replay/fetch_replay_data.sh`: immutable URL 다운로드, archive/bundle/file SHA-256 검증, 안전한 압축 해제
- `scripts/replay/run_replay_case.sh`: 격리된 temp/output/cache 생성, runner와 comparator 실행, JUnit 변환
- `scripts/replay/record_baseline.sh`: candidate report 생성만 수행하며 baseline 파일을 자동 승인하지 않음
- `scripts/ci/run_replay_tests.sh`: required case 목록을 고정하고 missing data를 실패 처리

fetcher는 path traversal, symlink escape, 예상 밖 파일을 거부한다. cleanup은 생성한 정확한 temp 경로에만 수행한다.

## 8. Replay report와 판정 계약

report에는 다음을 기록한다.

- report schema version, case ID, case manifest SHA-256
- dataset/config/plugin lock digest
- Git commit, dirty 여부, compiler/toolchain/container digest
- agent 순서와 선택 frame ID
- stage별 receipt, result/error code, revision before/after, artifact inventory
- authoritative Snapshot에서 얻은 health와 recovery-required 상태
- canonical event type/order와 stage/agent context
- loop/alignment identity와 method
- pose/map 수치 통계
- output-relative artifact path와 content digest(진단용 포함)
- Close 결과

경로는 output root 상대경로로 정규화하고 object key는 정렬한다. timestamp, PID, 임시 경로, raw pointer, scheduling-dependent event interleaving은 baseline exact field에서 제외한다.

### 8.1 Exact와 tolerance 구분

| 분류 | 판정 대상 | 방식 |
|---|---|---|
| 입력 무결성 | dataset/config/baseline bundle | SHA-256 exact |
| canonical metadata | schema/case ID, agent order, frame IDs, plugin IDs/capabilities, artifact key/type/state, 상대 파일명 | exact |
| workflow 의미 | stage 순서, success/failure code, revision delta, committed/recovery health, failure context | exact |
| topology identity | loop/alignment agent pair와 frame identity, method | 안정성 확인 후 exact; 동률 가능 시 허용 집합 명시 |
| pose/trajectory | translation/rotation, ATE/RPE | tolerance/range |
| map geometry | point count ratio, AABB/centroid delta, symmetric NN mean/RMS/p95/max, voxel overlap | tolerance/range |
| 비결정 정보 | timestamp, scheduling order, absolute path, runtime/FNV fingerprint | 진단 전용, 합격 판정 제외 |

Numeric PCD나 pose 파일 전체 byte hash를 golden 결과로 사용하지 않는다. canonical JSON metadata만 stable serializer를 거쳐 exact hash를 추가로 가질 수 있다.

## 9. Baseline 생성과 변경 정책

### 9.1 최초 생성

1. provenance와 data lock 검토를 먼저 통과한다.
2. clean freeze-compatible commit과 고정 container/toolchain에서 candidate를 생성한다.
3. 매 실행 fresh output/cache로 동일 case를 최소 5회 실행한다.
4. tiny는 지원 compiler 조합에서도 반복하여 exact field가 동일한지 확인한다.
5. 관찰된 numeric envelope에 안전계수를 적용하되, 별도의 domain 상한을 넘으면 baseline을 넓히지 않고 원인을 조사한다.
6. baseline PR에 five-run report, 최대 편차, 선택 threshold 근거, 산출물 변화 요약을 첨부한다.

deterministic required case는 우선 thread 수를 1로 고정한다. 병렬 실행 일관성은 별도 case로 만들며, 단일 baseline의 tolerance를 넓혀 비결정성을 숨기지 않는다. alignment/cache는 실행마다 비우거나 case 전용 경로를 사용한다.

### 9.2 변경 승인

- CI는 baseline을 쓰거나 자동 승인하지 않는다.
- baseline ID, dataset lock, threshold 변경은 이유와 before/after 비교 report가 있는 별도 검토 대상이다.
- threshold는 CI 실패를 해소하기 위해 자동 확장할 수 없다.
- algorithm/config의 의도된 변경은 기존 baseline을 덮어쓰지 않고 새 baseline version을 만든다.
- 이전 baseline은 정해진 보존 기간 동안 회귀 비교가 가능해야 한다.

## 10. 정상/실패 workflow 세부 계약

### 10.1 정상 case

각 stage 후 runner는 receipt와 Snapshot을 함께 확인한다.

| 단계 | 필수 확인 |
|---|---|
| Open | configured agents와 초기 health, config/input lock 일치 |
| DataLoad | agent artifact ready, expected revision delta, frame cardinality |
| Alignment | alignment artifact ready, anchor/agent identity와 method |
| MapUpdate | committed map artifact, stale generation 미승격, point/bounds 통계 |
| Save | manifest에 선언한 pose/map 파일 존재, temp/backup 잔여물 부재 |
| Snapshot/Artifacts | ordered agents, revision, health, artifact inventory가 report와 일치 |
| Config/Rerun | expected revision winner, 영향받는 artifact만 교체, 이전 valid presentation 유지 |
| Close | 진행 job/lease 정리와 성공 결과 |

### 10.2 failure case

첫 failure case는 commit 전 DataLoad 실패로 고정한다.

필수 assertion:

- receipt는 manifest에 선언된 error code/severity/stage/agent context와 일치한다.
- authoritative runtime revision은 실패 전 값과 같다.
- 이전 committed artifact와 presentation이 있다면 그대로 유지된다.
- failed candidate artifact가 ready/committed로 보이지 않는다.
- 최종 Save 파일과 `.tmp`/backup 잔여물이 없다.
- Close가 성공하고 동일 process의 후속 case 또는 새 Open을 방해하지 않는다.

추가 failure case로 Save destination conflict, config commit 경쟁, late cancellation을 넣을 수 있지만 기존 runtime invariant 테스트와 assertion을 무의미하게 복제하지 않는다. replay에서는 데이터/파일 경계를 실제로 통과하는 E2E 증거에 집중한다.

## 11. CTest와 CI 연결

### 11.1 CTest 등록

- primary layer: `L4`
- module: `workflows.replay`
- owner: `RuntimeWorkflow`
- synthetic contract/subset/failure lane: `pr`
- internal tiny/small/failure workflow: trusted `develop` push 또는 manual
- internal representative workflow: `schedule` 또는 manual

외부 데이터용 로컬 wrapper는 runner 또는 검증된 data/baseline이 없으면 77
`NOT_AVAILABLE`을 반환할 수 있다. required CI wrapper는 case/data/config/baseline
입력을 모두 요구하며 누락을 실패 처리한다. 저장소 내 synthetic L4 replay는
항상 실행되며 skip을 허용하지 않는다. 외부 dataset CTest를 직접 등록할 때만
`open_lmm_add_test`의 `SKIP_RETURN_CODE` 확장을 검토한다.

schema, comparator boundary, hash tamper, path traversal 검사는 data 없이 실행되는 L1/L2 테스트로 별도 등록한다.

### 11.2 CI job

`.github/workflows/replay.yml`에 다음 안정된 job 이름을 둔다.

- `replay / internal-tiny-small`: `develop` push와 manual 실행에서 tiny, small,
  failure 실행
- `replay / internal-representative-nightly`: schedule와 manual 실행에서
  representative 실행

기존 compiler matrix의 안정된 job 이름은 바꾸지 않는다. internal dataset secret은
fork PR에 제공하지 않으므로 replay는 compiler matrix와 분리된 trusted event에서만
실행한다. repository variable/secret과 immutable image를 등록하고 최초 hosted green
run을 확인한 뒤 required-check 적용 여부를 별도 운영 정책으로 결정한다.

CI는 report, JUnit, comparator diff와 필요한 로그를 업로드한다. 대형 PCD 전체 업로드는 기본 금지하고, 실패 시에만 크기/보존 기간 제한 아래 sampled diagnostic 또는 별도 artifact URL을 제공한다.

## 12. 구현 단계와 체크포인트

### Phase A — 계약과 unit foundation

- replay case/report/baseline JSON schema 추가
- parser/validator와 canonical JSON serializer 추가
- comparator를 library + CLI로 분리
- exact/tolerance 경계, quaternion sign equivalence, missing/extra agent/frame, corrupt PCD, threshold 경계 unit test 추가
- SHA-256 lock, archive safety, path normalization test 추가

완료 체크포인트: 외부 데이터 없이 schema/comparator/fetcher test가 deterministic하게 통과한다.

### Phase B — 공개 façade runner

- 기존 verifier의 semantic assertion을 case-driven collector로 이전
- `RuntimeClient`만 사용하는 runner 구현
- full normal workflow, event/Snapshot collection, Close RAII 구현
- private runtime include/link zero를 architecture policy에 추가

완료 체크포인트: synthetic-representative fixture로 전체 workflow report가 생성되고 architecture policy가 통과한다.

### Phase C — 데이터와 baseline

- 소유자가 승인한 내부 dataset 선정과 사용 범위 기록
- tiny window discovery와 small/representative case 고정
- provenance/attribution/data lock 작성
- 정상 five-run calibration과 failure fixture 생성
- baseline review 자료 생성

완료 체크포인트: **완료.** tiny 정상 baseline과 failure replay가 clean source에서 재현되었다.

### Phase D — CI와 운영

- fetch/run wrappers 및 CTest metadata 연결
- tiny/small required candidate job과 representative schedule 추가
- missing/corrupt data, timeout, artifact upload 동작 검증
- branch protection/release policy 증거 갱신

완료 체크포인트: workflow와 fail-closed/read-only baseline 계약은 구현되었다.
최초 hosted green run은 repository secret/variable과 immutable OCI image를
등록하는 운영 작업이며 구현 결과와 분리해 추적한다.

## 13. 검증 명령 계약

현재 구현의 대표 사용자 흐름은 다음과 같다.

```bash
bash scripts/replay/fetch_replay_data.sh \
  --url "$OPEN_LMM_REPLAY_TINY_URL" \
  --archive-sha256 "$OPEN_LMM_REPLAY_TINY_SHA256" \
  --destination /tmp/open_lmm_replay_data

bash scripts/replay/run_replay_case.sh \
  --runner /root/workspace/build/open_lmm/test/open_lmm_replay_runner \
  --compare /root/workspace/build/open_lmm/test/open_lmm_replay_compare \
  --case open_lmm/test/replay/manifests/tiny-v1.json \
  --data-root /tmp/open_lmm_replay_data \
  --config-root open_lmm/test/replay/configs/test1-test2-v1 \
  --output-root /tmp/open_lmm_replay_output \
  --evidence-root /tmp/open_lmm_replay_evidence \
  --container-digest sha256:<immutable-image-digest> \
  --baseline open_lmm/test/replay/baselines/tiny-v1.json

ctest --test-dir /root/workspace/build/open_lmm --output-on-failure \
  -R '^open_lmm_replay_.*_tests$'

# External/internal CI wrapper: case/data/config/baseline roots are supplied
# through the OPEN_LMM_REPLAY_* environment contract.
bash scripts/ci/run_replay_tests.sh \
  /root/workspace/build/open_lmm sha256:<immutable-image-digest>
```

구현 완료 검증 목록:

- schema/parser/comparator/fetcher unit tests
- tiny 정상 replay 최소 5회 fresh run
- tiny 결과의 compiler 간 exact-field 일치 확인
- small 정상 replay와 config rerun
- failure replay의 revision/artifact/file rollback assertion
- representative scheduled run
- `bash scripts/ci/check_architecture_policy.sh`
- clean compiler build와 full CTest
- `git diff --check`

## 14. 구현 PR 산출물 체크리스트

- [x] owner-controlled internal dataset과 provenance/usage declaration
- [x] versioned case/report/baseline schema
- [x] SHA-256 data lock과 안전한 fetcher
- [x] public `RuntimeClient` 기반 runner
- [x] manifest 기반 exact/tolerance comparator
- [x] tiny/small/representative/failure manifests
- [x] 최소 1개 정상 baseline과 five-run calibration evidence
- [x] 최소 1개 commit 전 failure replay
- [x] CTest L4/lane/owner metadata
- [x] trusted push/manual replay와 scheduled representative workflow
- [x] baseline 변경/보존/승인 절차
- [x] architecture policy와 targeted replay 검증 증거
- [ ] repository secrets/variables 및 immutable OCI image 등록 후 최초 hosted green run

Goal 03의 기술 구현과 owner-controlled internal provenance는 완료되었다. 공개
재배포는 의도적으로 승인 범위에서 제외되며 완료 조건이 아니다. 남은 hosted
green run은 데이터 권리 문제가 아니라 repository 운영 configuration이다.
