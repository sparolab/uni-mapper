# OpenLMM Goal 03 --- Replay Dataset 공식화 가이드

## 1. 목적

Goal 03용 데이터셋을 **공식화**한다는 것은 단순히 테스트용 데이터 폴더
하나를 지정하는 것이 아니다.

목표는 다음 조건을 만족하는 **재현 가능한 regression asset**을 만드는
것이다.

``` text
누가 실행해도
    ↓
동일한 dataset/config를 사용하고
    ↓
동일한 OpenLMM replay workflow를 실행하며
    ↓
versioned baseline과 비교할 수 있음
```

즉 다음이 고정되어야 한다.

-   데이터 출처와 소유권
-   사용할 session/agent와 frame
-   데이터 변환 과정
-   config
-   파일별 SHA-256
-   archive SHA-256
-   replay manifest
-   baseline
-   numeric tolerance
-   CI 실행 방법

Goal 03은 **map-merging 알고리즘의 절대 accuracy 평가**보다 **OpenLMM
전체 pipeline의 regression 검증**이 주목적이다.

------------------------------------------------------------------------

# 2. 가장 먼저 할 일 --- Canonical Source Dataset 선정

직접 취득한 원본 데이터셋 하나를 canonical source로 지정한다.

예:

``` text
dataset_id: openlmm-office-01
source: self-collected
owner: OpenLMM project
acquisition_date: 2026-XX-XX
sensor: <sensor information>
redistribution: permitted | internal-only
```

최소한 다음 질문에 답할 수 있어야 한다.

``` text
이 데이터는 어디서 왔는가?
누가 소유하는가?
언제 취득했는가?
어떤 sensor/config로 취득했는가?
외부 공개가 가능한가?
원본에서 어떤 가공이 이루어졌는가?
```

직접 취득한 데이터라면 provenance/권리 문제를 비교적 쉽게 해결할 수
있다.

------------------------------------------------------------------------

# 3. 원본 전체를 바로 CI에 사용하지 않는다

원본 dataset에서 용도별 tier를 만든다.

권장 구조:

``` text
Original Dataset
      │
      ├── tiny-v1
      │
      ├── small-v1
      │
      └── representative-v1
```

추가로 failure replay를 위한 deterministic failure case를 만든다.

``` text
tiny-v1
   ↓
deterministic corruption
   ↓
failure-v1
```

------------------------------------------------------------------------

# 4. Tiny Dataset

가장 먼저 **tiny-v1 하나를 완전히 공식화하는 것**을 권장한다.

## 권장 조건

``` text
2 map/session
session당 약 20~60 frames
cross-map overlap 존재
실제 alignment/loop 발생
PR CI에서 수분 이내 실행
```

중요한 것은 단순히 원본의 첫 50 frame을 자르는 것이 아니다.

두 session 사이에서 실제로:

``` text
Map A
   ↕ overlap
Map B
```

가 존재하고 OpenLMM이 cross-map alignment/loop를 생성할 수 있는 구간을
선택해야 한다.

### Tiny가 검증해야 할 workflow

``` text
Open
 ↓
DataLoad
 ↓
Alignment
 ↓
MapUpdate
 ↓
Save
 ↓
Snapshot / Artifacts
 ↓
Close
```

------------------------------------------------------------------------

# 5. Small Dataset

tiny보다 긴 실제 workload를 제공한다.

권장 조건:

``` text
2~3 map/session
더 긴 trajectory
더 많은 scan
Save 포함
Config Change / Rerun 포함
numeric regression 관찰 가능
```

용도:

``` text
PR/main dedicated replay
```

tiny가 빠른 correctness regression을 담당한다면 small은 조금 더 현실적인
pipeline regression을 담당한다.

------------------------------------------------------------------------

# 6. Representative Dataset

실제 OpenLMM 사용 환경에 가까운 dataset이다.

권장 조건:

``` text
3~7 map/session
실제 운용과 유사한 overlap 구조
실제 algorithm/plugin 조합
충분한 point 수
실제 map-merging workload
```

용도:

``` text
Nightly / External Replay
```

PR마다 돌릴 필요는 없다.

------------------------------------------------------------------------

# 7. Failure Dataset

새로운 대형 데이터셋을 따로 준비할 필요는 없다.

정상 tiny dataset에서 결정적으로 생성한다.

예:

``` text
tiny-v1
   ↓
PCD 하나 손상
   ↓
failure-corrupt-pcd-v1
```

또는:

``` text
pose count != scan count
```

등을 사용할 수 있다.

검증 목적:

``` text
DataLoad failure
       ↓
commit 이전 상태 유지
       ↓
revision 변화 없음
       ↓
partial artifact 미승격
       ↓
tmp/backup residue 없음
       ↓
Close 성공
```

즉 failure replay는 OpenLMM transaction/rollback semantics가 실제 파일
경계를 통과해도 유지되는지 확인한다.

------------------------------------------------------------------------

# 8. Ground Truth가 필요한가?

## Goal 03에서는 필수가 아니다

Goal 03의 목적은:

> "이 merge 결과가 실제 세계의 정답에 얼마나 가까운가?"

가 아니라:

> "같은 입력에서 OpenLMM이 기존 정상 동작과 비교해 regression 없이
> 동일한 의미의 결과를 생성하는가?"

이다.

따라서 다음 GT는 Goal 03 완료를 위해 필수가 아니다.

``` text
GT merged map
GT session-to-session transform
GT global trajectory
```

최소 입력은 다음 정도면 충분하다.

``` text
각 session의 scan / PCD
        +
각 session의 local pose
        +
session 간 overlap
        +
OpenLMM config
```

------------------------------------------------------------------------

# 9. Goal 03의 PASS/FAIL 기준

최초 정상 실행을 baseline으로 만든다.

그 이후 결과를 다음과 같이 비교한다.

## Exact Comparison

``` text
dataset/config hash
agent ordering
frame selection
stage ordering
success/failure code
revision delta
artifact type/state
plugin ID/capability
```

## Exact 또는 Allowed Set

``` text
alignment identity
loop identity
agent pair
frame correspondence
method
```

동률이나 알고리즘 특성상 복수의 정상 결과가 가능하면 reviewed allowed
set을 사용한다.

## Tolerance / Range

``` text
relative pose
trajectory
map point count
map centroid
AABB
symmetric NN RMS
map geometry
```

## Diagnostic Only

``` text
timestamp
PID
temporary path
scheduler-dependent event ordering
runtime fingerprint
```

------------------------------------------------------------------------

# 10. Accuracy Dataset은 별도 개념

향후 OpenLMM map-merging 알고리즘 자체의 정확도를 평가하려면 GT가
유용하다.

가장 가치가 높은 GT는:

``` text
T_A_B^GT
```

즉 session A와 B 사이의 실제 SE(3) transform이다.

알고리즘 결과:

``` text
T_A_B^est
```

와 비교하면:

``` text
translation error
rotation error
```

를 계산할 수 있다.

더 강한 dataset은 모든 session trajectory가 동일한 global frame GT를
갖는다.

``` text
T_W_A0
T_W_A1
...

T_W_B0
T_W_B1
...
```

이 경우:

-   relative map transform error
-   ATE
-   RPE
-   merged trajectory consistency
-   map geometry

등을 평가할 수 있다.

그러나 이것은 **Goal 03 regression dataset 공식화와 별도 작업**이다.

------------------------------------------------------------------------

# 11. 사용할 Frame을 정확히 고정

예:

``` text
tiny-v1

agent_a:
  frames: 1200~1240

agent_b:
  frames: 830~875
```

보다 더 강하게 exact frame list를 manifest에 기록하는 것이 좋다.

예:

``` json
{
  "agent_a_frames": [1200, 1201, 1202, 1203],
  "agent_b_frames": [830, 831, 832, 833]
}
```

이렇게 하면 원본 dataset에 파일이 추가되더라도 `tiny-v1`의 의미가 바뀌지
않는다.

------------------------------------------------------------------------

# 12. Reproducible Subset Generator 작성

tiny/small dataset을 수동 복사해서 만들지 않는다.

예:

``` text
tools/build_replay_subset.py
```

를 이용해:

``` text
Original Dataset
      +
Subset Manifest
      ↓
tiny-v1
      ↓
tiny-v1.tar.zst
```

가 항상 동일하게 생성되도록 한다.

예시 실행 형태:

``` bash
python tools/build_replay_subset.py \
  --source /data/openlmm-office-01 \
  --manifest tiny-v1.source.json \
  --output tiny-v1/
```

Generator 자체도 versioning한다.

``` text
generator_version: 1
```

가능하면 archive 생성 시 다음도 deterministic하게 고정한다.

-   file ordering
-   mtime
-   uid/gid
-   compression options

------------------------------------------------------------------------

# 13. SHA-256 Input Lock

생성된 모든 중요 입력 파일을 hash로 고정한다.

예:

``` text
agent_a/scans/0001.pcd → SHA-256
agent_a/scans/0002.pcd → SHA-256
agent_a/poses.txt       → SHA-256

agent_b/scans/0001.pcd → SHA-256
agent_b/poses.txt       → SHA-256

config.json             → SHA-256
```

그리고 최종 archive:

``` text
openlmm-replay-tiny-v1.tar.zst
```

에도 SHA-256을 기록한다.

구조:

``` text
Individual File SHA-256
          +
Config SHA-256
          +
Archive SHA-256
          ↓
Reproducible Input Lock
```

파일이 하나라도 달라지면 replay를 fail closed한다.

------------------------------------------------------------------------

# 14. Dataset Manifest 작성

예시:

``` json
{
  "dataset_id": "openlmm-office-tiny-v1",
  "source_dataset": "openlmm-office-01",
  "source": "self-collected",
  "owner": "OpenLMM project",
  "acquisition_date": "2026-07-10",
  "redistribution": "internal-only",

  "agents": [
    {
      "id": "agent-a",
      "frames": [1200, 1201, 1202],
      "pose_file": "agent-a/poses.txt"
    },
    {
      "id": "agent-b",
      "frames": [830, 831, 832],
      "pose_file": "agent-b/poses.txt"
    }
  ],

  "bundle_sha256": "...",
  "generator_version": "1",

  "transformations": [
    "selected frame subset only",
    "no point downsampling"
  ]
}
```

필요하다면 다음도 추가한다.

``` text
sensor
config hash
plugin IDs
original dataset version
archive URL
license/SPDX
attribution
```

GT 필드는 필수가 아니다.

------------------------------------------------------------------------

# 15. Dataset Storage

대형 PCD를 Git repository에 직접 넣는 것은 피한다.

권장:

``` text
Git Repository
    │
    ├── manifest
    ├── config
    ├── baseline
    └── hashes

External Immutable Storage
    │
    └── actual PCD/archive
```

외부 storage 후보:

``` text
Internal artifact server
GitHub Release artifact
Object storage
```

핵심은:

``` text
Immutable URL
      +
SHA-256
```

이다.

Public CI라면 secret 없이 받을 수 있는 공개 tiny/small bundle이
이상적이다.

Private CI라면 immutable private artifact를 사용할 수 있다.

------------------------------------------------------------------------

# 16. OpenLMM Replay 실행 검증

공식 baseline을 만들기 전에 각 tier가 실제로 OpenLMM에서 정상 실행되는지
확인한다.

``` text
Open
 ↓
DataLoad
 ↓
Alignment
 ↓
MapUpdate
 ↓
Save
 ↓
Snapshot
 ↓
Close
```

Tiny에서는 특히 다음을 확인한다.

``` text
cross-map alignment 존재
cross-map loop 존재 또는 의도한 alignment evidence 존재
```

overlap이 있어도 algorithm이 실제 correspondence를 생성하지 않는
구간이라면 tiny 후보를 다시 선정한다.

------------------------------------------------------------------------

# 17. Baseline Calibration

tiny-v1을 clean environment에서 최소 5회 실행한다.

``` text
Run 1
Run 2
Run 3
Run 4
Run 5
```

그 결과를 비교한다.

### 항상 동일한 값

``` text
→ exact baseline
```

### 조금씩 변하는 numeric 결과

``` text
→ tolerance/range baseline
```

예:

``` text
agent order            exact
stage result           exact
revision delta         exact
alignment pair         exact / allowed set

relative transform     tolerance
trajectory             tolerance
map RMS                tolerance
point count            range
AABB                    tolerance
```

Tolerance는 CI failure를 없애기 위해 임의로 크게 잡지 않는다.

실제 5회 variation을 관찰하고 domain적으로 허용 가능한 범위 안에서
결정한다.

------------------------------------------------------------------------

# 18. Baseline Versioning

예:

``` text
dataset:
tiny-v1

baseline:
tiny-baseline-v1
```

Baseline에는 최소 다음 정보를 기록한다.

``` text
dataset hash
config hash
baseline schema version
Git SHA
compiler/toolchain
plugin IDs/capabilities
expected metadata
numeric tolerance
```

이후 OpenLMM 변경 시:

``` text
Current Replay Result
          ↓
tiny-baseline-v1
          ↓
Comparator
          ↓
PASS / FAIL
```

로 판정한다.

------------------------------------------------------------------------

# 19. 권장 Repository 구조

``` text
open_lmm/test/replay/
├── manifests/
│   ├── tiny-v1.json
│   ├── small-v1.json
│   ├── representative-v1.json
│   └── failure-corrupt-pcd-v1.json
│
├── baselines/
│   ├── tiny-baseline-v1.json
│   └── small-baseline-v1.json
│
├── configs/
│   ├── tiny/
│   └── small/
│
├── schema/
│
└── runner/
```

실제 대형 scan/archive는 외부 immutable storage에 둔다.

------------------------------------------------------------------------

# 20. CI 연결

## PR / Main

``` text
replay / tiny-small
    │
    ├── tiny normal
    ├── small normal
    └── failure
```

## Nightly

``` text
replay / representative
```

CI에서 보존할 artifact:

``` text
Replay report
Comparator diff
JUnit
Failure log
Dataset hash
Baseline hash
Git SHA
Compiler/container metadata
```

------------------------------------------------------------------------

# 21. 처음부터 모든 Tier를 완성할 필요는 없음

가장 안전한 진행 순서는:

``` text
tiny-v1 선정
    ↓
tiny-v1 공식화
    ↓
normal baseline
    ↓
failure baseline
    ↓
PR CI 연결
    ↓
small-v1
    ↓
representative-v1
```

즉 **tiny-v1 하나를 먼저 완전히 닫는 것**이 가장 중요하다.

------------------------------------------------------------------------

# 22. 지금 실제로 해야 할 최소 작업

``` text
1. 원본 dataset 하나 선정

2. agent/session 목록 inventory

3. overlap이 있는 두 session 선정

4. 실제 alignment가 발생하는 frame 구간 탐색

5. tiny-v1 frame 목록 고정

6. reproducible subset generator 작성

7. provenance manifest 작성

8. file/config/archive SHA-256 생성

9. tiny replay 실행

10. clean environment에서 5회 반복

11. exact/tolerance baseline 결정

12. tiny-baseline-v1 승인

13. deterministic failure case 생성

14. PR replay CI 연결

15. 이후 small/representative로 확장
```

------------------------------------------------------------------------

# 23. 권장 최종 흐름

``` text
Self-Collected Original Dataset
             ↓
      Session Inventory
             ↓
    Overlap Region Search
             ↓
        tiny-v1 선정
             ↓
 Reproducible Subset Generator
             ↓
 Provenance + SHA-256 Lock
             ↓
       Replay 실행 검증
             ↓
      Clean Replay × 5
             ↓
   Exact / Tolerance 결정
             ↓
    tiny-baseline-v1 승인
             ↓
       Failure Replay
             ↓
          PR CI
             ↓
      small / representative
```

Goal 03 데이터셋 공식화에서 현재 가장 먼저 해야 할 실제 작업은 **원본
데이터의 session/agent를 inventory하고, 그중 overlap이 충분하면서
OpenLMM alignment가 실제 발생하는 두 session과 frame 구간을 `tiny-v1`
후보로 선정하는 것**이다.
