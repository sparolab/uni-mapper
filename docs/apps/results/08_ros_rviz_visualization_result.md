# P7 ROS GUI Removal + RViz Read-only Visualization 결과

- 판정: `GO`
- 검증 기준: `dev/reorder` / P7 commit `4451fc6`
- 후속 검증 기준: `542aaa2` + P7-C1 working tree
- 일시: 2026-08-27 UTC
- toolchain: GCC 12.3.0, CMake 3.25.3, ROS 2 Humble

## 1. Owner cutover

- `OpenLMMROSGui` source/header와 ROS GUI component를 제거했다.
- `OPEN_LMM_ROS_BUILD_GUI`, `gui_enabled`, `gui_plugin_path`와 ROS→GUI package/link
  dependency를 제거했다.
- `open_lmm_rosnode`는 항상 `OpenLMMROS` 하나를 생성한다.
- `OpenLMMROS`가 기존 단일 `RuntimeClient`와 private `RosVisualizationBridge`를 소유한다.
- standalone `applications/gui`와 GUI ABI/loader는 변경하지 않았다.

installed ROS DSO `libopen_lmm_ros.so.3.0.0`의 `NEEDED`에는
`libopen_lmm_client.so.3`, `sensor_msgs`, `nav_msgs`, `visualization_msgs`가 있고 GUI 또는
Iridescence가 없다. ROS install tree에서도 GUI artifact는 0개다.

## 2. Topic, QoS와 resource 계약

| Topic | Type | QoS |
|---|---|---|
| `~/visualization/<agent-key>/points` | `sensor_msgs/PointCloud2` XYZI float32 | reliable, transient-local, keep-last(1) |
| `~/visualization/<agent-key>/path` | `nav_msgs/Path` | reliable, transient-local, keep-last(1) |
| `~/visualization/loops` | `visualization_msgs/MarkerArray` | reliable, transient-local, keep-last(1) |

bridge는 worker 하나와 agent별 latest-request slot 하나만 사용한다. 동일 agent 요청은
최신 generation으로 덮어쓰며 generation/revision admission을 publish 직전에 다시 검사한다.
query 또는 conversion 실패, point-count 초과와 stale completion은 publish하지 않아 이전
transient-local sample이 유지된다. authoritative agent 제거만 empty point/path 및 두 loop
marker의 explicit `DELETE`를 publish한다.

`rviz_max_point_count`는 `1..2000000`, preview voxel은 기본 `0.4`이고 finite `>= 0`, frame은 non-empty로
fail-closed한다. graph fixture의 `/usr/bin/time -v` peak RSS는 90,240 KiB였고 exit status는
0이었다. 이는 작은 test fixture process 전체 측정값이며 대규모 dataset의 Goal 13 성능
evidence를 대체하지 않는다.

## 3. 검증 결과

- ROS exact-installed-core Release build/install: PASS
- ROS CTest: 3/3 PASS
  - conversion/admission/removal contract
  - goal admission
  - action/status/event + Map/Path/Loop graph, transient-local late join
- distribution policy: 7/7 PASS
- combined distribution: 8/8 PASS
- standalone GUI regression: 15/15 PASS
- `git diff --check`: PASS
- source-free compile DB: core/application source path 0
- installed launch import/generation: PASS, 7 launch entities
- installed RViz config/README presence: PASS
- developer `make ros-run` RViz window: 사용자 실행 확인
- large-dataset real GPU/driver 자동 회귀: Goal 13 optional integration lane으로 유지

combined owner inventory는 core 383, CLI 6, GUI 18, ROS 155, collision 0이다. version skew,
upgrade와 uninstall simulation도 모두 PASS다.

## 4. Evidence hashes

```text
owner inventory  12bd2e14eb1b53a9e9cf465beb5eb0751deacae3aff7de5b996f08a5cdbff526
metrics          ce36a739167ef512237dc4989b754db47f0f2cf5b60d767ada6c74807cc33353
ROS DSO          e072cff7efc8f7a4105ea5b030596065973e4e9d912f8aa8601df0ce88ec8f1c
launch           b4ca593de5dd392d8dfdb643abdeb6700021c456089b47459d499ff2cd7d0b72
RViz config      6aa79a8e3e5aaac81aa625085b92097a14ddb4a3673b923e43f10697f39efcb7
```

## 5. 후속 admission

- `P7-C1` RViz Control Panel: `GO`; late-cancel terminal-state 수정 포함
- `P7-C2` alignment와 `P7-C3` transactional config API: `PLANNED`, 구현명세 작성
- Goal 09 공급망과 Goal 13 real-driver/window 검증: P7 완료와 별도

따라서 P7 read-only visualization은 `GO`지만 ROS 제어 UI, interactive alignment 또는
config editor가 완료됐다는 의미는 아니다.
