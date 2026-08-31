# P7 ROS GUI Removal + RViz Read-only Visualization 구현명세서

- 상태: `IMPLEMENTED / GO` (검증 working tree; commit 전)
- 기준 branch/commit: `dev/reorder` / `863376d`
- 선행 판정: P4·P5 `GO`, P6 `SKIPPED / NOT REQUIRED`
- 제품 결정: `REMOVE_ROS_GUI + RVIZ_READ_ONLY`
- compatibility: 현재 v3 ROS composition에서 제거 승인

## 1. 목적과 완료 경계

P7은 ROS process 안에서 Iridescence GUI DSO를 load하던 `OpenLMMROSGui` composition을
제거하고, headless `OpenLMMROS`의 committed presentation을 RViz standard message로
표시한다.

첫 delivery는 다음만 완료한다.

- agent별 map `PointCloud2`
- agent별 trajectory `Path`
- intra/inter-loop `MarkerArray`
- 설치된 launch와 RViz config

RViz는 read-only presentation consumer다. runtime state, revision, job, config 또는
alignment authority를 새로 소유하지 않는다.

다음은 P7 첫 delivery에 포함하지 않는다.

- RViz Control Panel 또는 placeholder UI
- config apply/root replacement ROS API
- alignment review/response 또는 InteractiveMarker
- standalone `open_lmm_gui` 제거·변경
- remote GUI protocol
- ROS relocation
- ROS late-cancel 수정
- core public API/ABI 또는 plugin ABI redesign
- Goal 09 artifact publication과 Goal 13 real-driver 완료 선언

## 2. 현재 구조와 owner cutover

현재 `OPEN_LMM_ROS_BUILD_GUI=ON`이면 `open_lmm_rosnode`가
`OpenLMMROSGui`를 생성한다. 이 derived node는 base `OpenLMMROS`의 `RuntimeClient`를
`GuiRuntimeHost`와 공유하고 Iridescence plugin을 같은 process에 load한다.

P7 이후 canonical topology는 다음과 같다.

```text
open_lmm_rosnode / OpenLMMROS
  ├── RuntimeClient                 authoritative runtime owner
  ├── ExecutePipeline action
  ├── GetRuntimeStatus service
  ├── ExecutionEvent publisher
  └── RosVisualizationBridge        derived presentation owner
        └── bounded refresh worker
              ├── PointCloud2
              ├── Path
              └── MarkerArray
```

`RosVisualizationBridge`는 `OpenLMMROS`가 소유하는 private helper다. 별도 ROS component,
별도 `RuntimeClient`, core service 또는 repository를 만들지 않는다. bridge가 쓰는
`RuntimeClient` reference는 node보다 먼저 파괴될 수 없으며 shutdown에서 worker를 먼저
join한 뒤 runtime을 close한다.

## 3. ROS GUI 제거 계약

다음을 하나의 owner cutover로 제거한다.

- `OpenLMMROSGui` source/header와 component registration
- `open_lmm_ros_gui_component` DSO
- `OPEN_LMM_ROS_BUILD_GUI` CMake option과 조건부 node composition
- ROS의 `find_package(open_lmm ... COMPONENTS gui)` 및 `open_lmm::gui` link
- `gui_enabled`, `gui_plugin_path` ROS parameter
- ROS GUI application-boundary manifest edge와 package assertion

`open_lmm_rosnode`는 항상 `OpenLMMROS`를 생성하고 `open_lmm_ros_component`에만 link한다.
ROS package는 exact installed core `3.0.0`의 client contract만 소비한다.

이 제거는 현재 v3에서 승인된 ROS composition compatibility 변경이다. 삭제된 parameter나
component를 무시해 headless로 계속 실행하지 않는다. 사용자가 제거된 parameter를 넘긴
경우 ROS undeclared-parameter 정책에 따라 명확히 실패해야 한다. release note에는 다음
migration을 기록한다.

```text
old: open_lmm_rosnode + gui_enabled/gui_plugin_path + in-process GUI
new: open_lmm_rosnode + RViz launch/config + read-only visualization topics
```

standalone `applications/gui`, `open_lmm_gui`, `open_lmm_gui::gui`, v3 Compat-A
`open_lmm::gui`와 GUI plugin ABI v1은 P7에서 변경하지 않는다.

## 4. RViz bridge lifecycle

### 4.1 시작과 refresh trigger

runtime open과 event subscription이 성공한 뒤 bridge를 시작한다. 최초
`RuntimeClient::Snapshot()`의 agent catalog로 publisher state를 만들고 agent별 refresh를
요청한다.

기존 `ExecutionEvent` callback은 ROS event publish 후 bridge에 작은 refresh intent만
enqueue한다. callback thread에서 point projection, file read, ROS message allocation 또는
publish를 수행하지 않는다.

다음 committed 변화가 refresh trigger다.

- artifact commit/invalidation
- stage completion
- job completion
- root/config replacement 뒤 agent catalog 또는 visualization revision 변화

진행률 event만으로 full point refresh를 만들지 않는다. 첫 delivery는 실행 중 candidate
preview를 publish하지 않는다.

### 4.2 bounded worker와 generation

bridge는 worker `std::jthread` 하나와 agent별 latest-request slot 하나만 소유한다.
같은 agent의 미처리 요청은 최신 generation으로 덮어쓴다. queue는 configured agent 수보다
커질 수 없고 shutdown은 stop request 후 condition variable wake, worker join 순서다.

worker는 먼저 metadata-only `VisualizationQuery(include_points=false)`를 조회한다. point
subscriber가 있거나 transient-local point sample을 아직 만들지 않은 agent만
`include_points=true` query를 수행한다. point query는 `rviz_preview_voxel_size_m`을
전달한다.

조회와 conversion을 마친 뒤 다음을 모두 만족할 때만 publish한다.

- request generation이 agent의 최신 generation과 일치
- snapshot revision이 이미 publish된 revision보다 작지 않음
- authoritative snapshot에 agent가 계속 존재
- message frame과 payload validation 통과

실패, cancellation, supersession 또는 payload 초과는 기존 publisher sample을 삭제하지
않는다. 로그만으로 future behavior를 바꾸지 않으며 retry 여부는 다음 committed trigger나
subscriber transition이 결정한다.

### 4.3 agent 제거와 epoch 종료

authoritative runtime snapshot에서 기존 agent가 사라진 경우만 explicit removal이다.
bridge는 해당 agent에 empty PointCloud2와 empty Path, 관련 marker `DELETE`를 publish한 뒤
publisher와 retained metadata를 제거한다.

runtime reopen/root replacement로 epoch가 바뀌면 old epoch worker completion을 모두
폐기하고 explicit removal을 publish한 뒤 새 catalog를 시작한다. query 실패나 일시적
subscriber disconnect를 agent 제거로 해석하지 않는다.

## 5. Topic과 message 계약

모든 이름은 node-private relative topic으로 만들며 node namespace/remap을 존중한다.

| Topic | Type | Owner | 내용 |
|---|---|---|---|
| `~/visualization/<agent-key>/points` | `sensor_msgs/msg/PointCloud2` | agent publisher | committed map/preview points |
| `~/visualization/<agent-key>/path` | `nav_msgs/msg/Path` | agent publisher | snapshot pose order의 trajectory |
| `~/visualization/loops` | `visualization_msgs/msg/MarkerArray` | bridge | 전체 agent의 intra/inter-loop |

### 5.1 agent-key

`AgentId`에서 충돌 없이 다음과 같이 생성한다.

- prefix: `a_`
- ASCII alphanumeric: 그대로 유지
- `_`: `_5f`
- `-`: `_2d`
- `.`: `_2e`

예: `agent-1` → `a_agent_2d1`, `agent_1` → `a_agent_5f1`.
AgentId public validation이 허용하지 않는 byte는 fail-closed한다. mapping은 process와
config order에 의존하지 않으며 별도 mutable registry를 만들지 않는다.

### 5.2 frame과 timestamp

- parameter `rviz_frame_id`: non-empty string, 기본 `map`
- 모든 points/path/marker header는 같은 frame을 사용한다.
- bridge는 TF를 broadcast하지 않고 좌표를 재변환하지 않는다.
- snapshot pose와 point는 runtime global/map frame 값으로 해석한다.
- header stamp는 동일 publish batch에서 얻은 node clock time 하나를 공유한다.

### 5.3 PointCloud2

- height `1`, width `points.size()`, little-endian host 지원
- fields: `x`, `y`, `z`, `intensity`, 모두 `FLOAT32`
- offsets: `0`, `4`, `8`, `12`; point step `16`
- row step: `width * 16`
- `is_dense=false`
- snapshot point 순서를 유지하고 hidden PCL container를 만들지 않는다.

NaN/Inf point가 발견되면 일부만 조용히 삭제하지 않고 해당 replacement 전체를 거부한다.
이전 정상 cloud는 유지한다.

### 5.4 Path

`VisualizationPose.index` 오름차순으로 재정렬하지 않고 snapshot vector 순서를 그대로
사용한다. 각 `PoseStamped`는 transform translation/quaternion을 사용한다. non-finite 또는
non-normalizable pose가 하나라도 있으면 새 Path 전체를 거부한다.

### 5.5 loop MarkerArray

bridge는 최신 accepted metadata snapshot을 agent별로 보관하고 pose lookup을 만든다.
두 endpoint가 모두 존재하는 edge만 표시한다.

- marker namespace `open_lmm/intra_loop`: intra-agent loop
- marker namespace `open_lmm/inter_loop`: inter-agent loop
- type `LINE_LIST`, action `ADD`, fixed id `0`
- intra/inter marker는 서로 다른 고정 색상과 parameterized line width를 사용한다.
- 새 full marker batch를 만든 뒤 같은 id로 replace한다.
- explicit removal에만 `DELETE`를 사용한다.

trajectory edge는 Path가 소유하므로 MarkerArray에서 중복 표시하지 않는다.

## 6. QoS와 resource policy

points, path, loop topic은 모두 다음 QoS를 사용한다.

```text
history: KEEP_LAST
depth: 1
reliability: RELIABLE
durability: TRANSIENT_LOCAL
```

late subscriber는 마지막 committed presentation을 즉시 받아야 한다. retained sample과
bridge metadata는 agent당 하나만 유지한다.

고정 parameter는 다음과 같다.

| Parameter | 기본값 | validation |
|---|---:|---|
| `rviz_visualization_enabled` | `true` | boolean |
| `rviz_frame_id` | `map` | non-empty |
| `rviz_preview_voxel_size_m` | `0.4` | finite, `>= 0`; zero는 runtime canonical resolution |
| `rviz_max_point_count` | `2000000` | `1..2000000` |
| `rviz_loop_line_width_m` | `0.03` | finite, `> 0` |

point snapshot이 `rviz_max_point_count`를 넘으면 truncate하지 않는다. replacement을
거부하고 voxel size를 높이라는 diagnostic을 남긴다. runtime projection, snapshot,
PointCloud2 conversion과 DDS serialization의 peak copy/RSS를 result에 기록한다.

`rviz_visualization_enabled=false`이면 bridge worker와 visualization publisher를 만들지
않는다. action/service/event를 제공하는 headless ROS runtime behavior는 유지한다.

## 7. Launch, RViz config와 package ownership

`open_lmm_ros` package가 bridge, launch와 RViz config를 함께 소유한다. 별도 RViz package나
빈 Panel plugin package를 만들지 않는다.

설치 항목은 다음과 같다.

- `share/open_lmm_ros/launch/open_lmm_rviz.launch.py`
- `share/open_lmm_ros/rviz/open_lmm.rviz`

launch argument는 `config_path`, `rviz_frame_id`, `rviz_preview_voxel_size_m`,
`rviz_max_point_count`, `use_rviz`다. node와 RViz를 같은 namespace/remap 계약으로
실행한다. `use_rviz=false`는 topic producer만 실행한다.

RViz config는 `Agent1`부터 `Agent7`까지의 group을 제공하고 각 group 안에 `Map`,
`Path`, `Constraint` display를 둔다. Map과 Path는 agent마다 서로 다른 동일 계열 단색을
사용하며 빨강은 향후 dynamic 전용 presentation을 위해 사용하지 않는다. 현재 committed
snapshot은 `kFinalStaticMap`만 제공하고 point별 static/dynamic class를 제공하지 않으므로
근거 없이 dynamic cloud를 추정하거나 별도 topic을 만들지 않는다. Constraint는 기존 전역
loop MarkerArray topic을 각 group에서 표시한다. 동적 agent topic을 자동 발견하는 custom
display는 만들지 않는다.

package dependency는 exact `open_lmm 3.0.0`, `sensor_msgs`, `nav_msgs`,
`geometry_msgs`, `visualization_msgs`, ROS runtime/component/launch로 제한한다. `rviz2`는
launch 사용자를 위한 exec dependency이며 `OpenLMMROS` library의 link dependency가 아니다.

## 8. Architecture와 compatibility

- `RuntimeStateStore`와 `RuntimeClient`가 authoritative state owner다.
- bridge repository는 revision/generation과 last-published metadata만 보관한다.
- candidate, failed 또는 stale snapshot은 committed presentation을 대체하지 않는다.
- event callback과 ROS executor thread에서 expensive visualization work를 하지 않는다.
- runtime/state/domain/core는 ROS message나 RViz에 의존하지 않는다.
- ROS adapter는 installed public façade만 소비한다.
- `RuntimeClient` PImpl, C++ API/ABI와 plugin ABI v1은 변경하지 않는다.
- ROS action/service/message 이름과 `open_lmm_rosnode` executable 이름은 유지한다.
- ROS GUI component/parameters 제거만 승인된 breaking surface다.

ROS late-cancel은 별도 blocker다. P7 read-only bridge 구현에 섞지 않지만 제어 Panel
`P7-C1` admission 전에 authoritative terminal state 기준으로 수정하고 exact post-commit
cancel regression을 통과해야 한다.

후속 상태: 이 admission blocker는 P7-C1에서 해소됐으며 결과는
`results/10_rviz_control_panel_result.md`에 기록한다. 위 문장은 P7 baseline과 C1의 구현
순서를 보존한다.

## 9. 구현 순서

### WP1 — Characterization과 conversion contract

1. 기존 ROS GUI owner/link/install inventory를 고정한다.
2. agent-key, PointCloud2, Path, MarkerArray pure conversion contract test를 먼저 추가한다.
3. QoS, topic, frame과 resource parameter golden을 고정한다.

### WP2 — Private bridge

1. single-worker generation/coalescing owner를 추가한다.
2. `OpenLMMROS`의 기존 RuntimeClient와 event subscription에 연결한다.
3. last-valid, explicit removal, epoch reset과 shutdown ordering test를 통과한다.

### WP3 — ROS GUI owner removal

1. node를 unconditional `OpenLMMROS` composition으로 전환한다.
2. GUI source, component, option, parameters와 package dependency를 같은 commit 경계에서
   제거한다.
3. architecture manifest를 새 RViz edge로 갱신하고 core/standalone GUI 무변경을 확인한다.

### WP4 — Installed user path

1. launch, RViz config, dependency와 install rule을 추가한다.
2. installed-prefix ROS graph에서 late-join 및 source-free runtime smoke를 실행한다.
3. payload/RSS evidence와 result 문서를 기록한다.

중간 commit을 나눈다면 각 commit은 production owner와 test owner가 함께 존재해야 한다.
old GUI owner와 new RViz owner를 모두 canonical 상태로 장기간 유지하지 않는다.

## 10. Test plan

### 10.1 Contract/unit

- agent-key escape가 허용 AgentId 전체에서 결정적이고 collision-free
- PointCloud2 field name/type/offset/step/value/intensity
- Path vector order, transform, frame/stamp
- intra/inter-loop endpoint lookup, namespace/color/add/delete
- invalid frame/voxel/count와 non-finite payload fail-closed
- point-count 초과 시 truncate/empty replacement 없음

### 10.2 Concurrency/presentation

- rapid same-agent refresh가 latest generation 하나로 coalesce
- old revision과 slow stale completion reject
- query/conversion/cancel failure에서 previous sample 유지
- replacement message 완성 전 previous presentation 유지
- explicit agent removal만 empty/delete publish
- shutdown 중 queued/active worker drain 후 RuntimeClient close
- queue와 retained state가 configured agent 수에 bounded

### 10.3 ROS graph

- node open 후 action/status/event 기존 contract 유지
- Run action commit 뒤 points/path/loops의 frame과 revision-consistent batch 확인
- late subscriber가 transient-local latest sample 수신
- root replacement agent removal에서 stale RViz display 삭제
- namespace/remap launch와 `use_rviz=false` 동작
- headless alignment policy에서 GUI response 대기 없이 terminal result 도달

### 10.4 Package/policy

- GUI option 없이 exact installed core로 ROS configure/build/install
- ROS DSO/executable `NEEDED`, RPATH와 install manifest에 GUI/Iridescence가 없음
- repository architecture test에 ROS→GUI edge가 0
- standalone GUI build/package/lifetime test 무변경
- ROS CTest, GCC 12 installed-prefix lane, architecture/release gate와 `git diff --check`
- launch import, RViz config parse와 opt-in RViz process smoke

required CI는 standard message subscriber로 graph를 검증한다. 실제 window/GPU smoke는
Goal 13의 opt-in self-hosted/nightly lane으로 분리하며 headless required lane을 대체하지
않는다.

## 11. 완료 판정과 result

다음이 모두 충족돼야 P7 첫 delivery를 `GO`로 기록한다.

- [x] ROS GUI component/source/parameter/package dependency owner 0
- [x] ROS runtime owner는 `OpenLMMROS`의 RuntimeClient 하나
- [x] Map/Path/Loop topic contract와 late join PASS
- [x] stale/failure/removal/shutdown presentation contract PASS
- [x] payload limit과 peak RSS evidence 기록
- [x] installed-prefix launch/RViz config smoke PASS
- [x] 기존 ROS action/status/event와 standalone GUI regression PASS
- [x] result에 verified baseline, artifact/owner inventory와 후속 `P7-C1` admission 기록

결과 문서는 `docs/apps/results/08_ros_rviz_visualization_result.md`에 작성한다. 최소 다음을
포함한다.

- implementation commit과 toolchain
- 제거된 GUI owner/link/install count
- topic/QoS/parameter inventory
- agent/point/pose/edge sample counts
- stale/replacement/removal/late-join 결과
- point conversion peak RSS와 retained DDS payload
- source-free build, launch/RViz smoke evidence hash
- ROS late-cancel 상태와 `P7-C1` admission `GO`/`NO-GO`

## 12. 후속 Control Panel 계약

P7 첫 delivery는 다음 source/target을 만들지 않지만 후속 방향을 고정한다.

### P7-C1 — Operational Panel

- RViz Panel은 ROS action/status/event만 소비한다.
- Run All, Stage, Node, Optimize Through, Cancel, progress와 health를 제공한다.
- core/RuntimeClient/GUI library에 link하지 않는다.
- ROS late-cancel fix와 exact regression 없이는 admission `NO-GO`다.

### P7-C2 — Alignment

- alignment snapshot/response는 job id, request id와 session revision을 모두 운반한다.
- stale response는 fail-closed하며 runtime의 `RespondToAlignment` authority를 우회하지 않는다.
- InteractiveMarker는 candidate transform만 표현하고 Accept/Reject가 commit barrier를
  대신하지 않는다.
- Panel 미접속 시 기존 headless policy가 계속 동작한다.

### P7-C3 — Config

- Panel은 config file을 직접 읽고 쓰지 않는다.
- apply/root replacement request는 expected runtime/config revision과 schema domain을
  운반한다.
- candidate → validate → file commit barrier → committed runtime publication 순서를
  그대로 사용한다.
- 성공/실패 후 authoritative status를 다시 조회하고 revision divergence를 허용하지 않는다.

각 후속 단계는 별도 implementation spec, result와 rollback 경계를 갖는다. P7 read-only
완료를 이유로 이 단계들을 선승인하거나 완료 처리하지 않는다.

## 13. 중단 및 rollback 조건

다음 중 하나가 필요하면 P7 구현을 확대하지 않고 `NO-GO`로 기록한다.

- RViz bridge가 별도 RuntimeClient나 committed-state owner를 필요로 함
- core public API/ABI 또는 plugin ABI 변경이 필요함
- last-valid presentation을 유지하지 못하고 clear-before-build가 필요함
- unbounded worker queue 또는 payload retention이 필요함
- ROS adapter가 source-tree GUI/core private header를 소비해야 함
- standalone GUI 제거 또는 ROS relocation을 같은 cutover에 섞어야 함

rollback은 P7 owner-cutover commit을 revert해 기존 `OpenLMMROSGui` composition을 복원한다.
core, P1–P5 leaf owner와 P6 결정은 되돌리지 않는다. P7 실패 시 half-installed GUI/RViz
owner나 compatibility shim을 남기지 않는다.
