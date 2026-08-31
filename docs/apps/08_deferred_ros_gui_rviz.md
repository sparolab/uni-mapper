# Phase 7 — ROS GUI Removal / RViz Product Decision

## 1. 상태와 확정 결정

P6 ROS relocation은 `SKIPPED / NOT REQUIRED`로 종료됐으며 root `ros/`를 유지한다.
P7은 source 위치와 무관한 별도 제품 변경으로 다음과 같이 승인됐다.

- 상태: P7 read-only 및 P7-C1 `IMPLEMENTED / GO`, C2·C3 `PLANNED`
- 제품 결정: `REMOVE_ROS_GUI + RVIZ_READ_ONLY`
- compatibility: 현재 v3 ROS composition에서 즉시 제거 승인
- 구현명세: [08_ros_rviz_visualization_implementation_spec.md](08_ros_rviz_visualization_implementation_spec.md)

구현 결과와 필수 gate는
[results/08_ros_rviz_visualization_result.md](results/08_ros_rviz_visualization_result.md)에
기록됐다. 이후 개발 환경에서 `make ros-run`의 RViz window 실행은 사용자가 확인했다.
대규모 dataset과 실제 GPU/driver 조합의 자동 회귀는 계속 Goal 13 optional lane이다.

## 2. ROS와 GUI 관계

P7은 `OpenLMMROSGui`, ROS GUI component와 ROS→GUI package dependency를 제거한다.
`open_lmm_rosnode`는 `OpenLMMROS`와 그 단일 `RuntimeClient`만 소유한다.

standalone `applications/gui`와 `open_lmm_gui`는 제거하지 않는다. 다만 standalone GUI는
자체 `RuntimeClient`를 소유하므로 ROS runtime과 상태를 공유하지 않는다. P7은 둘을
연결하는 remote GUI protocol을 추가하지 않는다.

ROS GUI가 활성화하던 interactive alignment feedback은 P7 초기 범위에서 제공하지 않는다.
runtime은 기존 headless alignment policy를 사용하며 GUI 응답을 기다려서는 안 된다.

## 3. 첫 delivery — read-only RViz

첫 P7 delivery는 committed runtime presentation을 다음 standard ROS message로 제공한다.

- agent map: `sensor_msgs/PointCloud2`
- agent trajectory: `nav_msgs/Path`
- intra/inter-loop constraints: `visualization_msgs/MarkerArray`

RViz는 읽기 전용 display adapter다. pipeline 실행, config 편집, alignment 응답을
소유하지 않으며 committed runtime state의 두 번째 owner가 되지 않는다.

```text
OpenLMMROS / RuntimeClient
  ↓ committed event
bounded async snapshot worker
  ↓ revision + generation validation
PointCloud2 / Path / MarkerArray
  ↓
RViz
```

## 4. 후속 제어 단계

초기 P7에는 RViz Panel target이나 비활성 placeholder를 만들지 않는다. 다음 단계는 별도
승인과 result를 갖는다.

1. `P7-C1`: **IMPLEMENTED / GO** — late-cancel 수정과 기존 action/status/event만
   사용하는 실행·취소·상태 Panel
2. `P7-C2`: **PLANNED** — lease 및 revision/request-id 기반 alignment ROS contract와
   InteractiveMarker
3. `P7-C3`: **PLANNED** — expected revision 기반 transactional config form과
   apply/root replacement

ROS late-cancel 수정과 deterministic resolver regression은 `P7-C1`에서 완료됐다.
Panel은 `RuntimeClient`에 직접 링크하거나 config 파일을 직접 쓰지 않는다.

## 5. 고정 계약

- last-valid presentation 유지
- stale revision/generation reject
- failure/cancel 시 empty replacement 금지
- agent id topic sanitization과 stable mapping
- frame id/TF 의미 명시
- QoS `keep_last(1)` 기본
- transient-local 사용 전 retained memory 측정
- point payload voxel/budget 제한
- snapshot→ROS message peak copy/RSS 측정
- shutdown 중 worker/publisher drain
- `rviz_frame_id`의 좌표 의미를 문서화하고 RViz bridge가 TF owner가 되지 않음
- GUI package, plugin ABI, standalone GUI와 ROS source 위치를 변경하지 않음

## 6. 완료 증거

- PointCloud2 field/layout/value mapping
- agent별 Path order/frame
- MarkerArray add/delete/stale handling
- rapid supersession
- failed replacement retention
- large-map bounded payload
- subscriber late join QoS behavior
- ROS graph E2E와 RViz config/launch smoke
- self-hosted real-driver가 필요하면 Goal 13 lane과 연계

## 7. 완료 조건

- ROS install/link closure에 GUI component와 Iridescence dependency가 없다.
- RViz bridge가 committed state의 두 번째 owner가 아니다.
- P7 baseline은 read-only로 완료됐고, 이후 C1 Panel만 별도 owner/result로 추가됐음이
  명확하다. interactive alignment/config는 여전히 미지원이다.
- resource/presentation/shutdown regression이 executable test로 보호된다.
- ROS source 위치 변경과 결합하지 않고 별도 PR/release note로 제공된다.
- 결과는 `docs/apps/results/08_ros_rviz_visualization_result.md`에 기록한다.
