# P7-C1 RViz Control Panel 구현명세서

- 상태: `IMPLEMENTED / GO` (검증 working tree)
- 기준선: `dev/reorder` / `542aaa2`
- 선행조건: P7 read-only visualization `GO`, ROS late-cancel 수정

## 1. 목적과 경계

`open_lmm_ros`가 제공하는 기존 `ExecutePipeline` action, `GetRuntimeStatus`
service와 `ExecutionEvent` topic만으로 RViz 안에서 단일 runtime을 제어한다. Panel은
`RuntimeClient`, config file 또는 standalone GUI에 접근하지 않는 ROS leaf consumer다.

포함 범위는 Run All, 네 Stage, 여섯 Node, Optimize Through, active goal Cancel과
runtime/job/revision/progress 표시다. alignment review와 config editing은 각각 C2와 C3다.

## 2. 패키지와 dependency

- `open_lmm_ros`가 `libopen_lmm_rviz_control_panel.so`와 pluginlib XML을 설치한다.
- plugin class는 `open_lmm_ros/OpenLmmControlPanel`이며 base는
  `rviz_common::Panel`이다.
- Panel DSO는 Qt5 Widgets, rviz_common, pluginlib, rclcpp/action과 같은 package의
  generated ROS types만 사용한다.
- `open_lmm::client`, Iridescence, GUI plugin 또는 core header에는 링크하지 않는다.
- supplied `open_lmm.rviz`의 `Panels`에 기본 target namespace
  `/open_lmm_ros`로 등록한다.

## 3. command와 상태 계약

Panel은 target namespace 뒤에 `execute`, `status`, `events`를 붙인다. namespace는 RViz
config에 저장하며 변경 시 이전 subscription/client callback을 generation으로 폐기한다.

- Stage 값: DataLoad `0`, Alignment `1`, MapUpdate `2`, Save `3`
- Node 값: DataLoad `0`, LoopDetect `1`, Optimize `2`, MapUpdate `3`, PoseSave `4`,
  FallbackMapSave `5`
- 앞의 네 Node와 Optimize Through는 status의 agent가 필수다.
- action wire field는 변경하지 않으며 private contract test로 v3 숫자 mapping을 고정한다.
- status는 500ms 주기로 authoritative resync하고 event는 즉시 progress/message를 갱신한다.
- duplicate/out-of-order event는 무시하고 sequence gap은 status refresh를 요청한다. 500ms
  authoritative status baseline마다 local event watermark를 다시 세우므로 node 재시작 뒤 낮아진
  sequence도 이전 epoch 값에 막히지 않는다.
- active job 동안 submit button을 비활성화한다. Cancel은 Panel이 제출하지 않은 goal도
  포함해 action server의 현재 goal 전체에 요청한다.

ROS callback은 widget을 직접 갱신하지 않고 queued Qt invocation으로 전달한다. Panel
shutdown과 endpoint 교체 후 도착한 callback은 폐기한다.

## 4. late-cancel terminal 계약

ROS cancel-request 상태는 terminal authority가 아니다. job별 `RuntimeClient::Wait()` 성공은
항상 `SUCCEEDED`이며, 실패 receipt는 exact matching snapshot의 `kSucceeded`,
`kCancelled`, `kFailed`를 순서대로 success, canceled, aborted로 매핑한다. matching snapshot이
없으면 cancel error만 canceled이고 나머지는 aborted다. 성공 result는 error message를
노출하지 않는다.

`GetRuntimeStatus`에는 기존 config revision과 함께 committed runtime revision을 반환한다.
이는 response 끝에 추가된 v3 additive ROS field이며 endpoint 이름과 request shape는 유지한다.

## 5. 완료 증거

- terminal resolver의 committed-success/pre-commit-cancel/failure matrix
- ROS graph의 normal/cancel/status/action/event regression
- command shape, per-agent validation, status gating과 event gap model test
- `QT_QPA_PLATFORM=offscreen` Panel construction test
- plugin XML, installed DSO와 supplied RViz config 존재
- Panel DSO의 `NEEDED`에 OpenLMM client/GUI/Iridescence가 없음
- ROS CTest, distribution architecture/package gate와 standalone GUI regression 유지
