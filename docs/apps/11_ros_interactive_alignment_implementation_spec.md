# P7-C2 ROS Interactive Alignment 구현명세서

- 상태: `PLANNED / NOT IMPLEMENTED`
- admission: P7-C1 `GO`
- owner: `OpenLMMROS`의 private alignment transport

## 1. 목적과 owner

RViz에서 alignment proposal을 보고 자동 방법 재시도, 승인, manual transform, agent 제외
또는 취소를 선택한다. ROS node가 `RuntimeClient::AlignmentFeedback`,
`RespondToAlignment`, `SetAlignmentFeedbackEnabled`와 InteractiveMarker server의 단일
owner다. Panel과 RViz display는 ROS interface만 소비한다.

## 2. installed ROS contract

- `~/alignment/review`: `open_lmm_ros/msg/AlignmentReview`
- `~/alignment/control`: `open_lmm_ros/srv/ManageAlignmentControl`
- `~/alignment/respond`: `open_lmm_ros/srv/RespondToAlignment`
- `~/alignment/interactive_marker`: standard InteractiveMarker update/feedback namespace

`AlignmentReview`는 한 message에 header/frame, job ID, request ID, session revision,
review state, target/source agent, proposal method와 transform, metrics, current attempt와 bounded
history, target/source `PointCloud2`, diagnostic target/source `Path`와 loop `MarkerArray`를 담는다.
서로 다른 revision의 payload를 topic 여러 개에서 조립하지 않는다. QoS는 reliable,
transient-local, keep-last(1)이다.

`ManageAlignmentControl`은 `ACQUIRE`, `RENEW`, `RELEASE`, lease ID와 remaining duration을
제공한다. 한 node epoch에 하나의 lease만 허용하고 lease는 5초, Panel renewal은 1초다.
lease ID는 process epoch와 monotonic sequence로 재사용되지 않게 만든다.

`RespondToAlignment`은 lease ID, job ID, request ID, session revision과 decision을 요구한다.
decision은 Accept, Try KISS, Try Descriptor, Manual, Exclude Agent, Cancel이다. Manual은 같은
request/revision marker의 current 6-DoF pose를 사용한다.

## 3. lifecycle과 failure policy

- lease 획득 때만 feedback을 enable한다.
- release, expiry, node epoch 변경 또는 Panel shutdown은 feedback을 disable하고 active review를
  cancel한다. 이후 adaptive config는 headless policy로 돌아간다.
- broker notification이 만드는 alignment event마다 bounded worker가 snapshot을 조회한다.
  동일 request의 최신 session revision만 conversion/publish한다.
- marker 이름에 request ID와 session revision을 포함하고 stale feedback을 거부한다.
- response는 모든 ID가 current와 exact match하고 broker가 응답 가능한 상태일 때 한 번만
  전달한다.
- finite rigid transform이 아니거나 Manual 이외 decision에 transform이 붙으면 거부한다.
- target/source 합계가 2,000,000 points를 넘거나 conversion이 실패하면 current review를
  cancel하고 review marker를 제거한다. 이전 review를 새 요청처럼 유지하지 않는다.
- committed Map/Path/Constraint presentation은 alignment review failure로 제거하지 않는다.

## 4. 테스트와 완료 조건

- lease acquire contention, renew, release, expiry와 node restart
- Panel disconnect 중 active review cancel 및 다음 adaptive job의 headless 진행
- request/revision/lease mismatch, duplicate response, stale marker feedback
- 각 decision과 Exclude Agent admission, invalid transform
- rapid broker updates와 stale worker completion
- payload upper bound, peak copy/RSS와 terminal retained sample
- real RViz InteractiveMarker apply/reset/cancel E2E
- C1 control 및 P7 read-only visualization 회귀

C2 완료 전에는 message/service/marker placeholder를 설치하지 않는다.
