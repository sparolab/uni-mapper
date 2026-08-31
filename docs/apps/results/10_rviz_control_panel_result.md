# P7-C1 ROS Late-cancel + RViz Control Panel 결과

- 판정: `GO`
- 기준: `dev/reorder` / `542aaa2` + 현재 검증 working tree
- 일시: 2026-08-28 UTC
- toolchain: GCC 12.3.0, CMake 3.25.3, ROS 2 Humble, Qt 5.15.3

## 1. 구현 결과

- ROS terminal 결과에서 `goal_handle->is_canceling()` authority를 제거했다.
- successful Wait와 matching committed `kSucceeded`가 late cancel보다 우선한다.
- status response에 runtime revision을 추가했다.
- `open_lmm_ros/OpenLmmControlPanel` plugin DSO를 같은 ROS package에 추가했다.
- Panel은 execute/status/events만 소비하며 Run/Stage/Node/Optimize/Cancel을 제공한다.
- supplied RViz config에서 `/open_lmm_ros` 대상으로 Panel을 자동 표시한다.

## 2. 검증 결과

- exact installed-core ROS Release build/install: PASS
- developer entrypoint `make ros-build JOBS=2`: PASS; overlay plugin load PASS
- ROS CTest: 5/5 PASS
  - late-cancel/exact-job terminal resolver
  - Panel command/status/event pure model
  - Qt offscreen Panel construction
  - normal, pre-commit cancel, post-commit cancel와 next-goal ROS graph
  - 기존 Map/Path/Loop visualization contract
- installed pluginlib discovery + DSO instantiation: PASS
- Panel DSO `ldd -r`: unresolved symbol 0
- Panel DSO direct OpenLMM client/GUI/Iridescence `NEEDED`: 0
- distribution architecture/application-inventory/release-policy: 3/3 PASS
- standalone GUI installed-core regression (`Iridescence OFF`): 15/15 PASS
- standalone GUI source/link owner 변경 0; ROS Panel의 reverse dependency 0

installed Panel DSO는 1,168,400 bytes다. real display/GPU window smoke는 이번 headless
검증에 포함하지 않으며 기존 Goal 13 optional lane에 남긴다.

## 3. Evidence hashes

```text
Panel DSO          dae7ae3273e6a5d2fb86489f5db97e2afd67e9f03fcd656035f68d3eb7551ca4
plugin XML         ce5276eaee8c2551d9af82d5dcae8a107f1996f91a2ab427d9b463152a7bc099
installed RViz     1addfb38516ead0164de28795ab317b9d31e66c62bb0372ad20bdea623076635
```

## 4. 후속 admission

- P7-C2 Interactive Alignment: `PLANNED`, 구현명세 승인 가능
- P7-C3 Transactional Config Form: `PLANNED`, C2 이후 제품 순서
- Goal 09, Goal 13 real-driver와 crash durability는 별도다.
