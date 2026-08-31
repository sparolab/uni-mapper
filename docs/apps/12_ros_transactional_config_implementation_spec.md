# P7-C3 ROS Transactional Config Form 구현명세서

- 상태: `PLANNED / NOT IMPLEMENTED`
- admission: P7-C1 `GO`; 제품 순서는 C2 이후
- UI 범위: standalone GUI와 동일한 주요 필드

## 1. 목적과 편집 범위

RViz Panel이 runtime host의 파일을 직접 읽거나 쓰지 않고 committed config를 조회해 다음만
편집한다.

- dataset root, ordered agent list, output root
- Scan Context, SOLiD, STD selector
- KISS voxel size, Quatro, pose-NN maximum distance, inter-loop spacing
- ERASOR, DUFOMap, FreeDOM, HMM-MOS, OTD selector

Map-server tuning, raw JSON editor, arbitrary filesystem browser와 unknown field UI는 제외한다.

## 2. committed query contract

C3 구현 시 public `config_transaction.hpp`에 내부 type을 노출하지 않는
`CommittedConfigDocuments` DTO를 추가하고 `RuntimeClient::ConfigDocuments() const`를
additive PImpl API로 제공한다.

DTO는 runtime revision, config revision과 각 `ConfigDomain`의 canonical JSON 및 root가
선택한 logical document selector만 담는다. server absolute path, schema registry,
`RuntimeState` 또는 mutable pointer는 노출하지 않는다. 이 추가는 object layout을 바꾸지
않지만 exported symbol/public header 변경이므로 v3 exact-package compatibility review와
source-free consumer test를 요구한다.

## 3. installed ROS contract

- `~/config/get_editable`: `GetEditableConfig`
- `~/config/replace_root`: `ReplaceRootConfig`
- `~/config/apply_alignment`: `ApplyAlignmentConfig`
- `~/config/apply_map_update`: `ApplyMapUpdateConfig`

query response는 위 form 값, allowed selector 목록과 runtime/config revision을 반환한다.
모든 mutation request는 query에서 받은 expected runtime/config revision을 포함한다.

- ReplaceRootConfig는 dataset/output/ordered agents만 patch하고 기존 module selector와 unknown
  root field를 보존한 뒤 `RuntimeClient::ReplaceRootConfig`를 호출한다.
- ApplyAlignmentConfig는 selected loop-detector document와 네 alignment 값을 patch하고
  `ConfigDomain::kLoopDetector` transaction을 호출한다.
- ApplyMapUpdateConfig는 selected remover document를 검증하고
  `ConfigDomain::kDynamicRemover` transaction을 호출한다.

Panel은 candidate JSON이나 filesystem path를 만들지 않는다. ROS adapter가 committed
document의 복사본을 patch하고 schema validation을 통과한 candidate만 core transaction에
전달한다.

## 4. concurrency와 failure policy

- expected revision mismatch는 current revisions와 conflict error를 반환한다. 자동 retry,
  merge 또는 last-writer-wins는 금지한다.
- Panel은 conflict 후 draft를 유지하되 Apply를 비활성화하고 사용자가 Reload해야 한다.
- active job 동안 mutation UI를 비활성화하지만 server가 최종 admission authority다.
- schema/path/selector/filesystem failure와 commit 전 cancellation은 이전 committed state와
  file을 유지한다.
- root replacement 성공 시 새 epoch/revisions를 query해 Panel, event watermark와 RViz
  derived presentation을 resync한다.
- persisted JSON은 storage owner의 two-space formatting과 기존 key order preservation을
  계속 사용한다. Panel이 document 전체를 재직렬화하지 않는다.

## 5. 테스트와 완료 조건

- committed query가 file mutation이나 internal pointer를 노출하지 않음
- 각 form field round-trip과 selector allowlist
- stale runtime/config revision, simultaneous editors와 explicit reload
- root replacement receipt 및 old epoch callback rejection
- alignment/remover domain invalidation과 upstream artifact preservation
- unknown key와 기존 key order 보존
- invalid schema/path, write failure, cancellation과 rollback
- installed-prefix ROS consumer 및 public C++ compatibility test
- C1/C2/P7 visualization regression

C3 완료 전에는 public query나 ROS service placeholder를 추가하지 않는다.
