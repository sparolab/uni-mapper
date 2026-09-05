# 프로덕션 준비 현황

갱신일: 2026-09-05. 구현 대조 기준은 `0e17a37`과 후속 CMake 계약 정리다.
이 문서는 현재 상태를 기록한다. Git에서 제외된 `docs/`의 과거 감사·구현 명세는
작성 당시의 문제와 계획을 보존하며, 현재 미완료 목록의 기준으로 사용하지 않는다.

## 진행 순서

| 단계 | 상태 | 범위 |
|---|---|---|
| 1. 핵심 트랜잭션 회귀 보강 | 완료, `c9227d9` | commit 전 취소·저장 실패·revision 충돌·commit 후 recovery health |
| 2. 현재 코드 검증 및 후속 수정 | 로컬 검증 완료, `0e17a37` | ROS admission/cancellation, 동시 Snapshot, 테스트 종료 순서, ROS TSan CI |
| 3. 실제 데이터 반복 실행 | 대기 | 대표 데이터·큰 지도, 취소·설정 변경·재실행, 결과·최대 메모리·자원 회수 |
| 4. 빌드·문서 계약 정리 | 구현 완료, 로컬 검증 | CMake 3.25 최소 버전, 설치 consumer, CI 최소 버전 실행, 잔여 목록 갱신 |
| 5. Headless RC 배포 검증 | 대기 | 깨끗한 환경의 image/wheel 설치·실행, 동일 아티팩트 승격, 이전 버전 복구 |

합성 ROS graph 테스트는 3번의 실데이터 검증을 대신하지 않는다. 로컬 통과와 CI
정의 변경은 GitHub 원격 실행, branch protection 또는 RC 배포 완료를 뜻하지 않는다.

## 빌드 계약

- source build와 installed CMake consumer의 최소 버전은 **3.25**다.
  core/ROS의 과거 3.5 선언과 consumer fixture의 3.16 선언을 정정했다.
- core의 `add_subdirectory(... SYSTEM)`은 [CMake 3.25 기능](https://cmake.org/cmake/help/v3.25/command/add_subdirectory.html)이다.
- 일반 CMake와 ROS ament가 설치하는 core config는 모두 구버전을 dependency
  discovery 전에 거부한다. C++ API, SONAME, plugin ABI는 바뀌지 않는다.
- required `build / gcc12-gui-off` job은 CMake **3.25.0**을 사용하도록 설정했다.
  공식 release image의 **3.25.3** pin과 다른 compiler job 이름은 유지한다.
- [릴리스 정책](RELEASE_POLICY.md)의 same-toolchain 호환성 범위를 적용한다.
  algorithm plugin도 capability/name/schema/build generation을 create 전에 검증한다.

## 완료 항목과 잔여 항목

| 감사 ID | 현재 상태 |
|---|---|
| PLUG-101 | H1 완료: stale capability/name/schema/build generation 거부 |
| THR-101 / THR-102 | H1 완료: pipeline/executor 부분 thread 생성 실패 rollback |
| LIFE-101 | H1 완료: bounded joinable retirement, callback 내부 client 파괴 검증 |
| ROS-101 | H1 및 `0e17a37` 후속 완료: goal identity, pending cancel, terminal admission |
| BUILD-101 | 최소 CMake 선언·consumer·문서 정리 완료, 원격 최소 버전 job 실행 대기 |
| GUI-101 | 대기: feedback authority token과 move/multi-owner 수명 관리 |
| PATH-101 | 정책 결정 필요: 현재 외부 mutable config는 경고 후 호환 허용 |
| CI-101 | 부분 완료: thread/plugin/retirement/ROS gate 구현; GUI authority·trusted-root gate 잔여 |

CI-101의 별도 compiler-retry flaky 상태 기록과 LeakSanitizer 실행 환경도 잔여다.
현재 ownership fixture와 로그 보존을 전면적인 leak 검증 완료로 간주하지 않는다.
대용량 PCD streaming/preview, immutable dataset 정책, crash/power-loss durability는
별도 요구·설계 검토 대상이다. Recovery-required는 이미 fatal runtime health로
노출·유지되며, 전용 public 필드 추가는 별도 API 검토가 필요하다.

## 로컬 검증 기록

- 2번 최종 결과: core TSan 19/19, GUI TSan 4/4, ROS TSan 2개 각 10회,
  별도 graph 20회 연속; 일반 ROS와 ASan/UBSan ROS 각각 5/5 통과.
  [TSan 실행 계약과 검사 제외 범위](scripts/ci/README.md)를 함께 적용한다.
- 4번: CMake 3.25.0으로 새 core/ROS 빌드, 일반·ament 설치 경로와 source-free
  consumer를 검증했다. Core 전체 CTest **77/77**(package consumer 포함), ROS
  **5/5**가 통과했다. CMake 3.22.1은 여섯 source entry point와 두 installed
  core config에서 최소 버전 오류로 거부됐다.
- 원격 GitHub 최소 버전 job과 Headless RC는 이번 로컬 검증에 포함되지 않았다.
