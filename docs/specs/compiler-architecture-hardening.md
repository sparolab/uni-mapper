# OpenLMM 컴파일 경계 및 GCC ICE 구조 개선 명세

## 1. 배경

OpenLMM Release 빌드에서 GCC 12 internal compiler error(ICE)가 반복 발생했다.
대표 사례는 다음과 같다.

- `fmt::detail::write_int_noinline()`을 최적화하던 `dom` 패스의 segfault
- Eigen `Isometry3d::inverse()` 표현식을 최적화하던 `local-fnsummary` 패스의
  segfault
- `registration.cpp`와 표준 라이브러리 `basic_string` 코드를 최적화하던 `ira`
  패스의 segfault
- 동일한 fmt 코드에서 발생 위치가 `map_server.cpp`와 ROS 번역 단위 사이를 이동

UFOMap의 `-Wreorder` 및 Eigen의 `-Wmaybe-uninitialized` 같은 진단은 함께 출력될
수 있지만 ICE의 직접 원인은 아니다. ICE는 컴파일러 결함이지만, 무거운 공개
헤더와 과도한 템플릿 전파가 GCC 12의 취약한 최적화 경로를 반복해서 노출한다.

관찰된 실패 경로는 다음과 같다.

```text
무거운 공개 헤더와 헤더 내부 구현
  -> PCL/Eigen/KISS-Matcher/spdlog/fmt 템플릿의 연쇄 전파
  -> 크고 결합도 높은 번역 단위
  -> GCC 12에서 전체 코드를 -O3로 인라인 및 최적화
  -> dom/local-fnsummary 등 서로 다른 패스에서 ICE
```

따라서 ICE 자체의 직접 원인은 GCC 12 컴파일러 결함이며 OpenLMM의 런타임 버그가
아니다. 다만 실패 위치가 fmt, Eigen 및 서로 다른 번역 단위 사이를 이동한 것은
특정 소스 한 곳의 문제가 아니라, 공개 헤더를 통한 대규모 템플릿 전파와 중복된
고수준 최적화가 컴파일러 결함을 자주 노출하는 구조였음을 뜻한다. 이 명세는
컴파일러 버그를 애플리케이션 코드가 "수정"한다고 주장하지 않고, 취약한 최적화
입력의 크기와 결합도를 줄여 재현 가능성과 이식성을 높이는 것을 목표로 한다.

## 2. 목적

이 작업의 목적은 특정 소스 파일에 GCC 전용 최적화 비활성 옵션을 추가하는 것이
아니다. 컴파일 경계를 명확히 하여 다음 결과를 얻는 것이 목적이다.

- 공개 헤더를 통해 전파되는 구현 의존성을 최소화한다.
- 비템플릿 구현은 `.cpp`에서 한 번만 컴파일한다.
- PCL, Eigen, KISS-Matcher, spdlog/fmt가 동시에 노출되는 번역 단위를 줄인다.
- 수학 결과를 바꾸지 않고 컴파일러에 과도하게 복잡한 Eigen 표현식을 줄인다.
- CMake 최적화 정책의 중복과 타깃 간 누수를 제거한다.
- GCC 12뿐 아니라 신규 GCC 및 Clang에서도 지속해서 검증한다.

## 3. 비목표

다음 작업은 이 명세의 범위가 아니다.

- 알고리즘 결과나 dynamic remover 동작 변경
- 모든 외부 라이브러리 경고 제거
- 특정 GCC 패스를 끄는 `-fno-*` 옵션의 상시 적용
- 전체 Release 최적화 수준을 근거 없이 낮추는 변경
- PCL 또는 Eigen을 자체 자료형으로 전면 교체

## 4. 확인된 구조적 문제

### 4.1 공개 헤더의 무거운 의존성

`pipeline.hpp`의 `AgentPipelineCtx`가 `LoopDetectorOutput` 구체 타입을 값으로
보유한다. 이 때문에 단순한 파이프라인 제어 타입을 사용하려는 파일도
`loop_detector_base.hpp`와 그 하위 KISS-Matcher/PCL/Eigen 헤더를 포함한다.

최근에는 `Pipeline::Run()`과 `spdlog/spdlog.h`도 같은 공개 헤더에 추가되어 fmt
구현이 `map_server.cpp`를 포함한 모든 소비자 번역 단위로 전파됐다.

### 4.2 컴파일 계층과 소유 계층의 불일치

`Pipeline`은 공통 자료형처럼 보이지만 실제 실행 구현은 data loader, loop
detector, optimizer와 map updater를 조합하는 서버 계층의 책임이다. 이를
`open_lmm_common`에서 구현하면 KISS-Matcher 의존성이 하위 계층으로 역전된다.

### 4.3 복잡한 Eigen 지연 평가 표현식

`Eigen::Isometry3d::inverse()`와 block expression을 즉시 다른 템플릿 함수에
전달하면 GCC가 큰 표현식 트리를 인라인한다. 올바른 코드이지만 GCC 12 ICE를
유발한 사례가 있으므로 함수 경계에서는 의미가 명확한 값으로 평가해야 한다.

Rigid transform의 역변환은 다음과 같이 명시할 수 있다.

```cpp
Eigen::Isometry3d inverse = Eigen::Isometry3d::Identity();
inverse.linear() = transform.linear().transpose();
inverse.translation() = -(inverse.linear() * transform.translation());
```

이 변환은 입력 rotation이 정규 직교 행렬인 `Isometry3d` 계약을 사용하며 기존
수학 결과를 보존해야 한다.

### 4.4 CMake 최적화 정책 중복

현재 compile command에는 `-O3`가 두 번 이상 나타날 수 있다. 전역 flags,
build-type 기본값과 target property가 각각 최적화 옵션을 추가하는지 감사가
필요하다. 동일 플래그 반복이 ICE의 직접 원인은 아니지만 정책 소유권이
불명확하다는 증거다.

## 5. 목표 아키텍처

```text
common contract headers
  - 경량 enum, ID, Result, cancellation, value DTO
  - 로깅/GUI/PCL 구현 헤더 없음
             |
             v
algorithm targets
  - data loader / loop detector / optimizer / dynamic remover
  - 필요한 외부 라이브러리를 PRIVATE로 연결
             |
             v
map-server orchestration
  - Pipeline 실행 구현
  - stage/node 조합
  - 로깅 및 profiling 구현 호출
             |
             v
ROS / GUI adapters
  - 공개 command/event/snapshot 계약만 사용
```

공개 헤더의 include는 API를 선언하는 데 반드시 필요한 것만 허용한다. 구현을
위한 include와 로깅 포맷은 `.cpp`에 둔다.

## 6. 구현 요구사항

### 6.1 Pipeline 구현 분리

- `Pipeline::Run()`은 공개 헤더에서 제거하고 `.cpp`로 이동한다.
- `spdlog/spdlog.h`, `<chrono>`와 profiling 구현 include는 `.cpp`에 둔다.
- 구현은 `open_lmm_map_server` 또는 별도의 orchestration 타깃에서 컴파일한다.
- 이동 전후의 cancellation, timing log, exception 변환과 control-flow 동작은
  동일해야 한다.

### 6.2 파이프라인 DTO 분리

- `AgentPipelineCtx`가 `loop_detector_base.hpp`를 포함하지 않도록
  `LoopDetectorOutput`을 경량 DTO 헤더로 이동하는 방안을 적용한다.
- DTO에는 plugin factory, algorithm class 및 logging dependency를 포함하지 않는다.
- 값 소유가 필요하지 않은 대형 타입은 forward declaration과 PImpl 또는 명시적
  artifact repository 경계를 검토한다.
- 의존성 변경 후 target link는 가능한 한 `PRIVATE`로 제한한다.

### 6.3 로깅 및 profiling 격리

- 공개 헤더에서 직접 `spdlog::info()`를 호출하지 않는다.
- 헤더에서 필요한 계측은 경량 macro가 비활성 빌드에서 완전히 제거되어야 한다.
- 포맷 문자열을 처리하는 spdlog/fmt 호출은 비템플릿 `.cpp` 함수 뒤에 둔다.
- GUI OFF 및 Tracy OFF 빌드에서 관련 외부 헤더와 symbol이 불필요하게 전파되지
  않아야 한다.
- registration의 수치 계산, point-cloud 변환과 포맷 로깅은 서로 다른 번역
  단위로 분리한다. 수치 계산 번역 단위가 spdlog/fmt 및 PCL I/O 구현을 동시에
  인스턴스화하지 않게 한다.

### 6.4 공용 자료형 의존성 최소화

- `data_types.hpp` 같은 공용 계약 헤더는 실제 멤버 선언에 필요하지 않은 알고리즘
  구현 헤더를 포함하지 않는다.
- Scan Context, GUI, plugin factory 같은 상위 계층 구현은 공용 자료형에서
  역방향으로 참조하지 않는다.
- 타깃이 직접 사용하는 라이브러리는 직접 링크하고, 다른 타깃의 우연한
  `PUBLIC` 전이 링크에 의존하지 않는다.

### 6.5 Eigen 표현식 경계

- `inverse()`, block operation과 product expression을 PCL/GTSAM API로 바로 넘기는
  위치를 조사한다.
- 컴파일러 문제가 재현된 표현식은 명시적인 고정 크기 값으로 평가한다.
- `Isometry3d` 역변환을 수동 구현할 때 rotation 정규 직교 계약을 문서화한다.
- 변경 전후 transform을 무작위 rigid transform으로 비교하는 단위 테스트를
  추가한다.
- 일반 affine transform에 rigid inverse helper를 사용해서는 안 된다.

### 6.6 CMake 정책 정리

- Release 최적화 수준의 단일 소유 위치를 정한다.
- `CMAKE_CXX_FLAGS_RELEASE`를 `FORCE`로 덮어쓰지 않는다.
- 동일 타깃 compile command에 중복된 `-O2`/`-O3`가 발생하지 않게 한다.
- 외부 라이브러리 compile option을 OpenLMM 타깃에 PUBLIC으로 누출하지 않는다.
- GCC 전용 workaround가 불가피하면 compiler ID와 version 범위를 명시하고 제거
  조건 및 관련 이슈를 주석으로 남긴다.

### 6.7 컴파일러 검증 행렬

최소 검증 조합은 다음과 같다.

| 컴파일러 | 빌드 | GUI | 목적 |
|---|---|---|---|
| GCC 12 | Release | OFF | 현재 배포 환경 회귀 |
| GCC 12 | Release | ON | Iridescence 포함 최대 의존성 경로 |
| 최신 지원 GCC | Release | OFF | 컴파일러 업그레이드 준비 |
| Clang | Release | OFF | GCC 전용 의존과 UB 교차 검증 |

CI 환경이 준비되기 전까지 GCC 12의 GUI ON 빌드를 로컬 필수 검증으로 둔다.

Clang 검증은 단순히 GCC ICE가 사라지는지만 확인하는 절차가 아니다. GCC가
확장으로 허용하거나 진단하지 않은 표준 C++ 위반을 검출하는 교차 검증으로
사용한다. 실제 교차 빌드에서는 다음 이식성 문제가 확인됐다.

- OTD dependent type에 빠진 `typename`
- pinned UFOMap의 삭제된 `std::istringstream` 대입 연산 사용

외부 의존성 수정은 빌드 디렉터리를 직접 고치는 방식으로 처리하지 않는다.
고정된 upstream 버전에 대해 재현 가능한 patch 단계를 두고, 예상 원본과 다르면
configure 단계에서 실패하도록 한다.

### 6.8 변경 수용 절차

컴파일 경계나 빌드 정책 변경은 컴파일 성공만으로 완료 처리하지 않는다. 모든
변경은 다음 순서로 수용한다.

1. build/install/log를 제거하거나 격리한 clean Release build를 수행한다.
2. GCC 12 GUI OFF와 GUI ON을 모두 검증한다.
3. Clang Release GUI OFF를 검증한다.
4. core 및 ROS build tree의 CTest를 모두 수행한다.
5. 실제 compile command에서 컴파일러와 최적화 플래그 중복 여부를 확인한다.
6. test1/test2 실행을 3회 측정하고 wall-time median을 baseline과 비교한다.
7. pose와 PCD 산출물을 baseline artifact와 비교한다.

알고리즘 출력이 허용 오차를 넘거나 성능 회귀가 확인되면, ICE가 사라졌더라도
변경을 수용하지 않는다. 사용자 로컬 설정인 `config.json`은 검증 입력으로만
사용하며 구조 개선 커밋에 포함하지 않는다.

## 7. 구현 단계

### 단계 1: 즉시 안정화

- `Pipeline::Run()`과 spdlog 구현을 `.cpp`로 분리한다.
- `registration.cpp`의 재현되는 rigid inverse 표현식을 명시적으로 평가한다.
- GCC 12 Release, GUI ON 전체 빌드를 통과시킨다.

### 단계 2: include dependency 감사

- 주요 공개 헤더에 대해 include graph와 compile command를 수집한다.
- `pipeline.hpp`, `map_server.hpp`, plugin base header에서 불필요한 include를 제거한다.
- 제거 전후 clean build 시간과 전처리 결과 크기를 기록한다.

### 단계 3: DTO 및 타깃 계층 정리

- `LoopDetectorOutput` 등 파이프라인 전달 자료형을 경량 계약 헤더로 이동한다.
- common -> algorithm -> orchestration -> adapter 의존 방향을 강제한다.
- CMake PUBLIC/PRIVATE link interface를 재검토한다.

### 단계 4: 최적화 정책과 CI

- 중복 Release flags를 제거한다.
- GCC 12, 신규 GCC와 Clang 빌드 행렬을 추가한다.
- clean build 및 GUI ON 빌드 검증을 자동화한다.

## 8. 검증 방법

### 8.1 GCC 12 GUI OFF

```bash
cd /root/workspace
cb
```

### 8.2 GCC 12 GUI ON

```bash
cd /root/workspace
colcon build --symlink-install \
  --cmake-args -DUSE_CCACHE=OFF \
               -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=ON
```

### 8.3 Clang 15 GUI OFF

```bash
cd /root/workspace
CC=clang-15 CXX=clang++-15 colcon build --symlink-install \
  --cmake-args -DUSE_CCACHE=OFF \
               -DOPEN_LMM_BUILD_IRIDESCENCE_GUI=OFF
```

### 8.4 단위 및 회귀 테스트

```bash
ctest --test-dir /root/workspace/build/open_lmm --output-on-failure
ctest --test-dir /root/workspace/build/open_lmm_ros/open_lmm \
  --output-on-failure
```

알고리즘 코드 또는 transform 계산을 변경했다면 `docs/baseline.md`에 정의된 pose
및 PCD artifact 비교도 수행한다.

### 8.5 정적 확인

```bash
git diff --check
rg '#include <spdlog|#include <fmt' open_lmm -g '*.hpp' -g '*.h'
```

검색 결과가 무조건 0일 필요는 없지만, 공개 헤더에 남은 각 include는 API 선언에
필수라는 근거가 있어야 한다.

## 9. 완료 기준

- GCC 12 Release GUI OFF/ON 빌드가 clean build에서 모두 성공한다.
- `dom`, `local-fnsummary` 또는 fmt/Eigen 관련 ICE가 발생하지 않는다.
- OpenLMM CTest가 모두 통과한다.
- 기존 baseline 대비 pose 결과가 허용 오차 내에서 동일하다.
- `pipeline.hpp`가 spdlog/fmt와 profiling 구현을 공개하지 않는다.
- `AgentPipelineCtx` 사용자가 불필요하게 KISS-Matcher 전체를 포함하지 않는다.
- compile command의 Release 최적화 플래그가 중복되지 않는다.
- GUI OFF 빌드가 Iridescence/OpenGL 구현 의존성을 요구하지 않는다.
- 새 compiler workaround가 추가됐다면 적용 범위, 이유와 제거 조건이 문서화된다.

## 10. 금지되는 해결 방식

- 오류가 발생한 각 `.cpp`에 임의로 `-fno-tree-*` 옵션을 계속 추가하는 방식
- `map_server.cpp` 하나만 `-O0` 또는 `-O1`로 낮추고 원인을 해결했다고 판단하는 방식
- 사용하지 않는 공개 include를 유지한 채 GCC 버전만 올리는 방식
- Eigen 미초기화 경고를 경고 억제 옵션으로 숨기는 방식
- 수학적 동등성 테스트 없이 inverse 계산을 변경하는 방식

## 11. 현재 적용 상태

2026-08-15 작업 기준으로 다음 안정화 항목을 적용하고 검증했다.

- `Pipeline::Run()`을 헤더에서 구현 파일로 분리
- 공개 `pipeline.hpp`에서 spdlog와 profiling 구현 include 제거
- `LoopDetectorOutput` 계약을 공용 DTO로 이동하여 `pipeline.hpp`에서
  `loop_detector_base.hpp`와 KISS-Matcher 구현 헤더 전파 제거
- `MapUpdateNode`의 비템플릿 구현, PCL I/O와 spdlog를 `.cpp`로 분리
- `registration.cpp`의 rigid inverse를 명시적 계산으로 변경
- 후속 GCC 12 `ira` ICE 재현에 따라 registration의 수치 계산, spdlog 로깅,
  PCL point-cloud 처리를 각각 `registration.cpp`, `registration_log.cpp`,
  `registration_pointcloud.cpp`로 분리
- 공용 `data_types.hpp`에서 사용하지 않는 Scan Context 구현 include 제거
- `open_lmm_common`의 불필요한 GTSAM/TBB 전이 링크를 제거하고 실제 사용 타깃이
  GTSAM을 직접 링크하도록 의존성 소유권 수정
- 고정 seed의 무작위 rigid transform 128개로 Eigen inverse와의 동등성 및
  identity 합성을 확인하는 단위 테스트 추가
- 공개 헤더 금지 의존성과 전역 `-O3` 재도입을 막는 architecture boundary
  CTest 추가
- OpenLMM 및 ROS CMake의 중복 Release `-O3` 주입 제거
- 최초 GCC 12.3 Release, ccache OFF, GUI OFF/ON 클린 빌드 성공 후 완전한 새
  빌드에서 `registration.cpp`의 `ira` ICE가 추가로 발견됨
- registration 추가 분리 후 GCC 12.3 Release, ccache OFF, GUI ON 클린 빌드와
  core/ROS CTest 재통과
- core 및 ROS build tree CTest 각각 5/5 통과
- 주요 OpenLMM 번역 단위의 compile command에서 `-O3`가 한 번만 존재함을 확인
- GUI plugin 설치 산출물 생성 확인
- Clang 15 Release, ccache OFF, GUI OFF 클린 빌드 성공
- Clang 교차 빌드에서 발견한 OTD dependent type을 표준 C++ 문법으로 수정
- pinned UFOMap의 비표준 stream 대입을 재현 가능한 FetchContent patch로 수정
- 최종 test1/test2 3회 모두 baseline pose 및 PCD 양방향 비교 PASS
- 최종 3회 wall time `24.339 / 24.171 / 24.161초`, median `24.171초`

공식 baseline 23.79초 대비 median 차이는 +0.381초(+1.60%)이고, 기존 계측 허용
사례 24.30초보다 0.129초 빠르다. 알고리즘 결과와 성능 게이트를 통과한 것으로
판정한다.

컴파일러 행렬을 위한 GitHub Actions workflow와 동일 명령을 로컬에서 실행하는
`scripts/ci/build_and_test.sh`를 추가했다. 행렬은 GCC 12 GUI OFF/ON, GCC 13 GUI
OFF와 Clang 15 GUI OFF를 각각 독립된 clean build 경로에서 실행하며 core/ROS
CTest와 GUI plugin 산출물을 확인한다. GCC 13.4 GUI OFF 로컬 실행과 workflow
정적 검증은 통과했다.

단, 위의 최종 성능 수치와 전체 컴파일러 행렬 결과는 registration 추가 분리 전
측정값이므로 추가 분리 후 전체 검증을 다시 수행했다. 최신 소스는 다음 행렬을
모두 통과했다.

| 컴파일러 | GUI | clean build | core CTest | ROS CTest |
|---|---|---|---|---|
| GCC 12.3 | ON | PASS | 5/5 PASS | 5/5 PASS |
| GCC 12.3 | OFF | PASS | 5/5 PASS | 5/5 PASS |
| GCC 13.4 | OFF | PASS | 5/5 PASS | 5/5 PASS |
| Clang 15 | OFF | PASS | 5/5 PASS | 5/5 PASS |

GCC 12 GUI ON 작업공간 루트 빌드의 registration 관련 세 번역 단위는 모두
`g++-12`와 단일 `-O3`로 컴파일됨을 확인했다. GUI ON에서는 plugin 설치를,
GUI OFF에서는 plugin 부재를 확인했다.

같은 GCC 12 GUI ON 산출물로 test1/test2를 세 번 실행한 wall time은
`24.157 / 22.815 / 22.989초`, median `22.989초`다. 공식 baseline 23.79초보다
3.37% 빠르며 세 실행 모두 pose와 PCD 양방향 비교가 PASS했다. 따라서 로컬
구조·정확성·성능 게이트는 최종 PASS로 판정한다. GitHub-hosted CI의 실제 실행
결과는 workflow를 원격에서 실행한 뒤 별도로 기록한다.
