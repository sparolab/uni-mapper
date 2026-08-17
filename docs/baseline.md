# test1/test2 기준선

기준일: 2026-08-15
기준 커밋: `c3f4912`
대상 설정: `test1`, `test2`, Scan Context, incremental optimizer, ERASOR

## 보존 위치

```text
/root/workspace/output/baseline_c3f4912_test1_test2_2026_08_15
```

이 디렉터리에는 다음 자료가 있다.

- `optimized_poses_A.txt`, `optimized_poses_B.txt`
- `global_map_A.pcd`, `global_map_B.pcd`
- `run.log`
- `SHA256SUMS`

## 실행 방법

```bash
cd /root/workspace
cb
sb
ros2 run open_lmm_ros open_lmm_rosnode
```

측정 실행에서는 Bash `time -p`를 사용했다. GNU time과 Tracy는 현재 환경 및
현재 코드에 포함되지 않아 peak RSS와 함수 단위 profile은 이 기준선에 없다.
현재 코드는 각 Pipeline node 종료 시 다음 형식의 경량 계측 로그를 출력한다.

```text
[PROFILE] agent=A module=DataLoad elapsed_ms=1124.261
```

계측 범위는 `DataLoad`, `LoopDetect`, `Optimize`, `MapUpdate`다.

## 결과

| 항목 | Agent A | Agent B |
|---|---:|---:|
| 입력/optimized pose 수 | 310 | 358 |
| 저장 PCD point 수 | 1,639,880 | 1,796,511 |
| pose 파일 크기 | 21,568 bytes | 24,740 bytes |
| PCD 파일 크기 | 20,555,328 bytes | 22,535,854 bytes |

전체 실행 시간:

| 지표 | 값 |
|---|---:|
| real | 23.79 s |
| user | 177.24 s |
| sys | 3.04 s |

진행 로그에서 관찰된 주요 구간은 다음과 같다. tqdm 표시는 내부 진행률 기반의
근사치이므로 정밀 profiler 수치로 취급하지 않는다.

- Agent A Data Loader: 약 0.6 s
- Agent A Intra Loop Detector: 약 0.2 s
- Agent A Intra Backend Optimizer: 약 0.1 s
- Agent B Data Loader: 약 0.7 s
- Agent B Intra Loop Detector: 약 0.2 s
- Agent B Inter Loop Detector: 약 0.3 s
- Agent B Intra Backend Optimizer: 약 1.6 s
- Agent B Inter Backend Optimizer: 약 10.3 s
- Agent A Dynamic Remover: 약 1.9 s
- Agent B Dynamic Remover: 약 2.3 s

## 향후 변경 합격 기준

정확성과 성능을 별도로 판정한다.

### 정확성

- pose의 agent/index 집합과 개수가 동일해야 한다.
- translation 및 rotation 차이는 사전에 정한 tolerance 이하여야 한다.
- 기본 회귀 검사는 pose만 비교한다.
- PCD 검사는 map 생성·저장 경로를 변경할 때 `--pcd` 옵션으로 추가 수행한다.
- binary hash 불일치만으로 실패 처리하지 않는다. 저장 순서와 압축 차이가 있을 수 있다.
- 알고리즘 변경으로 결과 차이를 허용할 때는 근거와 새 baseline 승인을 기록한다.

### 성능

- 동일한 입력, config, build type과 유사한 시스템 부하에서 비교한다.
- 빠른 개발 반복에서는 1회 실행 결과로 우선 판정한다.
- 전체 wall time이 baseline과 비슷하거나 더 빨라야 한다.
- 새 profile 계측은 기본 비활성 빌드에서 측정 가능한 overhead를 만들지 않아야 한다.
- 최적화 대상 모듈은 구간 시간이 개선되어야 하며 다른 주요 구간의 회귀가 없어야 한다.

현재 단일 warm-cache 측정값을 개발 기준선으로 사용한다. 결과가 경계에 있거나
시스템 부하 영향이 의심되는 경우, 또는 최종 병합·릴리즈 판단이 필요한 경우에만
반복 실행하여 median을 확인한다.

## 자동 비교

```bash
/root/workspace/build/open_lmm/test/open_lmm_artifact_compare \
  /root/workspace/output/baseline_c3f4912_test1_test2_2026_08_15 \
  /root/workspace/output/<candidate>
```

기본 실행은 pose만 비교하며 tolerance는 translation 1 mm, rotation 1 mrad다.
두 값은 명령 뒤에 같은 순서로 전달해 조정할 수 있다. PCD까지 확인할 때는
candidate 경로 뒤에 `--pcd`를 붙인다. 이때 이어지는 tolerance는 pose translation,
pose rotation, PCD 양방향 RMS, PCD 최대 거리, point 수 변화율 순서이며 기본값은
각각 1 mm, 1 mrad, 2 cm, 20 cm, 0.1%다.

```bash
/root/workspace/build/open_lmm/test/open_lmm_artifact_compare \
  /root/workspace/output/baseline_c3f4912_test1_test2_2026_08_15 \
  /root/workspace/output/<candidate> --pcd
```

선택한 검사 전체가 허용 범위 안이면 `overall=PASS`와 종료 코드 0을 반환한다.

계측 추가 직후 생성한 `2026_8_15_1_49_45` 결과는 공식 baseline과 agent A/B
모두 PASS했다. 전체 실행은 24.30초로 baseline보다 0.51초 느렸으며 단일 실행의
일반적인 변동 범위로 판단한다.

## 컴파일 경계 구조 개선 최종 검증

2026-08-15 GCC ICE 구조 개선 후보는 Clang 15 Release GUI OFF와 GCC 12.3
Release GUI ON 클린 빌드 및 양쪽 CTest 통과 후, 최종 GCC 산출물로 다음 세 번을
측정했다.

| 실행 | wall time |
|---|---:|
| 1 | 24.339 s |
| 2 | 24.171 s |
| 3 | 24.161 s |
| median | 24.171 s |

공식 baseline 대비 median은 +0.381초(+1.60%)이고 기존 계측 빌드 24.30초보다
0.129초 빠르다. 세 실행 모두 pose와 PCD 양방향 비교가 PASS했다. 비교한 산출물은
다음과 같다.

- `2026_8_15_15_31_55`
- `2026_8_15_15_32_19`
- `2026_8_15_15_32_44`

세 실행에서 관찰한 범위는 다음과 같다.

- Agent A pose translation RMS: `4.64713354e-05` ~ `7.4070618e-05 m`
- Agent A PCD RMS: `0.00134987653` ~ `0.00185348792 m`
- Agent A point count ratio: `3.04900359e-06` ~ `1.70744201e-05`
- Agent B pose translation RMS: `4.41090941e-05` ~ `8.59069849e-05 m`
- Agent B PCD RMS: `0.00121866061` ~ `0.0020117448 m`
- Agent B point count ratio: `3.33980699e-06` ~ `4.45307599e-06`

정확성 tolerance와 기존 단일 실행 변동 범위 안이므로 구조 개선의 정확성 및
성능 게이트를 PASS로 판정한다.

## registration 추가 분리 후 재검증

GCC 12의 `ira` 패스에서 추가 ICE가 확인되어 registration 수치 계산, spdlog
로깅과 PCL point-cloud 처리를 별도 번역 단위로 분리했다. 이 변경까지 포함한
최신 소스를 GCC 12.3 Release, ccache OFF, GUI ON으로 작업공간 루트에서 다시
클린 빌드한 뒤 다음 세 번을 측정했다.

| 실행 | output | wall time |
|---|---|---:|
| 1 | `2026_8_15_15_55_14` | 24.157 s |
| 2 | `2026_8_15_15_55_38` | 22.815 s |
| 3 | `2026_8_15_15_56_1` | 22.989 s |
| median | - | 22.989 s |

공식 baseline 23.79초 대비 median은 0.801초(3.37%) 빠르다. 세 실행 모두
`open_lmm_artifact_compare <baseline> <candidate> --pcd`에서 Agent A/B pose와
PCD 양방향 비교가 `overall=PASS`였다.

관찰된 최대 오차 범위는 다음과 같다.

- pose translation RMS: Agent A `9.48963367e-05 m`, Agent B
  `1.06219939e-04 m`
- pose rotation RMS: Agent A `1.31769715e-06 rad`, Agent B
  `1.56015551e-06 rad`
- PCD 양방향 RMS: Agent A `0.00202787096 m`, Agent B `0.00227773442 m`
- point count ratio: Agent A `1.52450179e-05`, Agent B `3.33980699e-06`

모든 값이 문서에 정의한 tolerance 이내이므로 registration 추가 분리를 포함한
최신 구조 개선은 정확성 및 성능 게이트를 PASS로 판정한다.
