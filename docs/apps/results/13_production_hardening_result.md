# H1 Runtime·ROS·Plugin Production Hardening 결과

- 판정: **IMPLEMENTED / GO**
- 기준선: `dev/reorder` / `fa0c8e4`
- 검증일: 2026-08-28 UTC
- toolchain: GCC/G++ 12.3.0, ROS 2 Humble, CPython 3.10

## 1. 단계별 커밋

| 단계 | 커밋 | 결과 |
|---|---|---|
| H1-1 thread creation safety | `140e028b24ac789d882abc293499212d84c554a6` | 두 worker publication 전 rollback 및 `kResourceExhausted` 매핑 |
| H1-2 RuntimeClient retirement | `583c0c3172cae77284727bb9a44bc194c63655a9` | detached cleanup 제거, bounded joinable coordinator 도입 |
| H1-3 ROS early-cancel | `15db95c134ef650b2203329612b6b5de1f72deab` | exact Goal UUID 기반 accept→publish cancellation 보존 |
| H1-4 plugin generation | `2050a10a59ad4443827f5b590ff2f8556ee68ceb` | capability/name/schema/build generation을 create 전에 exact 검증 |
| package gate follow-up | `9c56652` | 새 public error golden 갱신과 이미 이동된 experiment CLI의 stale test 등록 제거 |

## 2. Fault와 ownership 증거

- Pipeline의 첫 번째·두 번째 thread 생성 실패에서 job ID, terminal snapshot,
  event sequence와 feedback state가 공개·변경되지 않았고 같은 controller의 다음 submit이
  성공했다.
- BoundedExecutor의 첫·중간·마지막 worker 생성 실패에서 이미 생성된 worker가 모두
  stop/join됐고 원래 `std::system_error`가 전파됐다.
- Runtime retirement queue는 `Impl`당 intrusive node 하나만 허용한다. 다중 retirement
  후 `pending == 0`, 완료 수 일치와 non-zero peak를 검사했으며 process 종료 시 drain 후
  worker를 join했다.
- terminal callback에서 마지막 C++ 및 Python Runtime owner 파괴와 callback 내부 move
  assignment가 detached thread나 GIL 보유 cleanup 없이 완료됐다.

## 3. ROS terminal matrix

- accept→submit 및 submit→job-publication 두 gap에서 같은 Goal UUID의 cancel을 수락하고
  pending cancel을 exact job에 한 번만 전달한다.
- 다른 UUID, 이전 goal과 repeated cancel은 현재 runtime job으로 전달되지 않는다.
- submit 전 accepted cancel은 `CANCELED`, 일반 submit 실패는 `ABORTED`다.
- runtime commit receipt가 먼저 성립한 late cancel은 기존 resolver에 따라
  `SUCCEEDED`가 우선한다.
- ROS CTest 5/5와 실제 action graph 회귀가 통과했다.

## 4. Plugin contract 증거

- Descriptor 2개와 remover 5개, 총 7개 built-in DSO가 package generation
  `open-lmm-3.0.0`을 보고하고 create/destroy 계약을 통과했다.
- stale capability/name/schema/build 및 null/empty metadata fixture는 모두 create count
  `0`에서 `kPluginLoadFailed`로 거부됐다.
- generated generation 상수는 DSO-local linkage를 사용한다. GNU-unique symbol 때문에
  DSO가 `NODELETE`가 되는 회귀를 plugin stress가 탐지했으며 수정 후 100회 lifetime
  stress와 전체 core suite가 통과했다.

## 5. 검증 결과

| 검증 | 결과 |
|---|---|
| Release core build + full CTest | **76/76 PASS** |
| H1 targeted core contracts | **PASS** |
| ROS CTest + graph cancellation matrix | **5/5 PASS** |
| Python wheel build/install + API/lifetime | **24/24 PASS**; source-free package fixture **18/18 PASS** |
| ASan+UBSan core manifest | **70/70 PASS** |
| TSan H1 targeted lane | **7/7 PASS** |
| architecture/module compile gates | **PASS** (full CTest 포함) |
| installed-prefix/source-free checks | core C/C++ consumer와 24개 public-header self-containment **PASS** |

공식 ASan 스크립트의 H1 core lane은 모두 통과했다. 이어 실행된 별도 GUI plugin
18,000-cycle RSS gate는 DSO/FD balance가 모두 0임에도 ASan allocator RSS slope가 기존
threshold를 넘어서 실패했다. 이는 H1 algorithm plugin 또는 runtime ownership 실패로
판정하지 않으며 GUI sanitizer RSS calibration의 별도 잔여 증거로 보존한다.

## 6. Raw evidence

- ASan log: `build/h1/sanitizers/h1-asan/ci.log`
  - SHA-256 `c514635c49c2fe786e3227d5caba73158aec6dd21fab5ebfe0e1e59c5667d9de`
- ASan CTest XML: `build/h1/sanitizers/h1-asan/ctest.xml`
  - SHA-256 `0b6e0286e327722d98ec1e00f3668d0d3fd33221683ab12f4be615c0dd661ae8`
- final core CTest XML: `build/h1/core-final.xml`
  - SHA-256 `69e9641fe970cca7eac5603ea3812e8bf29ca9512bf9542312daba4defdc1dc4`
- targeted TSan CTest XML: `build/h1/tsan/build/h1/tsan/h1-targeted.xml`
  - SHA-256 `8da9a898447ef07e65ec27e4ea5e4f36629be08b245405e9ed65077e7440125b`
- ROS CTest XML: `build/h1/ros-final.xml`
  - SHA-256 `35916919c4c09d1f61d63fc10904b0136c503adbcedd78593aa6f96bd2c76da0`
- Python wheel: `build/dev/python-wheel/open_lmm-3.0.0-cp310-cp310-linux_x86_64.whl`
  - SHA-256 `2c6484282f68b2f2c955df2d3c074be631b58432fda7c90e1e8256b7832d679d`
- source-free install component manifest SHA-256:
  `87190410ed998f4f14f1d07088ad72c3fc7995e119cea043e50f591412fe2cef`

## 7. 잔여 범위

H1 완료는 GUI alignment authority, mutable-config trusted root, P7-C2/C3, Goal 09
공급망, crash durability, 최소 CMake 버전 또는 real GPU/driver evidence 완료를 의미하지
않는다. 이 항목은 기존 독립 admission과 owner를 유지한다.
