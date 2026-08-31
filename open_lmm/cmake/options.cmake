# cmake/options.cmake — 모든 빌드 옵션을 이 파일에 집중

# ccache
option(USE_CCACHE "Build using Ccache if found on the path" ON)

# 시스템 의존성 옵션 — ON: 시스템 설치본 사용, OFF: FetchContent 자동 다운로드
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(USE_SYSTEM_PCL "Use system pre-installed PCL" ON)
option(USE_SYSTEM_GTSAM "Use system pre-installed GTSAM" ON) # tested on v4.2a9
option(USE_SYSTEM_NANOFLANN "Use system pre-installed nanoflann" OFF) # fetched v1.5.5
option(USE_SYSTEM_TQDMCPP "Use system pre-installed tqdmcpp" OFF) # fetched custom
option(USE_SYSTEM_SMALL_GICP "Use system pre-installed small_gicp" OFF)
option(USE_SYSTEM_KISS_MATCHER "Use system pre-installed kiss_matcher" OFF)
option(USE_SYSTEM_NLOHMANN_JSON "Use system pre-installed nlohmann_json" OFF)

# ccache 설정
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)

  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# 선택적 로컬 타임라인 프로파일링. 기본 빌드는 Tracy를 fetch/link하지 않는다.
option(OPEN_LMM_ENABLE_TRACY "Enable Tracy timeline profiling" OFF)
option(OPEN_LMM_ENABLE_TIMING_LOG "Enable lightweight [PROFILE] timing logs" ON)
option(OPEN_LMM_ENABLE_TSAN
  "Instrument OpenLMM and focused concurrency tests with ThreadSanitizer" OFF)
option(OPEN_LMM_ENABLE_ASAN_UBSAN
  "Instrument OpenLMM with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)
# Opt-in engineering-quality lanes.  Normal C++/ROS/package builds do not
# discover or require these tools.  CI enables each lane in a clean dedicated
# build so sanitizer, coverage and fuzz runtimes are never mixed accidentally.
option(OPEN_LMM_ENABLE_CLANG_TIDY
  "Run pinned clang-tidy on OpenLMM-owned C++ targets" OFF)
option(OPEN_LMM_ENABLE_STRICT_WARNINGS
  "Enable the reviewed first-party warnings profile" OFF)
option(OPEN_LMM_ENABLE_COVERAGE
  "Enable Clang source-based coverage instrumentation" OFF)
option(OPEN_LMM_ENABLE_FUZZING
  "Build Clang libFuzzer targets" OFF)

if(OPEN_LMM_ENABLE_TSAN AND OPEN_LMM_ENABLE_ASAN_UBSAN)
  message(FATAL_ERROR
    "OPEN_LMM_ENABLE_TSAN and OPEN_LMM_ENABLE_ASAN_UBSAN are mutually exclusive")
endif()

if(OPEN_LMM_ENABLE_COVERAGE AND
   (OPEN_LMM_ENABLE_TSAN OR OPEN_LMM_ENABLE_ASAN_UBSAN OR
    OPEN_LMM_ENABLE_FUZZING))
  message(FATAL_ERROR
    "OPEN_LMM_ENABLE_COVERAGE is mutually exclusive with sanitizer and fuzz builds")
endif()

if(OPEN_LMM_ENABLE_FUZZING AND
   (OPEN_LMM_ENABLE_TSAN OR OPEN_LMM_ENABLE_ASAN_UBSAN))
  message(FATAL_ERROR
    "OPEN_LMM_ENABLE_FUZZING owns its ASan/UBSan runtime and cannot be combined with sanitizer options")
endif()

if((OPEN_LMM_ENABLE_COVERAGE OR OPEN_LMM_ENABLE_FUZZING) AND
   NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  message(FATAL_ERROR
    "OPEN_LMM_ENABLE_COVERAGE and OPEN_LMM_ENABLE_FUZZING require Clang")
endif()

# Runtime plugin target selection. A disabled built-in is not configured,
# compiled, installed, or advertised as available by the typed config parser.
option(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT
  "Build the ScanContext descriptor plugin" ON)
option(OPEN_LMM_BUILD_DESCRIPTOR_SOLID
  "Build the SOLiD descriptor plugin" ON)
option(OPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS
  "Build the HMM-MOS online dynamic-remover plugin" ON)
option(OPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP
  "Build the DUFOMap online dynamic-remover plugin" ON)
option(OPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD
  "Build the OTD online dynamic-remover plugin" ON)
option(OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM
  "Build the FreeDOM offline dynamic-remover plugin" ON)
option(OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR
  "Build the ERASOR offline dynamic-remover plugin" ON)
