# cmake/options.cmake — 모든 빌드 옵션을 이 파일에 집중

# ccache
option(USE_CCACHE "Build using Ccache if found on the path" ON)

# 시스템 의존성 옵션 — ON: 시스템 설치본 사용, OFF: FetchContent 자동 다운로드
option(USE_SYSTEM_EIGEN3      "Use system pre-installed Eigen"         ON)
option(USE_SYSTEM_TBB         "Use system pre-installed oneAPI/tbb"    ON)
option(USE_SYSTEM_PCL         "Use system pre-installed PCL"           ON)
option(USE_SYSTEM_GTSAM       "Use system pre-installed GTSAM"         ON)   # tested on v4.2a9
option(USE_SYSTEM_NANOFLANN   "Use system pre-installed nanoflann"     OFF)  # fetched v1.5.5
option(USE_SYSTEM_TQDMCPP     "Use system pre-installed tqdmcpp"       OFF)  # fetched custom
option(USE_SYSTEM_SMALL_GICP  "Use system pre-installed small_gicp"    OFF)
option(USE_SYSTEM_KISS_MATCHER "Use system pre-installed kiss_matcher"  OFF)
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
