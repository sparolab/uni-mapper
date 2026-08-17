# cmake/externals.cmake — 외부 의존성 탐색/페칭
# 실제 버전 고정 및 FetchContent 로직은 thirdparty/ 서브파일에 위임.

if(CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif()

# thirdparty/find_dependencies.cmake 에 정의된 find_external_dependency() 함수와
# 각 의존성별 cmake 파일 사용.
# 이 구조를 유지하는 이유: thirdparty/ 안에 의존성별 세부 cmake 파일이 있어
# 외부 소스는 건드리지 않고 CMake include 경로만 교체.
include(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/find_dependencies.cmake)
