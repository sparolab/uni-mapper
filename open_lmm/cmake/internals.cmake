# cmake/internals.cmake — 내부 타겟 공통 컴파일 옵션 및 헬퍼 함수
# 실제 구현은 cmake/CompilerOptions.cmake에 유지 (MIT license 보존).
include(${CMAKE_CURRENT_LIST_DIR}/QualityOptions.cmake)
include(${CMAKE_CURRENT_LIST_DIR}/CompilerOptions.cmake)

# PRIVATE/PUBLIC 링크 속성 원칙:
# PUBLIC  : 이 라이브러리 헤더에 타입이 노출되는 의존성
#           (Eigen — AgentRawData에 Eigen::Isometry3d 포함)
#           (PCL   — ScanVec = vector<pcl::PointCloud<XYZI>::Ptr> 헤더에 노출)
#           (GTSAM — AgentOptimizedData 등)
# PRIVATE : .cpp 내부에서만 사용
#           (nanoflann, small_gicp, kiss_matcher, nlohmann_json 등)
