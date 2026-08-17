#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>
#include <vector>

namespace open_lmm {

// DataLoader의 출력 — DataLoader만 생성, 이후 읽기 전용
struct AgentRawData {
  char    agent_id{};
  PoseVec odom_poses;
  ScanVec filtered_scans;
  std::vector<Eigen::Vector3f> map_points;  // 2m voxel 다운샘플 맵 (KISSMatcher용)
};

// BackendOptimizer의 출력
struct AgentOptimizedData {
  char agent_id{};
  std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses;
  pcl::PointCloud<pcl::PointXYZ>                 kdtree_poses;  // 포즈 KD-tree 검색용
};

// LoopDetector의 출력 계약. 구체 detector 구현과 분리해 pipeline 사용자가
// KISS-Matcher 구현 헤더를 포함하지 않도록 한다.
struct LoopDetectorOutput {
  LoopPairVec intra_loops;
  LoopPairVec inter_loops;
  DatabaseKdtree agent_db;
  std::vector<Eigen::Vector3f> transformed_map_points;
};

// LoopDetector 전용 공유 상태 (에이전트 간 순차 누적)
struct DescriptorStore {
  DatabaseKdtree               total_db;    // anchor + follower descriptor DB 누적
  std::vector<Eigen::Vector3f> merged_map;  // KISSMatcher용 누적 맵 포인트

  ~DescriptorStore() {
    total_db.clear();
    merged_map.clear();
  }

  // anchor 에이전트 처리 후 호출
  void set_anchor(DatabaseKdtree&& db, std::vector<Eigen::Vector3f> map) {
    total_db   = std::move(db);
    merged_map = std::move(map);
  }

  // follower 처리 후 호출 — 다음 에이전트가 이 follower의 descriptor를 조회 가능
  void merge_descriptor_db(DatabaseKdtree&& follower_db) {
    total_db.merge(std::move(follower_db));
  }

  // follower의 KISSMatcher 결과 포인트 누적
  void merge_map(const std::vector<Eigen::Vector3f>& transformed_points) {
    merged_map.insert(merged_map.end(),
                      transformed_points.begin(), transformed_points.end());
  }
};

// GraphStore 제거 — ISAM2가 BackendOptimizerIncremental 멤버로 상태를 관리

}  // namespace open_lmm
