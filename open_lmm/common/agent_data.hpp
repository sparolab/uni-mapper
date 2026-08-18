#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <memory>
#include <optional>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/common/descriptor_index.hpp>
#include <vector>

namespace open_lmm {

// DataLoader의 출력 — DataLoader만 생성, 이후 읽기 전용
struct AgentRawData {
  AgentId agent_id;
  PoseVec odom_poses;
  ScanVec filtered_scans;
  std::vector<Eigen::Vector3f> map_points;  // 2m voxel 다운샘플 맵 (KISSMatcher용)
};

// BackendOptimizer의 출력
struct AgentOptimizedData {
  AgentId agent_id;
  std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses;
  pcl::PointCloud<pcl::PointXYZ>                 kdtree_poses;  // 포즈 KD-tree 검색용
};

using AgentRawDataHandle = std::shared_ptr<const AgentRawData>;
using AgentOptimizedDataHandle = std::shared_ptr<const AgentOptimizedData>;
using AgentRawDataMap = std::map<AgentId, AgentRawDataHandle>;
using AgentOptimizedDataMap = std::map<AgentId, AgentOptimizedDataHandle>;

// LoopDetector의 출력 계약. 구체 detector 구현과 분리해 pipeline 사용자가
// KISS-Matcher 구현 헤더를 포함하지 않도록 한다.
struct LoopDetectorOutput {
  LoopPairVec intra_loops;
  LoopPairVec inter_loops;
  DescriptorIndexHandle agent_descriptors;
  std::vector<Eigen::Vector3f> transformed_map_points;
  std::optional<Eigen::Isometry3d> accepted_global_T_agent;
  std::optional<AlignmentMethod> accepted_alignment_method;
  std::optional<AlignmentApproval> accepted_alignment_approval;
  AgentId accepted_target_agent;
  uint64_t accepted_at_unix_ms = 0;
  AlignmentMetrics accepted_alignment_metrics;
};

struct AlignedAgentMap {
  AgentId agent_id;
  std::vector<Eigen::Vector3f> original_map;
  Eigen::Isometry3d global_T_agent = Eigen::Isometry3d::Identity();
  AlignmentMethod accepted_method = AlignmentMethod::kKissMatcher;
  AlignmentApproval approval = AlignmentApproval::kAutomatic;
  AgentId target_agent;
  uint64_t accepted_at_unix_ms = 0;
  uint64_t revision = 0;
};

// LoopDetector 전용 공유 상태 (에이전트 간 순차 누적)
struct DescriptorStore {
  DescriptorIndexHandle total_db;  // anchor + follower descriptor index snapshot
  std::map<AgentId, DescriptorIndexHandle> per_agent_db;
  std::vector<AgentId> descriptor_order;
  std::vector<Eigen::Vector3f> merged_map;  // KISSMatcher용 누적 맵 포인트
  std::map<AgentId, AlignedAgentMap> aligned_maps;

  ~DescriptorStore() {
    clear();
  }

  void clear() {
    total_db.reset();
    per_agent_db.clear();
    descriptor_order.clear();
    merged_map.clear();
    aligned_maps.clear();
  }

  // Descriptor compute 결과는 optimization 이전에 commit할 수 있지만,
  // map target은 OptimizeNode 성공 이후에만 set_agent_map()으로 commit한다.
  void set_anchor_descriptor(AgentId agent_id, DescriptorIndexHandle index) {
    per_agent_db.clear();
    descriptor_order.clear();
    per_agent_db.emplace(agent_id, std::move(index));
    descriptor_order.push_back(agent_id);
    rebuild_descriptor_db();
  }

  // follower 처리 후 호출 — 다음 에이전트가 이 follower의 descriptor를 조회 가능
  void merge_descriptor_db(AgentId agent_id, DescriptorIndexHandle index) {
    if (!per_agent_db.contains(agent_id)) descriptor_order.push_back(agent_id);
    per_agent_db[agent_id] = std::move(index);
    rebuild_descriptor_db();
  }

  void rebuild_descriptor_db() {
    std::unique_ptr<DescriptorIndex> rebuilt;
    for (const AgentId& agent_id : descriptor_order) {
      const auto found = per_agent_db.find(agent_id);
      if (found == per_agent_db.end() || !found->second) continue;
      if (!rebuilt) {
        rebuilt = found->second->Clone();
      } else {
        rebuilt->merge(*found->second);
      }
    }
    total_db = DescriptorIndexHandle(std::move(rebuilt));
  }

  void set_agent_map(AgentId agent_id, std::vector<Eigen::Vector3f> original_map,
                     const Eigen::Isometry3d& global_T_agent,
                     AlignmentMethod method, AlignmentApproval approval,
                     AgentId target_agent, uint64_t accepted_at_unix_ms) {
    auto& state = aligned_maps[agent_id];
    state.agent_id = agent_id;
    state.original_map = std::move(original_map);
    state.global_T_agent = global_T_agent;
    state.accepted_method = method;
    state.approval = approval;
    state.target_agent = target_agent;
    state.accepted_at_unix_ms = accepted_at_unix_ms;
    ++state.revision;
    rebuild_merged_map();
  }

  void update_transform(const AgentId& agent_id,
                        const Eigen::Isometry3d& global_T_agent) {
    auto found = aligned_maps.find(agent_id);
    if (found == aligned_maps.end()) return;
    found->second.global_T_agent = global_T_agent;
    ++found->second.revision;
    rebuild_merged_map();
  }

  void update_transforms(
      const std::map<AgentId, Eigen::Isometry3d>& global_transforms) {
    bool changed = false;
    for (const auto& [agent_id, transform] : global_transforms) {
      auto found = aligned_maps.find(agent_id);
      if (found == aligned_maps.end()) continue;
      found->second.global_T_agent = transform;
      ++found->second.revision;
      changed = true;
    }
    if (changed) rebuild_merged_map();
  }

  void rebuild_merged_map() {
    std::size_t point_count = 0;
    for (const auto& [id, state] : aligned_maps) {
      (void)id;
      point_count += state.original_map.size();
    }
    merged_map.clear();
    merged_map.reserve(point_count);
    for (const auto& [id, state] : aligned_maps) {
      (void)id;
      const Eigen::Matrix4f transform = state.global_T_agent.matrix().cast<float>();
      for (const auto& point : state.original_map) {
        merged_map.push_back((transform * point.homogeneous()).head<3>());
      }
    }
  }
};

// GraphStore 제거 — ISAM2가 BackendOptimizerIncremental 멤버로 상태를 관리

}  // namespace open_lmm
