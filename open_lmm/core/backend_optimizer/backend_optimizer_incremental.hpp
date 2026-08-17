#pragma once
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include "backend_optimizer_base.hpp"

namespace open_lmm {

struct BackendOptimizerIncrementalParam {
  std::string backend_optimizer_type;
  double relinearize_threshold;
  int    relinearize_skip;
  int    isam_extra_updates;   // ISAM2 추가 재선형화 횟수 (기본 5)
  int    min_loop_frame_gap;   // intra-loop 최소 스캔 인덱스 차이 (기본 30)
  int    icp_search_num;       // registerPointCloud submap 검색 범위 (기본 3)
};

class BackendOptimizerIncremental : public BackendOptimizerBase {
 public:
  explicit BackendOptimizerIncremental(Config config);
  ~BackendOptimizerIncremental() override;
  void parseConfig(Config config) override;

  // GraphStore 제거 — 누적 graph/values를 클래스 멤버로 관리
  // (MapAligner가 에이전트마다 BackendOptimizer를 새로 생성하므로
  //  ISAM2 객체를 클래스 멤버로 공유하는 대신 accumulated graph를 멤버로 유지)
  std::map<char, AgentOptimizedData> Process(
      const AgentContext&                 ctx,
      const AgentRawData&                 raw_data,
      const LoopPairVec&                  intra_loops,
      const LoopPairVec&                  inter_loops,
      const std::map<char, AgentRawData>& all_raw_data) override;

  void initNoise();

 private:
  BackendOptimizerIncrementalParam param_;

  // 에이전트 간 공유 팩터 그래프 — 클래스 멤버로 누적 (GraphStore 대체)
  gtsam::NonlinearFactorGraph accumulated_graph_;
  gtsam::Values               accumulated_values_;

  gtsam::noiseModel::Diagonal::shared_ptr prior_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr odometry_noise_;
  gtsam::noiseModel::Base::shared_ptr     robust_loop_noise_;
  gtsam::noiseModel::Diagonal::shared_ptr large_noise_;
};

}  // namespace open_lmm