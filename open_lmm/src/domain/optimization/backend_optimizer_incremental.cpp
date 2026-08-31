#include "backend_optimizer_incremental.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include <domain/support/registration.hpp>
#include <foundation/diagnostics/profiling.hpp>
#include <domain/support/algorithm_invariants.hpp>

#include "BetweenFactorWithAnchoring.h"

namespace open_lmm {
namespace {

gtsam::Symbol GraphSymbol(const AgentContext& context, const AgentId& agent,
                          uint64_t index) {
  if (!context.catalog) {
    throw std::invalid_argument("optimizer agent symbol catalog is missing");
  }
  auto symbol = context.catalog->SymbolFor(agent);
  if (!symbol) throw std::invalid_argument(symbol.GetError().Message());
  return gtsam::Symbol(symbol.Value().Byte(), index);
}

}  // namespace

BackendOptimizerIncremental::BackendOptimizerIncremental(OptimizerConfig config)
    : param_(std::move(config)), lifecycle_(std::make_unique<Lifecycle>()) {
  initNoise();
}

BackendOptimizerIncremental::~BackendOptimizerIncremental() {}

void BackendOptimizerIncremental::Reset() {
  std::lock_guard lock(execution_mutex_);
  lifecycle_ = std::make_unique<Lifecycle>();
}

bool BackendOptimizerIncremental::HasProcessedAgent(const AgentId& agent_id) const {
  std::lock_guard lock(execution_mutex_);
  return lifecycle_->processed_agents.contains(agent_id);
}

std::size_t BackendOptimizerIncremental::ProcessedAgentCount() const {
  std::lock_guard lock(execution_mutex_);
  return lifecycle_->processed_agents.size();
}

Result<BackendOptimizerOutput> BackendOptimizerIncremental::Process(
    const AlgorithmExecutionContext& context,
    const BackendOptimizerInput& input) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation = CheckAlgorithmCancellation(context, "before optimization");
  if (!cancellation) {
    return Result<BackendOptimizerOutput>::Failure(cancellation.GetError());
  }
  std::lock_guard execution(execution_mutex_);
  cancellation = CheckAlgorithmCancellation(context, "after optimizer lock");
  if (!cancellation) {
    return Result<BackendOptimizerOutput>::Failure(cancellation.GetError());
  }
  try {
    return Result<BackendOptimizerOutput>::Ok(
        processTransactional(context, input));
  } catch (const CancellationException& error) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::Cancelled(error.what()), context));
  } catch (const std::exception& error) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed(error.what()), context));
  } catch (...) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed("unknown optimizer exception"), context));
  }
}

BackendOptimizerOutput BackendOptimizerIncremental::processTransactional(
    const AlgorithmExecutionContext& context,
    const BackendOptimizerInput& input) {
  const auto& ctx = context.agent;
  const auto& raw_data = input.raw_data;
  const auto& intra_loops = input.intra_loops;
  const auto& inter_loops = input.inter_loops;
  const auto& all_raw_data = input.all_raw_data;
  OPEN_LMM_ZONE_N("Optimizer.Process");
  OPEN_LMM_PLOT("optimizer.intra_loop_count", intra_loops.size());
  OPEN_LMM_PLOT("optimizer.inter_loop_count", inter_loops.size());

  if (raw_data.agent_id != ctx.id) {
    throw std::invalid_argument("optimizer agent context/raw data ID mismatch");
  }
  auto valid_raw = ValidateAgentRawData(
      raw_data, "optimizer input agent '" + ctx.id.Value() + "'");
  if (!valid_raw) throw std::invalid_argument(valid_raw.GetError().Message());
  auto valid_loops = ValidateLoopPairs(
      raw_data, intra_loops, inter_loops, all_raw_data,
      "optimizer input agent '" + ctx.id.Value() + "'");
  if (!valid_loops) {
    throw std::invalid_argument(valid_loops.GetError().Message());
  }
  if (lifecycle_->processed_agents.contains(ctx.id)) {
    throw std::invalid_argument("optimizer agent " + ctx.id.Value() +
                                " was already processed; call Reset before retry");
  }
  if (lifecycle_->processed_agents.empty() && !ctx.is_anchor()) {
    throw std::invalid_argument("optimizer first agent must be the anchor");
  }
  if (!lifecycle_->processed_agents.empty() && ctx.is_anchor()) {
    throw std::invalid_argument("optimizer anchor can only be processed first");
  }
  for (const auto& loop : inter_loops) {
    if (!lifecycle_->processed_agents.contains(loop.to.first)) {
      throw std::invalid_argument(
          "inter-loop target agent must be optimized before the source agent");
    }
  }

  // Transactional working state: failures leave the committed lifecycle intact.
  auto working_graph = lifecycle_->graph;
  auto working_values = lifecycle_->values;
  auto working_processed_agents = lifecycle_->processed_agents;
  working_processed_agents.insert(ctx.id);

  gtsam::Pose3 anchor_node = gtsam::Pose3(Eigen::Matrix4d::Identity());
  const auto& anchor_prior = ctx.is_anchor() ? prior_noise_ : large_noise_;

  //! 1. anchor prior
  gtsam::Symbol anchor_symbol = GraphSymbol(ctx, ctx.id, ANCHOR_IDX);
  working_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
      anchor_symbol, anchor_node, anchor_prior));
  working_values.insert(anchor_symbol, anchor_node);

  //! 2. odometry
  for (size_t i = 0; i < raw_data.odom_poses.size(); i++) {
    ThrowIfCancellationRequested(context.cancellation, "optimizer odometry");
    gtsam::Symbol node_current = GraphSymbol(ctx, ctx.id, i);
    working_values.insert(node_current,
                               gtsam::Pose3(raw_data.odom_poses[i].matrix()));
    if (i == 0) {
      working_graph.add(gtsam::PriorFactor<gtsam::Pose3>(
          node_current, gtsam::Pose3(raw_data.odom_poses[i].matrix()), anchor_prior));
    } else {
      gtsam::Symbol node_prev = GraphSymbol(ctx, ctx.id, i - 1);
      Eigen::Isometry3d rel = raw_data.odom_poses[i - 1].inverse() * raw_data.odom_poses[i];
      working_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_prev, node_current, gtsam::Pose3(rel.matrix()), odometry_noise_));
    }
  }

  //! 3. intra-loop
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kRegisterIntraLoops,
                          0, intra_loops.size());
  auto T1 = tq::tqdm(intra_loops);
  T1.set_prefix("Intra Backend Optimizer");
  uint64_t intra_loop_current = 0;
  for (auto loop : T1) {
    ThrowIfCancellationRequested(context.cancellation, "optimizer intra loop");
    if (!FrameGapAtLeast(loop.from.second, loop.to.second,
                         static_cast<size_t>(param_.min_loop_frame_gap))) {
      ReportAlgorithmProgress(
          context, AlgorithmProgressPhase::kRegisterIntraLoops,
          ++intra_loop_current, intra_loops.size());
      continue;
    }
    gtsam::Symbol node_from = GraphSymbol(ctx, loop.from.first, loop.from.second);
    gtsam::Symbol node_to = GraphSymbol(ctx, loop.to.first, loop.to.second);
    auto refined = registerPointCloud(
        raw_data.filtered_scans, raw_data.odom_poses,
        raw_data.filtered_scans[loop.from.second], loop, param_.icp_search_num);
    if (refined) {
      working_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_from, node_to, gtsam::Pose3(refined.value().matrix()),
          robust_loop_noise_));
    }
    ReportAlgorithmProgress(context,
                            AlgorithmProgressPhase::kRegisterIntraLoops,
                            ++intra_loop_current, intra_loops.size());
  }
  T1.finish();

  //! 4. inter-loop (follower만)
  if (!ctx.is_anchor()) {
    std::size_t valid_inter_loop_count = 0;
    ReportAlgorithmProgress(
        context, AlgorithmProgressPhase::kRegisterInterLoops, 0,
        inter_loops.size());
    auto T2 = tq::tqdm(inter_loops);
    T2.set_prefix("Inter Backend Optimizer");
    uint64_t inter_loop_current = 0;
    for (auto loop : T2) {
      ThrowIfCancellationRequested(context.cancellation, "optimizer inter loop");
      gtsam::Symbol node_from = GraphSymbol(ctx, loop.from.first, loop.from.second);
      gtsam::Symbol node_to = GraphSymbol(ctx, loop.to.first, loop.to.second);
      const auto& raw_to = *all_raw_data.at(loop.to.first);
      auto refined = registerPointCloud(
          raw_to.filtered_scans, raw_to.odom_poses,
          raw_data.filtered_scans[loop.from.second], loop, param_.icp_search_num);
      if (refined) {
        // TODO(gil) : use BetweenFactorWithAnchoring?
        working_graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            node_from, node_to, gtsam::Pose3(refined.value().matrix()),
            robust_loop_noise_));
        ++valid_inter_loop_count;
      }
      ReportAlgorithmProgress(
          context, AlgorithmProgressPhase::kRegisterInterLoops,
          ++inter_loop_current, inter_loops.size());
    }
    T2.finish();
    if (valid_inter_loop_count == 0) {
      throw std::runtime_error(
          "all inter-agent loop registration refinements were rejected");
    }
  }

  //! 5. ISAM2 최적화 (누적된 전체 그래프로 배치 최적화)
  // 진짜 증분화는 BackendOptimizer를 MapServer 레벨에서 공유할 때 가능 (Step 05 이후)
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 0);
  {
    OPEN_LMM_ZONE_N("Optimizer.ISAM2.BatchUpdate");
  gtsam::ISAM2Params isam_param;
  isam_param.relinearizeThreshold = param_.relinearize_threshold;
  isam_param.relinearizeSkip      = param_.relinearize_skip;
  gtsam::ISAM2 isam_(isam_param);
  isam_.update(working_graph, working_values);
  for (int i = 0; i < param_.isam_extra_updates; ++i) {
    ThrowIfCancellationRequested(context.cancellation, "optimizer update");
    isam_.update();
  }
  working_values = isam_.calculateBestEstimate();
  }
  OPEN_LMM_PLOT("optimizer.factor_count", working_graph.size());
  OPEN_LMM_PLOT("optimizer.value_count", working_values.size());

  //! 6. 모든 에이전트 포즈 추출
  std::map<AgentId, std::vector<std::pair<int, Eigen::Isometry3d>>> all_poses;
  for (const auto& key_value : working_values) {
    gtsam::Key key = key_value.key;
    gtsam::Symbol symbol(key);
    if (symbol.index() == ANCHOR_IDX) continue;

    auto graph_symbol = AgentSymbol::FromByte(symbol.chr());
    if (!graph_symbol) {
      throw std::runtime_error(graph_symbol.GetError().Message());
    }
    if (!ctx.catalog) {
      throw std::runtime_error("optimizer agent symbol catalog is missing");
    }
    auto agent = ctx.catalog->AgentFor(graph_symbol.Value());
    if (!agent) throw std::runtime_error(agent.GetError().Message());
    gtsam::Symbol anchor_sym(symbol.chr(), ANCHOR_IDX);
    if (!working_values.exists(anchor_sym)) continue;

    Eigen::Matrix4d anchor_pose =
        working_values.at<gtsam::Pose3>(anchor_sym).matrix();
    Eigen::Matrix4d pose = working_values.at<gtsam::Pose3>(key).matrix();
    Eigen::Isometry3d global_pose(anchor_pose * pose);
    all_poses[agent.Value()].push_back(
        {static_cast<int>(symbol.index()), global_pose});
  }

  std::map<AgentId, AgentOptimizedData> all_results;
  for (auto& [agent, poses] : all_poses) {
    std::sort(poses.begin(), poses.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    AgentOptimizedData opt;
    opt.agent_id = agent;
    opt.optimized_poses = std::move(poses);
    for (const auto& [idx, p] : opt.optimized_poses) {
      opt.kdtree_poses.push_back(pcl::PointXYZ(p.translation().x(),
                                                p.translation().y(),
                                                p.translation().z()));
    }
    all_results[agent] = std::move(opt);
  }
  for (const auto& [agent, optimized] : all_results) {
    const AgentRawData* corresponding_raw = nullptr;
    if (agent == raw_data.agent_id) {
      corresponding_raw = &raw_data;
    } else {
      const auto found = all_raw_data.find(agent);
      if (found != all_raw_data.end() && found->second) {
        corresponding_raw = found->second.get();
      }
    }
    if (!corresponding_raw) {
      throw std::invalid_argument(
          "optimizer output references raw data for unknown agent '" +
          agent.Value() + "'");
    }
    auto valid = ValidateOptimizedData(
        optimized, *corresponding_raw,
        "optimizer output agent '" + agent.Value() + "'");
    if (!valid) throw std::invalid_argument(valid.GetError().Message());
  }
  ThrowIfCancellationRequested(context.cancellation, "optimizer commit");
  // Construct the complete next lifecycle before publication. Pointer swap is
  // non-throwing, so no failure can expose a partially updated optimizer.
  auto next = std::make_unique<Lifecycle>();
  next->graph = std::move(working_graph);
  next->values = std::move(working_values);
  next->processed_agents = std::move(working_processed_agents);
  lifecycle_.swap(next);
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 1, 1);
  return all_results;
}

// TODO(gil) : add noise parameter config and parsing function
void BackendOptimizerIncremental::initNoise() {
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12)
          .finished());

  odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::Vector robust_noise_vec(6);
  robust_noise_vec << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
  robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(robust_noise_vec));

  large_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e8, 1e8, 1e8)
          .finished());
}

}  // namespace open_lmm
