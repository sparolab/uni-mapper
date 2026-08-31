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

bool ContainsAgent(const std::vector<AgentId>& agents, const AgentId& agent) {
  return std::find(agents.begin(), agents.end(), agent) != agents.end();
}

}  // namespace

BackendOptimizerIncremental::BackendOptimizerIncremental(OptimizerConfig config)
    : param_(std::move(config)),
      lifecycle_(std::make_unique<Lifecycle>(IsamParams())) {
  lifecycle_->diagnostics.solver_constructions = 1;
  initNoise();
}

BackendOptimizerIncremental::~BackendOptimizerIncremental() {}

void BackendOptimizerIncremental::Reset() {
  std::lock_guard lock(execution_mutex_);
  lifecycle_ = std::make_unique<Lifecycle>(IsamParams());
  lifecycle_->diagnostics.solver_constructions = 1;
}

bool BackendOptimizerIncremental::HasProcessedAgent(const AgentId& agent_id) const {
  std::lock_guard lock(execution_mutex_);
  return ContainsAgent(lifecycle_->processed_agents, agent_id);
}

std::size_t BackendOptimizerIncremental::ProcessedAgentCount() const {
  std::lock_guard lock(execution_mutex_);
  return lifecycle_->processed_agents.size();
}

bool BackendOptimizerIncremental::IsUsable() const {
  std::lock_guard lock(execution_mutex_);
  return !lifecycle_->poisoned;
}

BackendOptimizerIncremental::Diagnostics
BackendOptimizerIncremental::GetDiagnostics() const {
  std::lock_guard lock(execution_mutex_);
  Diagnostics diagnostics = lifecycle_->diagnostics;
  diagnostics.factor_count = lifecycle_->solver.getFactorsUnsafe().nrFactors();
  diagnostics.value_count = lifecycle_->solver.getLinearizationPoint().size();
  diagnostics.poisoned = lifecycle_->poisoned;
  return diagnostics;
}

Result<std::shared_ptr<BackendOptimizerBase>>
BackendOptimizerIncremental::ForkCandidate() const {
  std::lock_guard lock(execution_mutex_);
  if (lifecycle_->poisoned) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        Error::OptimizationFailed("cannot fork an unusable optimizer"));
  }
  try {
    auto candidate = std::make_shared<BackendOptimizerIncremental>(param_);
    candidate->lifecycle_ = std::make_unique<Lifecycle>(*lifecycle_);
    ++candidate->lifecycle_->diagnostics.solver_constructions;
    ++candidate->lifecycle_->diagnostics.candidate_forks;
    return Result<std::shared_ptr<BackendOptimizerBase>>::Ok(
        std::move(candidate));
  } catch (const std::exception& error) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        Error::OptimizationFailed(std::string("optimizer fork failed: ") +
                                  error.what()));
  } catch (...) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        Error::OptimizationFailed("optimizer fork failed"));
  }
}

Result<BackendOptimizerOutput> BackendOptimizerIncremental::OptimizePrefix(
    const AlgorithmExecutionContext& context,
    const std::vector<AgentId>& retained_agents,
    const AgentRawDataMap& all_raw_data) {
  AlgorithmExecutionTimer timer(context);
  auto cancellation =
      CheckAlgorithmCancellation(context, "before optimizer prefix refinement");
  if (!cancellation) {
    return Result<BackendOptimizerOutput>::Failure(cancellation.GetError());
  }
  std::lock_guard lock(execution_mutex_);
  if (lifecycle_->poisoned) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed("optimizer candidate is unusable"), context));
  }
  if (retained_agents.empty() ||
      retained_agents.size() > lifecycle_->processed_agents.size() ||
      !std::equal(retained_agents.begin(), retained_agents.end(),
                  lifecycle_->processed_agents.begin())) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::InvalidArgument(
            "optimizer retained agents must be an exact processed prefix"),
        context));
  }

  gtsam::FactorIndices removals;
  gtsam::KeyVector removed_keys;
  for (std::size_t index = retained_agents.size();
       index < lifecycle_->processed_agents.size(); ++index) {
    const AgentId& agent = lifecycle_->processed_agents[index];
    const auto factors = lifecycle_->factor_indices.find(agent);
    const auto keys = lifecycle_->agent_keys.find(agent);
    if (factors == lifecycle_->factor_indices.end() ||
        keys == lifecycle_->agent_keys.end()) {
      return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
          Error::OptimizationFailed("optimizer factor ledger is incomplete"),
          context));
    }
    removals.insert(removals.end(), factors->second.begin(),
                    factors->second.end());
    removed_keys.insert(removed_keys.end(), keys->second.begin(),
                        keys->second.end());
  }
  std::sort(removals.begin(), removals.end());
  removals.erase(std::unique(removals.begin(), removals.end()), removals.end());

  bool mutation_started = false;
  try {
    ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 0);
    if (!removals.empty()) {
      mutation_started = true;
      const auto update = lifecycle_->solver.update(
          gtsam::NonlinearFactorGraph{}, gtsam::Values{}, removals);
      lifecycle_->diagnostics.variables_relinearized +=
          update.variablesRelinearized;
      lifecycle_->diagnostics.variables_reeliminated +=
          update.variablesReeliminated;
    }
    for (int i = 0; i < param_.isam_extra_updates; ++i) {
      ThrowIfCancellationRequested(context.cancellation,
                                   "optimizer prefix update");
      mutation_started = true;
      const auto update = lifecycle_->solver.update();
      lifecycle_->diagnostics.variables_relinearized +=
          update.variablesRelinearized;
      lifecycle_->diagnostics.variables_reeliminated +=
          update.variablesReeliminated;
    }
    const auto values = lifecycle_->solver.calculateBestEstimate();
    for (gtsam::Key key : removed_keys) {
      if (lifecycle_->solver.valueExists(key)) {
        throw std::runtime_error(
            "optimizer suffix variable survived factor removal");
      }
    }
    auto output = BuildOutput(context, values, all_raw_data, nullptr);
    ThrowIfCancellationRequested(context.cancellation,
                                 "optimizer prefix commit");
    for (std::size_t index = retained_agents.size();
         index < lifecycle_->processed_agents.size(); ++index) {
      const AgentId& agent = lifecycle_->processed_agents[index];
      lifecycle_->factor_indices.erase(agent);
      lifecycle_->agent_keys.erase(agent);
    }
    lifecycle_->processed_agents.resize(retained_agents.size());
    lifecycle_->diagnostics.removed_factors += removals.size();
    ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 1, 1);
    return Result<BackendOptimizerOutput>::Ok(std::move(output));
  } catch (const CancellationException& error) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::Cancelled(error.what()), context));
  } catch (const std::exception& error) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed(error.what()), context));
  } catch (...) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed("unknown optimizer prefix exception"),
        context));
  }
}

gtsam::ISAM2Params BackendOptimizerIncremental::IsamParams() const {
  gtsam::ISAM2Params params;
  params.relinearizeThreshold = param_.relinearize_threshold;
  params.relinearizeSkip = param_.relinearize_skip;
  return params;
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
  if (lifecycle_->poisoned) {
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed(
            "optimizer candidate is unusable after a mutating failure"),
        context));
  }
  cancellation = CheckAlgorithmCancellation(context, "after optimizer lock");
  if (!cancellation) {
    return Result<BackendOptimizerOutput>::Failure(cancellation.GetError());
  }
  bool mutation_started = false;
  try {
    return Result<BackendOptimizerOutput>::Ok(
        processTransactional(context, input, mutation_started));
  } catch (const CancellationException& error) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::Cancelled(error.what()), context));
  } catch (const std::exception& error) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed(error.what()), context));
  } catch (...) {
    if (mutation_started) lifecycle_->poisoned = true;
    return Result<BackendOptimizerOutput>::Failure(WithAlgorithmContext(
        Error::OptimizationFailed("unknown optimizer exception"), context));
  }
}

BackendOptimizerOutput BackendOptimizerIncremental::processTransactional(
    const AlgorithmExecutionContext& context,
    const BackendOptimizerInput& input, bool& mutation_started) {
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
  if (ContainsAgent(lifecycle_->processed_agents, ctx.id)) {
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
    if (!ContainsAgent(lifecycle_->processed_agents, loop.to.first)) {
      throw std::invalid_argument(
          "inter-loop target agent must be optimized before the source agent");
    }
  }

  // Prepare every fallible registration result before mutating the
  // command-private solver. Only this agent's graph delta is submitted.
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  gtsam::KeyVector new_keys;

  gtsam::Pose3 anchor_node = gtsam::Pose3(Eigen::Matrix4d::Identity());
  const auto& anchor_prior = ctx.is_anchor() ? prior_noise_ : large_noise_;

  //! 1. anchor prior
  gtsam::Symbol anchor_symbol = GraphSymbol(ctx, ctx.id, ANCHOR_IDX);
  new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
      anchor_symbol, anchor_node, anchor_prior));
  new_values.insert(anchor_symbol, anchor_node);
  new_keys.push_back(anchor_symbol);

  //! 2. odometry
  for (size_t i = 0; i < raw_data.odom_poses.size(); i++) {
    ThrowIfCancellationRequested(context.cancellation, "optimizer odometry");
    gtsam::Symbol node_current = GraphSymbol(ctx, ctx.id, i);
    new_values.insert(node_current,
                      gtsam::Pose3(raw_data.odom_poses[i].matrix()));
    new_keys.push_back(node_current);
    if (i == 0) {
      new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(
          node_current, gtsam::Pose3(raw_data.odom_poses[i].matrix()), anchor_prior));
    } else {
      gtsam::Symbol node_prev = GraphSymbol(ctx, ctx.id, i - 1);
      Eigen::Isometry3d rel = raw_data.odom_poses[i - 1].inverse() * raw_data.odom_poses[i];
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
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
      new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
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
        new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
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

  //! 5. Persistent ISAM2 update with this agent's graph delta.
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 0);
  gtsam::Values working_values;
  gtsam::FactorIndices added_factor_indices;
  {
    OPEN_LMM_ZONE_N("Optimizer.ISAM2.IncrementalUpdate");
    mutation_started = true;
    const auto update = lifecycle_->solver.update(new_factors, new_values);
    added_factor_indices = update.newFactorsIndices;
    ++lifecycle_->diagnostics.nonempty_updates;
    lifecycle_->diagnostics.submitted_factors += new_factors.size();
    lifecycle_->diagnostics.variables_relinearized +=
        update.variablesRelinearized;
    lifecycle_->diagnostics.variables_reeliminated +=
        update.variablesReeliminated;
    for (int i = 0; i < param_.isam_extra_updates; ++i) {
      ThrowIfCancellationRequested(context.cancellation, "optimizer update");
      const auto extra = lifecycle_->solver.update();
      lifecycle_->diagnostics.variables_relinearized +=
          extra.variablesRelinearized;
      lifecycle_->diagnostics.variables_reeliminated +=
          extra.variablesReeliminated;
    }
    working_values = lifecycle_->solver.calculateBestEstimate();
  }
  OPEN_LMM_PLOT("optimizer.delta_factor_count", new_factors.size());
  OPEN_LMM_PLOT("optimizer.factor_count",
                lifecycle_->solver.getFactorsUnsafe().nrFactors());
  OPEN_LMM_PLOT("optimizer.value_count", working_values.size());

  auto all_results = BuildOutput(context, working_values, all_raw_data,
                                 &raw_data);
  ThrowIfCancellationRequested(context.cancellation, "optimizer commit");
  lifecycle_->processed_agents.push_back(ctx.id);
  lifecycle_->factor_indices.emplace(ctx.id, std::move(added_factor_indices));
  lifecycle_->agent_keys.emplace(ctx.id, std::move(new_keys));
  ReportAlgorithmProgress(context, AlgorithmProgressPhase::kSolveGraph, 1, 1);
  return all_results;
}

BackendOptimizerOutput BackendOptimizerIncremental::BuildOutput(
    const AlgorithmExecutionContext& context, const gtsam::Values& values,
    const AgentRawDataMap& all_raw_data,
    const AgentRawData* current_raw_data) const {
  std::map<AgentId, std::vector<std::pair<int, Eigen::Isometry3d>>> all_poses;
  for (const auto& key_value : values) {
    gtsam::Key key = key_value.key;
    gtsam::Symbol symbol(key);
    if (symbol.index() == ANCHOR_IDX) continue;

    auto graph_symbol = AgentSymbol::FromByte(symbol.chr());
    if (!graph_symbol) {
      throw std::runtime_error(graph_symbol.GetError().Message());
    }
    if (!context.agent.catalog) {
      throw std::runtime_error("optimizer agent symbol catalog is missing");
    }
    auto agent = context.agent.catalog->AgentFor(graph_symbol.Value());
    if (!agent) throw std::runtime_error(agent.GetError().Message());
    gtsam::Symbol anchor_sym(symbol.chr(), ANCHOR_IDX);
    if (!values.exists(anchor_sym)) continue;

    Eigen::Matrix4d anchor_pose =
        values.at<gtsam::Pose3>(anchor_sym).matrix();
    Eigen::Matrix4d pose = values.at<gtsam::Pose3>(key).matrix();
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
    if (current_raw_data && agent == current_raw_data->agent_id) {
      corresponding_raw = current_raw_data;
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
