#pragma once

#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>

#include <Eigen/Geometry>
#include <filesystem>
#include <map>
#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/data_types.hpp>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/interface_descriptor_kdtree.hpp>

#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/load_module.hpp>

namespace open_lmm {

struct LoopDetectorInput {
  const AgentContext&                        agent_ctx;
  const AgentRawData&                        current;           // 현 에이전트 raw 데이터
  const DescriptorStore&                     descriptor_store;  // read-only
  const std::map<char, AgentRawData>&        all_raw_data;      // 이전 에이전트 raw 데이터
  const std::map<char, AgentOptimizedData>&  all_optimized;     // 이전 에이전트 최적화 포즈
  const std::shared_ptr<AlignmentFeedbackBroker>& alignment_feedback;
  const std::shared_ptr<CancellationToken>& cancellation;
  const StoredAlignment* stored_alignment = nullptr;
};

class LoopDetectorBase {
 public:
  LoopDetectorBase() = default;
  explicit LoopDetectorBase(Config config);
  virtual ~LoopDetectorBase() = default;
  virtual LoopDetectorOutput Process(const LoopDetectorInput& input) = 0;
  static Result<std::unique_ptr<LoopDetectorBase>> createInstance(Config config);
};

}  // namespace open_lmm
