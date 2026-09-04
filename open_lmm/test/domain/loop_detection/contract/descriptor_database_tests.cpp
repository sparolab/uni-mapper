#include <domain/loop_detection/database/database_kdtree.h>
#include <domain/loop_detection/intra_loop_temporal_gate.hpp>
#include "support/check.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace open_lmm;

class Payload final : public DescriptorOpaquePayload {};

DescriptorArtifact Artifact(const DescriptorIndexMetadata& metadata,
                            double key) {
  return DescriptorArtifact::Create(
             metadata.plugin_id, metadata.format_id, metadata.format_version,
             std::vector<double>(metadata.index_dimension, key),
             std::make_shared<Payload>())
      .Value();
}

class Engine final : public DescriptorEngine {
 public:
  explicit Engine(DescriptorIndexMetadata metadata)
      : metadata_(std::move(metadata)) {}

  const DescriptorIndexMetadata& IndexMetadata() const override {
    return metadata_;
  }

  Result<DescriptorArtifact> Make(const AlgorithmExecutionContext&,
                                  const DescriptorPointView&) const override {
    return Result<DescriptorArtifact>::Failure(
        Error::InvalidArgument("fixture does not make descriptors"));
  }

  Result<DescriptorMatch> Compare(
      const AlgorithmExecutionContext&, const DescriptorArtifact& lhs,
      const DescriptorArtifact& rhs) const override {
    ++compare_calls;
    if (throw_nonstd) throw 9;
    return Result<DescriptorMatch>::Ok(
        DescriptorMatch{std::abs(lhs.index_key().front() -
                                 rhs.index_key().front()),
                        Eigen::Isometry3d::Identity()});
  }

  mutable int compare_calls = 0;
  bool throw_nonstd = false;

 private:
  DescriptorIndexMetadata metadata_;
};

class LegacyDescriptor final : public IDescriptorKdtree {
 public:
  explicit LegacyDescriptor(double key)
      : descriptor_(Eigen::MatrixXd::Constant(1, 1, key)),
        key_(Eigen::VectorXd::Constant(1, key)) {}
  const Eigen::MatrixXd& getDescriptor() const override { return descriptor_; }
  const Eigen::VectorXd& getDescriptorKey() const override { return key_; }
  std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr&) override {
    return std::make_shared<LegacyDescriptor>(key_(0));
  }
  std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const override {
    const auto typed = std::dynamic_pointer_cast<LegacyDescriptor>(other);
    if (!typed) throw std::invalid_argument("wrong fixture descriptor");
    return {std::abs(key_(0) - typed->key_(0)),
            Eigen::Isometry3d::Identity()};
  }

 private:
  Eigen::MatrixXd descriptor_;
  Eigen::VectorXd key_;
};

AlgorithmExecutionContext Context() {
  AlgorithmExecutionContext context;
  context.base_revision = 17;
  context.cancellation = std::make_shared<CancellationToken>();
  return context;
}

AgentId Agent(const char* value) { return AgentId::Parse(value).Value(); }

const DescriptorIndexMetadata kMetadata{"fixture.scan", "scan_context", 1, 1};
const DatabaseKdtreeParams kParams{1, 4, 10.0, 50};

struct TemporalRun {
  std::vector<std::optional<std::size_t>> matches;
  std::size_t final_database_size = 0;
  std::optional<std::size_t> post_flush_match;
};

TemporalRun RunTemporalSequence(const std::vector<double>& keys,
                                std::size_t minimum_frame_gap,
                                std::size_t rebuild_threshold,
                                std::optional<double> post_flush_query = {}) {
  auto engine = std::make_shared<Engine>(kMetadata);
  DatabaseKdtree database(
      DatabaseKdtreeParams{1, 4, 1000.0, rebuild_threshold}, engine);
  IntraLoopTemporalGate gate(minimum_frame_gap);
  const auto insert = [&](std::size_t frame_index,
                          DescriptorArtifact descriptor) {
    database.insertArtifact(Agent("A"), frame_index, std::move(descriptor));
  };

  TemporalRun run;
  for (std::size_t index = 0; index < keys.size(); ++index) {
    gate.PromoteEligible(index, insert);
    auto queried = database.queryArtifact(
        Context(), Artifact(kMetadata, keys[index]));
    Check(queried.IsOk(), "temporal query succeeds");
    run.matches.push_back(
        queried.Value() ? std::optional<std::size_t>(
                              std::get<1>(*queried.Value()))
                        : std::nullopt);
    gate.Defer(index, Artifact(kMetadata, keys[index]));
  }
  gate.Flush(insert);
  run.final_database_size = database.getSize();

  if (post_flush_query) {
    auto queried = database.queryArtifact(
        Context(), Artifact(kMetadata, *post_flush_query));
    Check(queried.IsOk(), "post-flush descriptor query succeeds");
    if (queried.Value()) run.post_flush_match = std::get<1>(*queried.Value());
  }
  return run;
}

void TestCopyMergeAndClearReuse() {
  auto left_engine = std::make_shared<Engine>(kMetadata);
  auto right_engine = std::make_shared<Engine>(kMetadata);
  DatabaseKdtree left(kParams, left_engine);
  DatabaseKdtree right(kParams, right_engine);
  left.insertArtifact(Agent("A"), 1, Artifact(kMetadata, 1.0));
  right.insertArtifact(Agent("B"), 2, Artifact(kMetadata, 2.0));

  DatabaseKdtree copied(left);
  auto copy_match = copied.queryArtifact(Context(), Artifact(kMetadata, 1.0));
  Check(copy_match && copy_match.Value() &&
            std::get<0>(*copy_match.Value()) == Agent("A"),
        "copied artifact database remains queryable");

  left.merge(right);
  Check(left.getSize() == 2, "compatible engine identities merge");
  auto merged = left.queryArtifact(Context(), Artifact(kMetadata, 2.0));
  Check(merged && merged.Value() &&
            std::get<0>(*merged.Value()) == Agent("B"),
        "merged descriptor is queryable");

  left.clear();
  Check(left.getSize() == 0, "clear removes descriptors");
  left.insertArtifact(Agent("C"), 3, Artifact(kMetadata, 3.0));
  auto reused = left.queryArtifact(Context(), Artifact(kMetadata, 3.0));
  Check(reused && reused.Value() &&
            std::get<0>(*reused.Value()) == Agent("C"),
        "cleared database can be reused immediately");
}

void TestTransactionalCompatibilityValidation() {
  auto engine = std::make_shared<Engine>(kMetadata);
  DatabaseKdtree destination(kParams, engine);
  destination.insertArtifact(Agent("A"), 1, Artifact(kMetadata, 1.0));

  for (const DescriptorIndexMetadata& metadata : {
           DescriptorIndexMetadata{"other.plugin", "scan_context", 1, 1},
           DescriptorIndexMetadata{"fixture.scan", "other_format", 1, 1},
           DescriptorIndexMetadata{"fixture.scan", "scan_context", 2, 1},
       }) {
    DatabaseKdtree source(kParams, std::make_shared<Engine>(metadata));
    source.insertArtifact(Agent("B"), 2, Artifact(metadata, 2.0));
    try {
      destination.merge(source);
      Check(false, "incompatible descriptor identity must reject merge");
    } catch (const std::invalid_argument&) {
    }
    Check(destination.getSize() == 1,
          "rejected merge preserves destination database");
  }

  const DescriptorIndexMetadata dimension_metadata{
      "fixture.scan", "scan_context", 1, 2};
  DatabaseKdtree dimension_source(
      DatabaseKdtreeParams{2, 4, 10.0, 50},
      std::make_shared<Engine>(dimension_metadata));
  dimension_source.insertArtifact(Agent("B"), 2,
                                  Artifact(dimension_metadata, 2.0));
  try {
    destination.merge(dimension_source);
    Check(false, "incompatible key dimension must reject merge");
  } catch (const std::invalid_argument&) {
  }
  Check(destination.getSize() == 1,
        "dimension failure is transactional");
}

void TestMixedStorageAndFailureBoundaries() {
  auto engine = std::make_shared<Engine>(kMetadata);
  DatabaseKdtree artifacts(kParams, engine);
  artifacts.insertArtifact(Agent("A"), 1, Artifact(kMetadata, 1.0));
  try {
    artifacts.insert(Agent("A"), 2,
                     std::make_shared<LegacyDescriptor>(2.0));
    Check(false, "legacy insert into artifact database must fail");
  } catch (const std::invalid_argument&) {
  }
  try {
    artifacts.query(std::make_shared<LegacyDescriptor>(1.0));
    Check(false, "legacy query of artifact database must fail");
  } catch (const std::invalid_argument&) {
  }

  DatabaseKdtree legacy(kParams);
  legacy.insert(Agent("L"), 1, std::make_shared<LegacyDescriptor>(1.0));
  try {
    legacy.insertArtifact(Agent("L"), 2, Artifact(kMetadata, 2.0));
    Check(false, "artifact insert into legacy database must fail");
  } catch (const std::invalid_argument&) {
  }
  auto wrong_query = legacy.queryArtifact(Context(), Artifact(kMetadata, 1.0));
  Check(!wrong_query && wrong_query.GetError().context.runtime_revision == 17,
        "artifact query of legacy database is a contextual Result failure");

  auto cancelled_context = Context();
  cancelled_context.cancellation->Request();
  auto cancelled =
      artifacts.queryArtifact(cancelled_context, Artifact(kMetadata, 1.0));
  Check(!cancelled && cancelled.GetError().code == Error::Code::kCancelled &&
            engine->compare_calls == 0,
        "pre-cancelled query does not invoke descriptor engine");

  engine->throw_nonstd = true;
  auto unknown = artifacts.queryArtifact(Context(), Artifact(kMetadata, 1.0));
  Check(!unknown && unknown.GetError().context.node == "descriptor.index.query",
        "non-standard query exception becomes contextual Result failure");
}

void TestIntraLoopTemporalGateBoundaries() {
  const auto gated = RunTemporalSequence({0, 0, 0, 0, 0}, 3, 50);
  Check(!gated.matches[2], "frame gap N-1 remains ineligible");
  Check(gated.matches[3] && *gated.matches[3] == 0,
        "frame gap N becomes eligible");

  const auto disabled = RunTemporalSequence({0, 0}, 0, 50);
  Check(disabled.matches[1] && *disabled.matches[1] == 0,
        "zero frame gap makes the immediately prior descriptor eligible");

  const auto larger_than_sequence =
      RunTemporalSequence({0, 1, 2, 3}, 10, 50, 3);
  Check(std::none_of(larger_than_sequence.matches.begin(),
                     larger_than_sequence.matches.end(),
                     [](const auto& match) { return match.has_value(); }),
        "gap larger than scan count produces no intra-loop candidates");
  Check(larger_than_sequence.final_database_size == 4,
        "deferred tail is flushed into the final descriptor database");
  Check(larger_than_sequence.post_flush_match &&
            *larger_than_sequence.post_flush_match == 3,
        "the final descriptor is available to later inter-agent queries");
}

void TestIntraLoopTemporalGateIsIndependentOfRebuildCadence() {
  const std::vector<double> keys{0, 10, 0.1, 10.1, 0.2, 10.2, 0.3};
  const auto frequent_rebuild = RunTemporalSequence(keys, 2, 2);
  const auto batched_rebuild = RunTemporalSequence(keys, 2, 50);
  Check(frequent_rebuild.matches == batched_rebuild.matches,
        "temporal eligibility is independent of KD-tree rebuild cadence");

  const auto recent_nearest_excluded =
      RunTemporalSequence({0, 100, 1, 1}, 3, 50);
  Check(recent_nearest_excluded.matches[3] &&
            *recent_nearest_excluded.matches[3] == 0,
        "an older eligible descriptor wins while the nearer recent tail is "
        "excluded");
}

}  // namespace

int main() {
  TestCopyMergeAndClearReuse();
  TestTransactionalCompatibilityValidation();
  TestMixedStorageAndFailureBoundaries();
  TestIntraLoopTemporalGateBoundaries();
  TestIntraLoopTemporalGateIsIndependentOfRebuildCadence();
  std::cout << "Descriptor database tests passed\n";
  return EXIT_SUCCESS;
}
