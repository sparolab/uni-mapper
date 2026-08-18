#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

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

}  // namespace

int main() {
  TestCopyMergeAndClearReuse();
  TestTransactionalCompatibilityValidation();
  TestMixedStorageAndFailureBoundaries();
  std::cout << "Descriptor database tests passed\n";
  return EXIT_SUCCESS;
}
