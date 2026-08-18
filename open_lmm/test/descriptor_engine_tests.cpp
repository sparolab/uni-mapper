#include <open_lmm/core/descriptor/built_in_descriptor_engine.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

struct FakeState {
  enum class MakeMode { kSuccess, kNull, kStdThrow, kNonStdThrow, kBadKey };
  MakeMode make_mode = MakeMode::kSuccess;
  bool bad_match = false;
  bool throw_match = false;
  int make_calls = 0;
  std::vector<std::string> destruction_order;
};

class FakeDescriptor final : public IDescriptorKdtree {
 public:
  explicit FakeDescriptor(std::shared_ptr<FakeState> state, double value = 0.0,
                          std::shared_ptr<int> library_owner = {},
                          std::string destruction_label = {})
      : state_(std::move(state)),
        descriptor_(Eigen::MatrixXd::Constant(1, 1, value)),
        key_(Eigen::VectorXd::Constant(1, value)),
        library_owner_(std::move(library_owner)),
        destruction_label_(std::move(destruction_label)) {}

  ~FakeDescriptor() override {
    if (!destruction_label_.empty()) {
      state_->destruction_order.push_back(destruction_label_);
    }
  }

  const Eigen::MatrixXd& getDescriptor() const override { return descriptor_; }
  const Eigen::VectorXd& getDescriptorKey() const override { return key_; }

  std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan) override {
    ++state_->make_calls;
    switch (state_->make_mode) {
      case FakeState::MakeMode::kNull:
        return {};
      case FakeState::MakeMode::kStdThrow:
        throw std::runtime_error("fake make failure");
      case FakeState::MakeMode::kNonStdThrow:
        throw 7;
      case FakeState::MakeMode::kBadKey: {
        auto output = std::make_shared<FakeDescriptor>(state_);
        output->key_(0) = std::numeric_limits<double>::quiet_NaN();
        return output;
      }
      case FakeState::MakeMode::kSuccess:
        return std::make_shared<FakeDescriptor>(
            state_, scan->front().x, std::shared_ptr<int>{},
            destruction_label_ == "owner" ? "descriptor" : "");
    }
    return {};
  }

  std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const override {
    if (state_->throw_match) throw 11;
    const auto typed = std::dynamic_pointer_cast<FakeDescriptor>(other);
    if (!typed) throw std::invalid_argument("incompatible fake descriptor");
    const double score = state_->bad_match
                             ? std::numeric_limits<double>::quiet_NaN()
                             : std::abs(key_(0) - typed->key_(0));
    return {score, Eigen::Isometry3d::Identity()};
  }

 private:
  std::shared_ptr<FakeState> state_;
  Eigen::MatrixXd descriptor_;
  Eigen::VectorXd key_;
  std::shared_ptr<int> library_owner_;
  std::string destruction_label_;
};

class TestPayload final : public DescriptorOpaquePayload {};

AlgorithmExecutionContext Context() {
  AlgorithmExecutionContext context;
  context.base_revision = 42;
  context.cancellation = std::make_shared<CancellationToken>();
  return context;
}

DescriptorPointView Points(float x) {
  static std::vector<pcl::PointXYZI> points(1);
  points[0] = pcl::PointXYZI{x, 0.0F, 0.0F, 1.0F};
  return {std::span<const pcl::PointXYZI>(points)};
}

void CheckContext(const Error& error, const char* operation) {
  Check(error.context.runtime_revision == 42,
        "descriptor error preserves revision");
  Check(error.context.node == operation,
        "descriptor error preserves operation");
  Check(error.context.plugin == "scan_context.v1",
        "descriptor error preserves plugin identity");
}

void TestArtifactValidationAndOwnership() {
  Check(!DescriptorArtifact::Create(
             "", "scan_context", 1, {1.0},
             std::make_shared<TestPayload>()),
        "empty artifact plugin identity is rejected");
  Check(!DescriptorArtifact::Create(
             "plugin", "scan_context", 1,
             {std::numeric_limits<double>::infinity()},
             std::make_shared<TestPayload>()),
        "non-finite artifact index key is rejected");
  Check(!DescriptorArtifact::Create("plugin", "scan_context", 1, {1.0}, {}),
        "null artifact payload is rejected");

  auto state = std::make_shared<FakeState>();
  auto library_owner = std::make_shared<int>(1);
  std::weak_ptr<int> lifetime = library_owner;
  auto prototype =
      std::make_shared<FakeDescriptor>(state, 0.0, library_owner, "owner");
  auto engine = BuiltInDescriptorEngine::Create(
      "scan_context.v1", "scan_context", 1, prototype).Value();
  std::optional<DescriptorArtifact> artifact;
  {
    auto made = engine->Make(Context(), Points(2.0F));
    Check(made && made.Value().index_key() == std::vector<double>{2.0},
          "artifact owns the index key produced by the engine");
    artifact.emplace(std::move(made).Value());
  }
  library_owner.reset();
  prototype.reset();
  engine.reset();
  Check(!lifetime.expired(),
        "artifact opaque payload owns descriptor/plugin lifetime");
  artifact.reset();
  Check(state->destruction_order ==
            std::vector<std::string>{"descriptor", "owner"},
        "artifact destroys code-backed descriptor before plugin owner");
  Check(lifetime.expired(),
        "artifact destruction releases the retained plugin owner");
}

void TestV1WrapperAndCompare() {
  auto state = std::make_shared<FakeState>();
  auto engine = BuiltInDescriptorEngine::Create(
      "scan_context.v1", "scan_context", 1,
      std::make_shared<FakeDescriptor>(state)).Value();
  auto lhs = engine->Make(Context(), Points(1.0F));
  auto rhs = engine->Make(Context(), Points(4.0F));
  Check(lhs && rhs, "v1 adapter produces the artifact contract");
  auto match = engine->Compare(Context(), lhs.Value(), rhs.Value());
  Check(match && match.Value().score == 3.0,
        "v1 adapter uses the descriptor compare contract");
}

void TestFailureBoundaries() {
  auto state = std::make_shared<FakeState>();
  auto engine = BuiltInDescriptorEngine::Create(
      "scan_context.v1", "scan_context", 1,
      std::make_shared<FakeDescriptor>(state)).Value();

  auto cancelled_context = Context();
  cancelled_context.cancellation->Request();
  auto cancelled = engine->Make(cancelled_context, Points(1.0F));
  Check(!cancelled && cancelled.GetError().code == Error::Code::kCancelled &&
            state->make_calls == 0,
        "pre-cancelled make does not enter the descriptor implementation");
  CheckContext(cancelled.GetError(), "descriptor.make");

  auto invalid_points = Points(std::numeric_limits<float>::infinity());
  auto invalid_input = engine->Make(Context(), invalid_points);
  Check(!invalid_input && state->make_calls == 0,
        "non-finite point view is rejected before plugin invocation");

  for (auto mode : {FakeState::MakeMode::kNull,
                    FakeState::MakeMode::kStdThrow,
                    FakeState::MakeMode::kNonStdThrow,
                    FakeState::MakeMode::kBadKey}) {
    state->make_mode = mode;
    auto failure = engine->Make(Context(), Points(1.0F));
    Check(!failure, "invalid descriptor make becomes Result failure");
    CheckContext(failure.GetError(), "descriptor.make");
  }

  state->make_mode = FakeState::MakeMode::kSuccess;
  auto lhs = engine->Make(Context(), Points(1.0F)).Value();
  auto rhs = engine->Make(Context(), Points(2.0F)).Value();
  state->throw_match = true;
  auto thrown = engine->Compare(Context(), lhs, rhs);
  Check(!thrown, "non-standard compare exception becomes Result failure");
  CheckContext(thrown.GetError(), "descriptor.compare");
  state->throw_match = false;
  state->bad_match = true;
  Check(!engine->Compare(Context(), lhs, rhs),
        "non-finite descriptor score is rejected");

  auto other_engine = BuiltInDescriptorEngine::Create(
      "other.plugin", "scan_context", 1,
      std::make_shared<FakeDescriptor>(std::make_shared<FakeState>())).Value();
  auto other = other_engine->Make(Context(), Points(2.0F)).Value();
  Check(!engine->Compare(Context(), lhs, other),
        "cross-plugin artifact comparison is rejected");
}

}  // namespace

int main() {
  TestArtifactValidationAndOwnership();
  TestV1WrapperAndCompare();
  TestFailureBoundaries();
  std::cout << "Descriptor engine tests passed\n";
  return EXIT_SUCCESS;
}
