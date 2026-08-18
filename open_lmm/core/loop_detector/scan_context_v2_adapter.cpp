#include "scan_context_v2_adapter.hpp"

#include <open_lmm/common/plugin_host_v2.hpp>
#include <open_lmm/utils/config.hpp>

#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <utility>

namespace open_lmm {
namespace {

class ScanContextV2Adapter final : public IDescriptorKdtree {
 public:
  ScanContextV2Adapter(std::shared_ptr<PluginV2> plugin,
                       Eigen::MatrixXd descriptor, Eigen::VectorXd key)
      : plugin_(std::move(plugin)), descriptor_(std::move(descriptor)),
        key_(std::move(key)) {}

  const Eigen::MatrixXd& getDescriptor() const override {
    return descriptor_;
  }

  const Eigen::VectorXd& getDescriptorKey() const override { return key_; }

  std::shared_ptr<IDescriptorKdtree> makeDescriptor(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& scan) override {
    if (!plugin_ || !scan) {
      throw std::invalid_argument("ABI-v2 Scan Context requires a point cloud");
    }
    PluginV2Call call{OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2, nullptr, 0};
    call.points = PluginPointView{
        scan->points.data(), static_cast<uint64_t>(scan->points.size()),
        sizeof(pcl::PointXYZI), OPEN_LMM_ELEMENT_F32_V2,
        OPEN_LMM_ENDIAN_LITTLE_V2};
    auto response = plugin_->Call(call);
    if (!response) {
      throw std::runtime_error(response.GetError().Message());
    }
    auto decoded = Decode(std::move(response).Value());
    if (!decoded) {
      throw std::runtime_error(decoded.GetError().Message());
    }
    auto [descriptor, key] = std::move(decoded).Value();
    return std::make_shared<ScanContextV2Adapter>(
        plugin_, std::move(descriptor), std::move(key));
  }

  std::pair<double, Eigen::Isometry3d> distance(
      const std::shared_ptr<IDescriptorKdtree>& other) const override {
    if (!other || descriptor_.rows() != other->getDescriptor().rows() ||
        descriptor_.cols() != other->getDescriptor().cols() ||
        descriptor_.cols() == 0) {
      throw std::invalid_argument("incompatible ABI-v2 Scan Context descriptor");
    }
    double minimum = std::numeric_limits<double>::max();
    Eigen::Index minimum_offset = 0;
    for (Eigen::Index offset = 0; offset < descriptor_.cols(); ++offset) {
      double sum = 0.0;
      for (Eigen::Index sector = 0; sector < descriptor_.cols(); ++sector) {
        const Eigen::Index shifted = (sector + offset) % descriptor_.cols();
        const Eigen::VectorXd lhs = descriptor_.col(shifted);
        const Eigen::VectorXd rhs = other->getDescriptor().col(sector);
        const double lhs_norm = lhs.norm();
        const double rhs_norm = rhs.norm();
        if (lhs_norm > 0.0 && rhs_norm > 0.0) {
          sum += 1.0 - lhs.dot(rhs) / (lhs_norm * rhs_norm);
        } else if ((lhs_norm > 0.0) != (rhs_norm > 0.0)) {
          sum += 1.0;
        }
      }
      const double candidate = sum / static_cast<double>(descriptor_.cols());
      if (candidate < minimum) {
        minimum = candidate;
        minimum_offset = offset;
      }
    }
    Eigen::Isometry3d relative = Eigen::Isometry3d::Identity();
    relative.rotate(Eigen::AngleAxisd(
        static_cast<double>(minimum_offset) * 2.0 * M_PI /
            static_cast<double>(descriptor_.cols()),
        Eigen::Vector3d::UnitZ()));
    return {minimum, relative};
  }

  static Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>> Decode(
      std::vector<uint8_t> bytes) {
    if (bytes.size() < sizeof(open_lmm_descriptor_result_v2)) {
      return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Failure(
          Error::PluginLoadFailed("ABI-v2 descriptor result is truncated"));
    }
    open_lmm_descriptor_result_v2 header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    if (header.struct_size != sizeof(header) ||
        header.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
        header.element_type != OPEN_LMM_ELEMENT_F64_V2 ||
        header.endian != OPEN_LMM_ENDIAN_LITTLE_V2 ||
        std::endian::native != std::endian::little || header.rows == 0 ||
        header.columns == 0 || header.key_count == 0 ||
        header.rows > std::numeric_limits<uint64_t>::max() / header.columns) {
      return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Failure(
          Error::PluginLoadFailed("invalid ABI-v2 descriptor result header"));
    }
    const uint64_t descriptor_count = header.rows * header.columns;
    if (descriptor_count >
        std::numeric_limits<uint64_t>::max() - header.key_count) {
      return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Failure(
          Error::PluginLoadFailed("ABI-v2 descriptor result size overflow"));
    }
    const uint64_t value_count = descriptor_count + header.key_count;
    if (value_count >
        (std::numeric_limits<std::size_t>::max() - sizeof(header)) /
            sizeof(double)) {
      return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Failure(
          Error::PluginLoadFailed("ABI-v2 descriptor result is too large"));
    }
    const std::size_t expected = sizeof(header) +
        static_cast<std::size_t>(value_count) * sizeof(double);
    if (bytes.size() != expected ||
        header.rows > static_cast<uint64_t>(std::numeric_limits<Eigen::Index>::max()) ||
        header.columns > static_cast<uint64_t>(std::numeric_limits<Eigen::Index>::max()) ||
        header.key_count > static_cast<uint64_t>(std::numeric_limits<Eigen::Index>::max())) {
      return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Failure(
          Error::PluginLoadFailed("ABI-v2 descriptor result length mismatch"));
    }
    Eigen::MatrixXd descriptor(
        static_cast<Eigen::Index>(header.rows),
        static_cast<Eigen::Index>(header.columns));
    Eigen::VectorXd key(static_cast<Eigen::Index>(header.key_count));
    const uint8_t* cursor = bytes.data() + sizeof(header);
    for (Eigen::Index row = 0; row < descriptor.rows(); ++row) {
      for (Eigen::Index column = 0; column < descriptor.cols(); ++column) {
        std::memcpy(&descriptor(row, column), cursor, sizeof(double));
        cursor += sizeof(double);
      }
    }
    for (Eigen::Index index = 0; index < key.size(); ++index) {
      std::memcpy(&key(index), cursor, sizeof(double));
      cursor += sizeof(double);
    }
    return Result<std::pair<Eigen::MatrixXd, Eigen::VectorXd>>::Ok(
        {std::move(descriptor), std::move(key)});
  }

 private:
  std::shared_ptr<PluginV2> plugin_;
  Eigen::MatrixXd descriptor_;
  Eigen::VectorXd key_;
};

Result<std::shared_ptr<PluginV2>> LoadPlugin(
    const std::string& shared_library, const std::string& config_json) {
  auto loaded = LoadPluginV2(
      shared_library, "descriptor", config_json,
      OPEN_LMM_CAPABILITY_POINT_VIEW_V2);
  if (!loaded) {
    return Result<std::shared_ptr<PluginV2>>::Failure(loaded.GetError());
  }
  return Result<std::shared_ptr<PluginV2>>::Ok(
      std::make_shared<PluginV2>(std::move(loaded).Value()));
}

}  // namespace

Result<std::shared_ptr<IDescriptorKdtree>> LoadScanContextV2Adapter(
    const std::string& shared_library, const std::string& config_json) {
  Config config = Config::FromJson(config_json, "scan_context ABI-v2 config");
  if (!config.is_valid()) {
    return Result<std::shared_ptr<IDescriptorKdtree>>::Failure(
        Error::ParseError(config.error_message()));
  }
  const int rows = config.param<int>("loop_detector", "num_ring", 20);
  const int columns = config.param<int>("loop_detector", "num_sector", 60);
  if (rows <= 0 || columns <= 0) {
    return Result<std::shared_ptr<IDescriptorKdtree>>::Failure(
        Error::InvalidArgument("invalid ABI-v2 Scan Context dimensions"));
  }
  auto plugin = LoadPlugin(shared_library, config_json);
  if (!plugin) {
    return Result<std::shared_ptr<IDescriptorKdtree>>::Failure(
        plugin.GetError());
  }
  return Result<std::shared_ptr<IDescriptorKdtree>>::Ok(
      std::make_shared<ScanContextV2Adapter>(
          std::move(plugin).Value(), Eigen::MatrixXd::Zero(rows, columns),
          Eigen::VectorXd::Zero(rows)));
}

Result<void> InspectScanContextV2Plugin(
    const std::string& shared_library, const std::string& config_json) {
  auto plugin = LoadPlugin(shared_library, config_json);
  if (!plugin) return Result<void>::Failure(plugin.GetError());
  return Result<void>::Ok();
}

}  // namespace open_lmm
