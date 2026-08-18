#include "generic_descriptor_v2_adapter.hpp"

#include <open_lmm/common/plugin_host_v2.hpp>
#include <open_lmm/common/validation.hpp>

#include <dlfcn.h>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <span>
#include <sstream>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

namespace open_lmm {
namespace {

constexpr uint64_t kDescriptorCapabilities =
    OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2 |
    OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2;

int32_t OPEN_LMM_PLUGIN_CALL_V2 IsContextCancelled(void* context) {
  const auto* token = static_cast<const CancellationToken*>(context);
  return token && token->IsCancellationRequested() ? 1 : 0;
}

bool DeclaresOperation(const PluginV2Metadata& metadata,
                       std::string_view name, uint64_t required_bit) {
  return std::any_of(metadata.operations.begin(), metadata.operations.end(),
                     [&](const auto& operation) {
    return operation.operation == name &&
           (operation.required_capability_bits & required_bit) == required_bit;
  });
}

std::string ConfigIdentity(std::string_view config) {
  uint64_t hash = UINT64_C(14695981039346656037);
  for (const unsigned char byte : config) {
    hash ^= byte;
    hash *= UINT64_C(1099511628211);
  }
  std::ostringstream stream;
  stream << std::hex << hash;
  return stream.str();
}

bool ValidProbeString(const open_lmm_string_view_v2& value,
                      bool require_non_empty = false) {
  return value.struct_size >= sizeof(value) &&
         value.abi_major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR &&
         value.size <= UINT64_C(1024) * 1024 &&
         (value.size == 0 || value.data != nullptr) &&
         (!require_non_empty || value.size != 0);
}

bool ValidKnownMetadata(const open_lmm_plugin_descriptor_v2& descriptor) {
  if (!ValidProbeString(descriptor.plugin_kind, true) ||
      !ValidProbeString(descriptor.plugin_name, true) ||
      !ValidProbeString(descriptor.capability)) {
    return false;
  }
  if (std::string_view(descriptor.plugin_kind.data,
                       descriptor.plugin_kind.size) != "descriptor") {
    return false;
  }
  if (descriptor.abi_minor == 0) return true;
  if (!ValidProbeString(descriptor.plugin_id, true) ||
      !ValidProbeString(descriptor.plugin_version, true) ||
      !ValidProbeString(descriptor.schema_id) ||
      descriptor.operation_count > 128 ||
      (descriptor.operation_count != 0 && !descriptor.operations) ||
      descriptor.artifact_format_count > 64 ||
      (descriptor.artifact_format_count != 0 &&
       !descriptor.artifact_formats) ||
      descriptor.thread_safety > OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2 ||
      descriptor.cancellation > OPEN_LMM_CANCELLATION_COOPERATIVE_V2) {
    return false;
  }
  const bool concurrent_metadata = descriptor.thread_safety ==
      OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2;
  const bool concurrent_capability =
      (descriptor.capability_bits & OPEN_LMM_CAPABILITY_CONCURRENT_CALLS_V2) !=
      0;
  if (concurrent_metadata != concurrent_capability) return false;
  for (uint64_t i = 0; i < descriptor.operation_count; ++i) {
    const auto& operation = descriptor.operations[i];
    if (operation.struct_size < sizeof(operation) ||
        operation.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
        !ValidProbeString(operation.operation, true) ||
        (operation.required_capability_bits & descriptor.capability_bits) !=
            operation.required_capability_bits) {
      return false;
    }
    for (uint64_t prior = 0; prior < i; ++prior) {
      const auto& existing = descriptor.operations[prior].operation;
      if (existing.size == operation.operation.size &&
          std::memcmp(existing.data, operation.operation.data,
                      static_cast<std::size_t>(existing.size)) == 0) {
        return false;
      }
    }
  }
  for (uint64_t i = 0; i < descriptor.artifact_format_count; ++i) {
    const auto& format = descriptor.artifact_formats[i];
    if (format.struct_size < sizeof(format) ||
        format.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
        !ValidProbeString(format.format_id, true) || format.format_version == 0 ||
        format.index_dimension == 0) {
      return false;
    }
    for (uint64_t prior = 0; prior < i; ++prior) {
      const auto& existing = descriptor.artifact_formats[prior].format_id;
      if (existing.size == format.format_id.size &&
          descriptor.artifact_formats[prior].format_version ==
              format.format_version &&
          std::memcmp(existing.data, format.format_id.data,
                      static_cast<std::size_t>(existing.size)) == 0) {
        return false;
      }
    }
  }
  if ((descriptor.capability_bits & OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) !=
      0) {
    if (descriptor.schema_id.size == 0 || descriptor.schema_version == 0) {
      return false;
    }
    bool declares_schema = false;
    for (uint64_t i = 0; i < descriptor.operation_count; ++i) {
      const auto& operation = descriptor.operations[i];
      declares_schema = declares_schema ||
          (std::string_view(operation.operation.data,
                            operation.operation.size) ==
               OPEN_LMM_PLUGIN_SCHEMA_OPERATION_V2 &&
           (operation.required_capability_bits &
            OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0);
    }
    if (!declares_schema) return false;
  }
  return true;
}

template <typename T>
T WireHeader() {
  T value{};
  value.struct_size = sizeof(T);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return value;
}

bool AddOverflow(std::size_t lhs, uint64_t rhs, std::size_t* result) {
  if (rhs > std::numeric_limits<std::size_t>::max() - lhs) return true;
  *result = lhs + static_cast<std::size_t>(rhs);
  return false;
}

uint64_t ScalarWidth(open_lmm_element_type_v2 type) {
  if (type == OPEN_LMM_ELEMENT_F32_V2) return sizeof(float);
  if (type == OPEN_LMM_ELEMENT_F64_V2) return sizeof(double);
  return 0;
}

template <typename T>
T ByteSwap(T value) {
  static_assert(std::is_trivially_copyable_v<T>);
  std::array<std::byte, sizeof(T)> bytes{};
  std::memcpy(bytes.data(), &value, sizeof(T));
  std::reverse(bytes.begin(), bytes.end());
  std::memcpy(&value, bytes.data(), sizeof(T));
  return value;
}

template <typename T>
T ReadScalar(const uint8_t* bytes, open_lmm_endian_v2 endian) {
  T value{};
  std::memcpy(&value, bytes, sizeof(T));
  const bool wire_little = endian == OPEN_LMM_ENDIAN_LITTLE_V2;
  const bool host_little = std::endian::native == std::endian::little;
  if (wire_little != host_little) value = ByteSwap(value);
  return value;
}

class GenericPayload final : public DescriptorOpaquePayload {
 public:
  GenericPayload(std::shared_ptr<PluginV2> owner, std::string engine_identity,
                 std::vector<uint8_t> bytes)
      : owner(std::move(owner)), engine_identity(std::move(engine_identity)),
        bytes(std::move(bytes)) {}

  // Reverse destruction releases bytes before the code/handle owner.
  std::shared_ptr<PluginV2> owner;
  std::string engine_identity;
  std::vector<uint8_t> bytes;
};

class GenericDescriptorV2Engine final : public DescriptorEngine {
 public:
  GenericDescriptorV2Engine(std::shared_ptr<PluginV2> plugin,
                            DescriptorIndexMetadata metadata)
      : plugin_(std::move(plugin)), metadata_(std::move(metadata)) {}

  const DescriptorIndexMetadata& IndexMetadata() const override {
    return metadata_;
  }

  Result<DescriptorArtifact> Make(
      const AlgorithmExecutionContext& supplied,
      const DescriptorPointView& points) const override {
    auto context = Context(supplied, "descriptor.make");
    AlgorithmExecutionTimer timer(context);
    auto active = CheckAlgorithmCancellation(context, "before descriptor make");
    if (!active) return Result<DescriptorArtifact>::Failure(active.GetError());
    if (points.points.empty()) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor point view is empty"), context));
    }
    PluginV2Call call{OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2, nullptr, 0};
    call.points = PluginPointView{
        points.points.data(), static_cast<uint64_t>(points.points.size()),
        sizeof(pcl::PointXYZI), OPEN_LMM_ELEMENT_F32_V2,
        std::endian::native == std::endian::little
            ? OPEN_LMM_ENDIAN_LITTLE_V2
            : OPEN_LMM_ENDIAN_BIG_V2};
    ApplyLimits(context, &call);
    if (context.cancellation) {
      call.cancellation_context = context.cancellation.get();
      call.is_cancelled = &IsContextCancelled;
    }
    auto response = plugin_->Call(call);
    if (!response) {
      return Result<DescriptorArtifact>::Failure(
          WithAlgorithmContext(response.GetError(), context));
    }
    auto artifact = DecodeArtifact(context, std::move(response).Value());
    if (!artifact) return artifact;
    active = CheckAlgorithmCancellation(context, "after descriptor make");
    if (!active) {
      return Result<DescriptorArtifact>::Failure(active.GetError());
    }
    return artifact;
  }

  Result<DescriptorMatch> Compare(
      const AlgorithmExecutionContext& supplied,
      const DescriptorArtifact& lhs,
      const DescriptorArtifact& rhs) const override {
    auto context = Context(supplied, "descriptor.compare");
    AlgorithmExecutionTimer timer(context);
    auto active =
        CheckAlgorithmCancellation(context, "before descriptor compare");
    if (!active) return Result<DescriptorMatch>::Failure(active.GetError());
    if (!Compatible(lhs) || !Compatible(rhs)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor artifact identity mismatch"),
          context));
    }
    const auto lhs_payload =
        std::dynamic_pointer_cast<const GenericPayload>(lhs.opaque_payload());
    const auto rhs_payload =
        std::dynamic_pointer_cast<const GenericPayload>(rhs.opaque_payload());
    if (!lhs_payload || !rhs_payload) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor artifact payload mismatch"),
          context));
    }
    if (lhs_payload->engine_identity != metadata_.engine_identity ||
        rhs_payload->engine_identity != metadata_.engine_identity) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor artifact engine identity mismatch"),
          context));
    }
    auto header = WireHeader<open_lmm_descriptor_compare_request_v2>();
    header.format_id_size = metadata_.format_id.size();
    header.format_version = metadata_.format_version;
    header.lhs_payload_size = lhs_payload->bytes.size();
    header.rhs_payload_size = rhs_payload->bytes.size();
    std::size_t size = sizeof(header);
    if (AddOverflow(size, header.format_id_size, &size) ||
        AddOverflow(size, header.lhs_payload_size, &size) ||
        AddOverflow(size, header.rhs_payload_size, &size)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("descriptor compare request overflow"),
          context));
    }
    std::vector<uint8_t> request(size);
    uint8_t* cursor = request.data();
    std::memcpy(cursor, &header, sizeof(header));
    cursor += sizeof(header);
    std::memcpy(cursor, metadata_.format_id.data(), metadata_.format_id.size());
    cursor += metadata_.format_id.size();
    std::memcpy(cursor, lhs_payload->bytes.data(), lhs_payload->bytes.size());
    cursor += lhs_payload->bytes.size();
    std::memcpy(cursor, rhs_payload->bytes.data(), rhs_payload->bytes.size());

    PluginV2Call call{OPEN_LMM_DESCRIPTOR_COMPARE_OPERATION_V2, request.data(),
                      static_cast<uint64_t>(request.size())};
    ApplyLimits(context, &call);
    if (context.cancellation) {
      call.cancellation_context = context.cancellation.get();
      call.is_cancelled = &IsContextCancelled;
    }
    auto response = plugin_->Call(call);
    if (!response) {
      return Result<DescriptorMatch>::Failure(
          WithAlgorithmContext(response.GetError(), context));
    }
    return DecodeMatch(context, std::move(response).Value());
  }

 private:
  AlgorithmExecutionContext Context(const AlgorithmExecutionContext& context,
                                    const char* operation) const {
    auto enriched = context;
    enriched.operation = operation;
    enriched.plugin_id = metadata_.plugin_id;
    return enriched;
  }

  void ApplyLimits(const AlgorithmExecutionContext& context,
                   PluginV2Call* call) const {
    if (context.resource_budget.maximum_result_bytes != 0) {
      call->result_limits.maximum_result_bytes =
          context.resource_budget.maximum_result_bytes;
      call->result_limits.maximum_chunk_bytes = std::min(
          call->result_limits.maximum_chunk_bytes,
          call->result_limits.maximum_result_bytes);
    }
  }

  bool Compatible(const DescriptorArtifact& artifact) const {
    return artifact.plugin_id() == metadata_.plugin_id &&
           artifact.format_id() == metadata_.format_id &&
           artifact.format_version() == metadata_.format_version &&
           artifact.index_key().size() == metadata_.index_dimension;
  }

  Result<DescriptorArtifact> DecodeArtifact(
      const AlgorithmExecutionContext& context,
      std::vector<uint8_t> bytes) const {
    if (bytes.size() < sizeof(open_lmm_descriptor_artifact_header_v2)) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor artifact header is truncated"),
          context));
    }
    open_lmm_descriptor_artifact_header_v2 header{};
    std::memcpy(&header, bytes.data(), sizeof(header));
    const uint64_t width = ScalarWidth(header.element_type);
    if (header.struct_size < sizeof(header) || header.struct_size > bytes.size() ||
        header.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR || width == 0 ||
        (header.endian != OPEN_LMM_ENDIAN_LITTLE_V2 &&
         header.endian != OPEN_LMM_ENDIAN_BIG_V2) ||
        header.key_count != metadata_.index_dimension ||
        header.key_count > std::numeric_limits<uint64_t>::max() / width) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor artifact header is malformed"),
          context));
    }
    std::size_t expected = header.struct_size;
    if (AddOverflow(expected, header.format_id_size, &expected) ||
        AddOverflow(expected, header.key_count * width, &expected) ||
        AddOverflow(expected, header.payload_size, &expected) ||
        expected != bytes.size()) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor artifact length mismatch"),
          context));
    }
    const uint8_t* cursor = bytes.data() + header.struct_size;
    const std::string format(reinterpret_cast<const char*>(cursor),
                             static_cast<std::size_t>(header.format_id_size));
    cursor += header.format_id_size;
    if (format != metadata_.format_id ||
        header.format_version != metadata_.format_version) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor artifact format mismatch"),
          context));
    }
    std::vector<double> key;
    key.reserve(static_cast<std::size_t>(header.key_count));
    for (uint64_t index = 0; index < header.key_count; ++index) {
      const double value = header.element_type == OPEN_LMM_ELEMENT_F32_V2
          ? static_cast<double>(ReadScalar<float>(cursor, header.endian))
          : ReadScalar<double>(cursor, header.endian);
      if (!std::isfinite(value)) {
        return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
            Error::PluginLoadFailed("descriptor index key is non-finite"),
            context));
      }
      key.push_back(value);
      cursor += width;
    }
    const uint8_t* end = bytes.data() + bytes.size();
    std::vector<uint8_t> payload(cursor, end);
    if (payload.size() != header.payload_size) {
      return Result<DescriptorArtifact>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor opaque payload mismatch"),
          context));
    }
    auto artifact = DescriptorArtifact::Create(
        metadata_.plugin_id, format, header.format_version, std::move(key),
        std::make_shared<GenericPayload>(plugin_, metadata_.engine_identity,
                                         std::move(payload)));
    if (!artifact) {
      return Result<DescriptorArtifact>::Failure(
          WithAlgorithmContext(artifact.GetError(), context));
    }
    return artifact;
  }

  Result<DescriptorMatch> DecodeMatch(
      const AlgorithmExecutionContext& context,
      const std::vector<uint8_t>& bytes) const {
    if (bytes.size() < sizeof(open_lmm_descriptor_compare_result_v2)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor compare result length mismatch"),
          context));
    }
    open_lmm_descriptor_compare_result_v2 wire{};
    std::memcpy(&wire, bytes.data(), sizeof(wire));
    if (wire.struct_size < sizeof(wire) || wire.struct_size != bytes.size() ||
        wire.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
        wire.element_type != OPEN_LMM_ELEMENT_F64_V2 ||
        (wire.endian != OPEN_LMM_ENDIAN_LITTLE_V2 &&
         wire.endian != OPEN_LMM_ENDIAN_BIG_V2)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor compare result is malformed"),
          context));
    }
    const auto* raw = reinterpret_cast<const uint8_t*>(&wire.score);
    const double score = ReadScalar<double>(raw, wire.endian);
    Eigen::Matrix4d matrix;
    raw = reinterpret_cast<const uint8_t*>(wire.relative_pose_row_major);
    for (Eigen::Index row = 0; row < 4; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        matrix(row, column) = ReadScalar<double>(
            raw + static_cast<std::size_t>(row * 4 + column) * sizeof(double),
            wire.endian);
      }
    }
    Eigen::Isometry3d pose(matrix);
    if (!std::isfinite(score) || !IsFiniteRigidTransform(pose)) {
      return Result<DescriptorMatch>::Failure(WithAlgorithmContext(
          Error::PluginLoadFailed("descriptor compare result is not rigid"),
          context));
    }
    auto active =
        CheckAlgorithmCancellation(context, "after descriptor compare");
    if (!active) return Result<DescriptorMatch>::Failure(active.GetError());
    return Result<DescriptorMatch>::Ok(DescriptorMatch{score, pose});
  }

  // The engine is declared before artifact payload bytes in GenericPayload,
  // so artifacts retain the plugin handle independently when the engine dies.
  std::shared_ptr<PluginV2> plugin_;
  DescriptorIndexMetadata metadata_;
};

}  // namespace

Result<DescriptorV2Availability> ProbeGenericDescriptorV2Plugin(
    const std::string& shared_library) {
  void* library = dlopen(shared_library.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library) {
    const char* message = dlerror();
    return Result<DescriptorV2Availability>::Failure(
        Error::PluginLoadFailed(
            std::string("descriptor ABI-v2 probe dlopen failed: ") +
            (message ? message : "unknown error")));
  }
  const auto close_library = [&] { dlclose(library); };
  const auto symbol = [&](const char* name) {
    dlerror();
    void* value = dlsym(library, name);
    const char* error = dlerror();
    return error ? nullptr : value;
  };
  void* query_raw = symbol(OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2);
  if (!query_raw || !symbol(OPEN_LMM_PLUGIN_OPEN_SYMBOL_V2) ||
      !symbol(OPEN_LMM_PLUGIN_CALL_SYMBOL_V2) ||
      !symbol(OPEN_LMM_PLUGIN_CLOSE_SYMBOL_V2)) {
    close_library();
    return Result<DescriptorV2Availability>::Ok(
        DescriptorV2Availability::kUnavailable);
  }
  auto query = reinterpret_cast<open_lmm_plugin_query_fn_v2>(query_raw);
  auto descriptor = WireHeader<open_lmm_plugin_descriptor_v2>();
  open_lmm_status_v2 status{};
  try {
    status = query(&descriptor);
  } catch (...) {
    close_library();
    return Result<DescriptorV2Availability>::Failure(
        Error::PluginLoadFailed("descriptor ABI-v2 query threw"));
  }
  const bool status_header = status.struct_size >= sizeof(status) &&
                             status.abi_major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  const bool message_valid = status_header &&
      (status.message.size == 0 || status.message.data != nullptr) &&
      status.message.struct_size >= sizeof(status.message) &&
      status.message.abi_major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  const uint32_t required_prefix = descriptor.abi_minor == 0
      ? OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_0_SIZE
      : OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_1_SIZE;
  if (!status_header || !message_valid || status.code != OPEN_LMM_STATUS_OK_V2 ||
      descriptor.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      descriptor.struct_size < required_prefix ||
      descriptor.minimum_host_minor > OPEN_LMM_PLUGIN_ABI_V2_MINOR ||
      !ValidKnownMetadata(descriptor)) {
    close_library();
    return Result<DescriptorV2Availability>::Failure(
        Error::PluginLoadFailed("descriptor ABI-v2 query is malformed"));
  }
  if ((descriptor.capability_bits & kDescriptorCapabilities) !=
      kDescriptorCapabilities) {
    close_library();
    return Result<DescriptorV2Availability>::Ok(
        DescriptorV2Availability::kUnavailable);
  }
  close_library();
  return Result<DescriptorV2Availability>::Ok(
      DescriptorV2Availability::kAvailable);
}

Result<std::shared_ptr<DescriptorEngine>> LoadGenericDescriptorV2Adapter(
    const std::string& shared_library, const std::string& config_json) {
  auto loaded = LoadPluginV2(shared_library, "descriptor", config_json,
                             kDescriptorCapabilities);
  if (!loaded) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        loaded.GetError());
  }
  auto plugin =
      std::make_shared<PluginV2>(std::move(loaded).Value());
  const auto& metadata = plugin->Metadata();
  if (!DeclaresOperation(metadata, OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2,
                         OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2) ||
      !DeclaresOperation(metadata, OPEN_LMM_DESCRIPTOR_COMPARE_OPERATION_V2,
                         OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2) ||
      !DeclaresOperation(metadata, OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2,
                         OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2)) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        Error::PluginLoadFailed(
            "descriptor ABI-v2 canonical operation metadata is incomplete"));
  }
  if (metadata.artifact_formats.size() != 1) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        Error::PluginLoadFailed(
            "descriptor ABI-v2 plugin must advertise exactly one format"));
  }
  const auto& format = metadata.artifact_formats.front();
  PluginV2Call dimension_call{OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2,
                              nullptr, 0};
  auto dimension_response = plugin->Call(dimension_call);
  if (!dimension_response ||
      dimension_response.Value().size() <
          sizeof(open_lmm_descriptor_index_key_header_v2)) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        dimension_response
            ? Error::PluginLoadFailed(
                  "descriptor ABI-v2 index metadata response is malformed")
            : dimension_response.GetError());
  }
  open_lmm_descriptor_index_key_header_v2 dimension_header{};
  std::memcpy(&dimension_header, dimension_response.Value().data(),
              sizeof(dimension_header));
  if (dimension_header.struct_size < sizeof(dimension_header) ||
      dimension_header.struct_size != dimension_response.Value().size() ||
      dimension_header.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      dimension_header.element_type != OPEN_LMM_ELEMENT_F64_V2 ||
      (dimension_header.endian != OPEN_LMM_ENDIAN_LITTLE_V2 &&
       dimension_header.endian != OPEN_LMM_ENDIAN_BIG_V2) ||
      dimension_header.key_count == 0) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        Error::PluginLoadFailed(
            "descriptor ABI-v2 index metadata response is malformed"));
  }
  DescriptorIndexMetadata index_metadata{
      metadata.plugin_id, format.format_id, format.format_version,
      static_cast<std::size_t>(dimension_header.key_count),
      metadata.plugin_id + ":" + ConfigIdentity(config_json)};
  if (index_metadata.plugin_id.empty() || index_metadata.format_id.empty() ||
      index_metadata.format_version == 0 ||
      index_metadata.index_dimension == 0) {
    return Result<std::shared_ptr<DescriptorEngine>>::Failure(
        Error::PluginLoadFailed("descriptor ABI-v2 metadata is incomplete"));
  }
  return Result<std::shared_ptr<DescriptorEngine>>::Ok(
      std::make_shared<GenericDescriptorV2Engine>(
          std::move(plugin), std::move(index_metadata)));
}

Result<void> InspectGenericDescriptorV2Plugin(
    const std::string& shared_library, const std::string& config_json) {
  auto loaded = LoadGenericDescriptorV2Adapter(shared_library, config_json);
  if (!loaded) return Result<void>::Failure(loaded.GetError());
  return Result<void>::Ok();
}

}  // namespace open_lmm
