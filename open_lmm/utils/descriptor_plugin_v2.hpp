#pragma once

#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/common/plugin_api_v2.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <bit>
#include <cstring>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string_view>
#include <vector>

namespace open_lmm {

struct DescriptorV2PayloadHeader {
  uint64_t rows = 0;
  uint64_t columns = 0;
  uint64_t key_count = 0;
};

template <typename T>
T DescriptorV2Header() {
  T value{};
  value.struct_size = sizeof(T);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return value;
}

inline open_lmm_string_view_v2 DescriptorV2Text(const char* value) {
  auto text = DescriptorV2Header<open_lmm_string_view_v2>();
  text.data = value;
  text.size = value ? std::strlen(value) : 0;
  return text;
}

inline open_lmm_status_v2 DescriptorV2Status(open_lmm_status_code_v2 code,
                                              const char* message) {
  auto status = DescriptorV2Header<open_lmm_status_v2>();
  status.code = code;
  status.message = DescriptorV2Text(message);
  return status;
}

inline bool DescriptorV2ValidHeader(uint32_t size, uint16_t major,
                                    std::size_t required) {
  return size >= required && major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
}

template <typename Model>
class DescriptorPluginV2Dispatcher {
 public:
  struct State {
    open_lmm_host_api_v2 host{};
    std::unique_ptr<Model> model;
  };

  static open_lmm_status_v2 Call(State* state, std::string_view format_id,
                                 const open_lmm_call_v2* call,
                                 open_lmm_result_sink_v2* sink) noexcept {
    if (!state || !state->model || !call || !sink || !sink->write ||
        !DescriptorV2ValidHeader(call->struct_size, call->abi_major,
                                 OPEN_LMM_CALL_V2_MINOR_0_SIZE) ||
        !DescriptorV2ValidHeader(sink->struct_size, sink->abi_major,
                                 sizeof(*sink)) ||
        !call->operation.data) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "invalid descriptor call");
    }
    try {
      const std::string_view operation(call->operation.data,
                                       call->operation.size);
      if (operation == OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2) {
        return Make(state, format_id, call->points, sink);
      }
      if (operation == OPEN_LMM_DESCRIPTOR_COMPARE_OPERATION_V2) {
        return Compare(state, format_id, call->request, sink);
      }
      if (operation == OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2) {
        return IndexKey(state, call->request, sink);
      }
      return DescriptorV2Status(OPEN_LMM_STATUS_UNSUPPORTED_CAPABILITY_V2,
                                "unsupported descriptor operation");
    } catch (...) {
      return DescriptorV2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                                "descriptor operation failed");
    }
  }

 private:
  template <typename T>
  static T ByteSwap(T value) {
    uint8_t bytes[sizeof(T)];
    std::memcpy(bytes, &value, sizeof(T));
    std::reverse(bytes, bytes + sizeof(T));
    std::memcpy(&value, bytes, sizeof(T));
    return value;
  }

  template <typename T>
  static T ReadNumeric(const uint8_t* source, open_lmm_endian_v2 endian) {
    T value{};
    std::memcpy(&value, source, sizeof(T));
    const bool wire_little = endian == OPEN_LMM_ENDIAN_LITTLE_V2;
    const bool host_little = std::endian::native == std::endian::little;
    return wire_little == host_little ? value : ByteSwap(value);
  }

  template <typename T>
  static void WriteLittle(uint8_t* destination, T value) {
    if constexpr (std::endian::native == std::endian::big) {
      value = ByteSwap(value);
    }
    std::memcpy(destination, &value, sizeof(T));
  }
  static bool Cancelled(const State& state) {
    return state.host.is_cancelled &&
           state.host.is_cancelled(state.host.host_context) != 0;
  }

  static bool CopyPoint(const open_lmm_point_view_v2& view, uint64_t index,
                        pcl::PointXYZI* point) {
    const auto* source = static_cast<const uint8_t*>(view.data) +
                         index * view.stride_bytes;
    if (view.element_type == OPEN_LMM_ELEMENT_F32_V2) {
      point->x = ReadNumeric<float>(source, view.endian);
      point->y = ReadNumeric<float>(source + sizeof(float), view.endian);
      point->z = ReadNumeric<float>(source + 2 * sizeof(float), view.endian);
      return std::isfinite(point->x) && std::isfinite(point->y) &&
             std::isfinite(point->z);
    }
    if (view.element_type == OPEN_LMM_ELEMENT_F64_V2) {
      double xyz[3]{};
      for (int i = 0; i < 3; ++i) {
        xyz[i] = ReadNumeric<double>(source + i * sizeof(double), view.endian);
      }
      point->x = static_cast<float>(xyz[0]);
      point->y = static_cast<float>(xyz[1]);
      point->z = static_cast<float>(xyz[2]);
      return std::isfinite(xyz[0]) && std::isfinite(xyz[1]) &&
             std::isfinite(xyz[2]);
    }
    return false;
  }

  static open_lmm_status_v2 Write(open_lmm_result_sink_v2* sink,
                                  const std::vector<uint8_t>& bytes) {
    auto chunk = DescriptorV2Header<open_lmm_blob_view_v2>();
    chunk.data = bytes.data();
    chunk.size = bytes.size();
    chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
    if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2) {
      return DescriptorV2Status(OPEN_LMM_STATUS_HOST_ERROR_V2,
                                "host rejected descriptor result");
    }
    return DescriptorV2Status(OPEN_LMM_STATUS_OK_V2, "");
  }

  static std::vector<uint8_t> EncodePayload(
      const std::shared_ptr<IDescriptorKdtree>& descriptor) {
    const auto& matrix = descriptor->getDescriptor();
    const auto& key = descriptor->getDescriptorKey();
    if (matrix.rows() <= 0 || matrix.cols() <= 0 || key.size() <= 0 ||
        !matrix.allFinite() || !key.allFinite()) {
      throw std::invalid_argument("invalid descriptor payload");
    }
    DescriptorV2PayloadHeader header{
        static_cast<uint64_t>(matrix.rows()),
        static_cast<uint64_t>(matrix.cols()),
        static_cast<uint64_t>(key.size())};
    if (header.rows > std::numeric_limits<uint64_t>::max() / header.columns ||
        header.rows * header.columns >
            std::numeric_limits<uint64_t>::max() - header.key_count) {
      throw std::overflow_error("descriptor payload overflow");
    }
    const uint64_t values = header.rows * header.columns + header.key_count;
    if (values > (std::numeric_limits<std::size_t>::max() - sizeof(header)) /
                     sizeof(double)) {
      throw std::overflow_error("descriptor payload too large");
    }
    std::vector<uint8_t> bytes(sizeof(header) + values * sizeof(double));
    WriteLittle<uint64_t>(bytes.data(), header.rows);
    WriteLittle<uint64_t>(bytes.data() + sizeof(uint64_t), header.columns);
    WriteLittle<uint64_t>(bytes.data() + 2 * sizeof(uint64_t), header.key_count);
    uint8_t* cursor = bytes.data() + sizeof(header);
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
        const double value = matrix(row, column);
        WriteLittle<double>(cursor, value);
        cursor += sizeof(value);
      }
    }
    for (Eigen::Index i = 0; i < key.size(); ++i) {
      WriteLittle<double>(cursor, key(i));
      cursor += sizeof(double);
    }
    return bytes;
  }

  static std::shared_ptr<Model> DecodePayload(const State& state,
                                               const uint8_t* bytes,
                                               uint64_t size) {
    if (!bytes || size < sizeof(DescriptorV2PayloadHeader)) {
      throw std::invalid_argument("descriptor payload is truncated");
    }
    DescriptorV2PayloadHeader header{};
    header.rows = ReadNumeric<uint64_t>(bytes, OPEN_LMM_ENDIAN_LITTLE_V2);
    header.columns = ReadNumeric<uint64_t>(bytes + sizeof(uint64_t),
                                           OPEN_LMM_ENDIAN_LITTLE_V2);
    header.key_count = ReadNumeric<uint64_t>(bytes + 2 * sizeof(uint64_t),
                                             OPEN_LMM_ENDIAN_LITTLE_V2);
    if (header.rows == 0 || header.columns == 0 || header.key_count == 0 ||
        header.rows > std::numeric_limits<uint64_t>::max() / header.columns ||
        header.rows * header.columns >
            std::numeric_limits<uint64_t>::max() - header.key_count) {
      throw std::invalid_argument("descriptor payload header is malformed");
    }
    const uint64_t values = header.rows * header.columns + header.key_count;
    if (values > (std::numeric_limits<uint64_t>::max() - sizeof(header)) /
                     sizeof(double) ||
        size != sizeof(header) + values * sizeof(double) ||
        header.rows > static_cast<uint64_t>(
                          std::numeric_limits<Eigen::Index>::max()) ||
        header.columns > static_cast<uint64_t>(
                             std::numeric_limits<Eigen::Index>::max()) ||
        header.key_count > static_cast<uint64_t>(
                               std::numeric_limits<Eigen::Index>::max())) {
      throw std::invalid_argument("descriptor payload length mismatch");
    }
    Eigen::MatrixXd matrix(static_cast<Eigen::Index>(header.rows),
                           static_cast<Eigen::Index>(header.columns));
    Eigen::VectorXd key(static_cast<Eigen::Index>(header.key_count));
    const uint8_t* cursor = bytes + sizeof(header);
    for (Eigen::Index row = 0; row < matrix.rows(); ++row) {
      for (Eigen::Index column = 0; column < matrix.cols(); ++column) {
        matrix(row, column) =
            ReadNumeric<double>(cursor, OPEN_LMM_ENDIAN_LITTLE_V2);
        cursor += sizeof(double);
      }
    }
    for (Eigen::Index i = 0; i < key.size(); ++i) {
      key(i) = ReadNumeric<double>(cursor, OPEN_LMM_ENDIAN_LITTLE_V2);
      cursor += sizeof(double);
    }
    if (!matrix.allFinite() || !key.allFinite()) {
      throw std::invalid_argument("descriptor payload is non-finite");
    }
    return Model::FromWire(state.model->getParams(), std::move(matrix),
                           std::move(key));
  }

  static open_lmm_status_v2 Make(State* state, std::string_view format_id,
                                 const open_lmm_point_view_v2& points,
                                 open_lmm_result_sink_v2* sink) {
    const uint64_t width = points.element_type == OPEN_LMM_ELEMENT_F32_V2
                               ? sizeof(float)
                           : points.element_type == OPEN_LMM_ELEMENT_F64_V2
                               ? sizeof(double)
                               : 0;
    if (!DescriptorV2ValidHeader(points.struct_size, points.abi_major,
                                 sizeof(points)) ||
        points.memory_location != OPEN_LMM_MEMORY_HOST_V2 ||
        (points.endian != OPEN_LMM_ENDIAN_LITTLE_V2 &&
         points.endian != OPEN_LMM_ENDIAN_BIG_V2) || width == 0 ||
        points.count == 0 || !points.data ||
        points.stride_bytes < 3 * width ||
        points.stride_bytes >
            std::numeric_limits<uint64_t>::max() / points.count ||
        points.count > std::numeric_limits<uint32_t>::max()) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "invalid descriptor point view");
    }
    if (Cancelled(*state)) {
      return DescriptorV2Status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
    }
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.resize(static_cast<std::size_t>(points.count));
    for (uint64_t index = 0; index < points.count; ++index) {
      if ((index & UINT64_C(4095)) == 0 && Cancelled(*state)) {
        return DescriptorV2Status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
      }
      if (!CopyPoint(points, index, &cloud->points[index])) {
        return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                  "invalid descriptor point record");
      }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    auto generated = state->model->makeDescriptor(cloud);
    if (!generated || generated->getDescriptorKey().size() <= 0) {
      return DescriptorV2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                                "descriptor generation returned empty");
    }
    const auto& key = generated->getDescriptorKey();
    auto payload = EncodePayload(generated);
    auto header = DescriptorV2Header<open_lmm_descriptor_artifact_header_v2>();
    header.format_id_size = format_id.size();
    header.format_version = 1;
    header.key_count = static_cast<uint64_t>(key.size());
    header.element_type = OPEN_LMM_ELEMENT_F64_V2;
    header.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
    header.payload_size = payload.size();
    const std::size_t maximum = std::numeric_limits<std::size_t>::max();
    if (format_id.size() > maximum - sizeof(header) ||
        payload.size() > maximum - sizeof(header) - format_id.size() ||
        header.key_count >
            (maximum - sizeof(header) - format_id.size() - payload.size()) /
                sizeof(double)) {
      return DescriptorV2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                                "descriptor artifact size overflow");
    }
    std::vector<uint8_t> bytes(sizeof(header) + format_id.size() +
                               key.size() * sizeof(double) + payload.size());
    uint8_t* cursor = bytes.data();
    std::memcpy(cursor, &header, sizeof(header));
    cursor += sizeof(header);
    std::memcpy(cursor, format_id.data(), format_id.size());
    cursor += format_id.size();
    for (Eigen::Index index = 0; index < key.size(); ++index) {
      const double value = key(index);
      if (!std::isfinite(value)) {
        return DescriptorV2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                                  "descriptor key is non-finite");
      }
      WriteLittle<double>(cursor, value);
      cursor += sizeof(value);
    }
    std::memcpy(cursor, payload.data(), payload.size());
    return Write(sink, bytes);
  }

  static open_lmm_status_v2 Compare(State* state, std::string_view format_id,
                                    const open_lmm_blob_view_v2& request,
                                    open_lmm_result_sink_v2* sink) {
    if (request.memory_location != OPEN_LMM_MEMORY_HOST_V2 || !request.data ||
        request.size < sizeof(open_lmm_descriptor_compare_request_v2)) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "invalid descriptor compare request");
    }
    open_lmm_descriptor_compare_request_v2 header{};
    std::memcpy(&header, request.data, sizeof(header));
    if (!DescriptorV2ValidHeader(header.struct_size, header.abi_major,
                                 sizeof(header)) ||
        header.format_version != 1 ||
        header.format_id_size != format_id.size() ||
        header.lhs_payload_size == 0 || header.rhs_payload_size == 0 ||
        header.struct_size > request.size ||
        header.format_id_size > request.size - header.struct_size ||
        header.lhs_payload_size >
            request.size - header.struct_size - header.format_id_size ||
        header.rhs_payload_size !=
            request.size - header.struct_size - header.format_id_size -
                header.lhs_payload_size) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "malformed descriptor compare request");
    }
    const auto* cursor = static_cast<const uint8_t*>(request.data) +
                         header.struct_size;
    if (std::memcmp(cursor, format_id.data(), format_id.size()) != 0) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "descriptor compare format mismatch");
    }
    cursor += format_id.size();
    auto lhs = DecodePayload(*state, cursor, header.lhs_payload_size);
    auto rhs = DecodePayload(*state, cursor + header.lhs_payload_size,
                             header.rhs_payload_size);
    if (Cancelled(*state)) {
      return DescriptorV2Status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
    }
    const auto [score, pose] = lhs->distance(rhs);
    auto result = DescriptorV2Header<open_lmm_descriptor_compare_result_v2>();
    result.score = score;
    result.element_type = OPEN_LMM_ELEMENT_F64_V2;
    result.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
    const Eigen::Matrix4d matrix = pose.matrix();
    for (Eigen::Index row = 0; row < 4; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        result.relative_pose_row_major[row * 4 + column] = matrix(row, column);
      }
    }
    std::vector<uint8_t> bytes(sizeof(result));
    std::memcpy(bytes.data(), &result, sizeof(result));
    WriteLittle<double>(bytes.data() + offsetof(
                            open_lmm_descriptor_compare_result_v2, score),
                        score);
    uint8_t* pose_bytes = bytes.data() + offsetof(
        open_lmm_descriptor_compare_result_v2, relative_pose_row_major);
    for (Eigen::Index row = 0; row < 4; ++row) {
      for (Eigen::Index column = 0; column < 4; ++column) {
        WriteLittle<double>(pose_bytes + (row * 4 + column) * sizeof(double),
                            matrix(row, column));
      }
    }
    return Write(sink, bytes);
  }

  static open_lmm_status_v2 IndexKey(State* state,
                                     const open_lmm_blob_view_v2& request,
                                     open_lmm_result_sink_v2* sink) {
    if (request.memory_location != OPEN_LMM_MEMORY_HOST_V2) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "invalid descriptor index request");
    }
    if (request.size == 0) {
      auto header =
          DescriptorV2Header<open_lmm_descriptor_index_key_header_v2>();
      header.key_count = state->model->getParams().descriptor_vector_dim;
      header.element_type = OPEN_LMM_ELEMENT_F64_V2;
      header.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
      std::vector<uint8_t> bytes(sizeof(header));
      std::memcpy(bytes.data(), &header, sizeof(header));
      return Write(sink, bytes);
    }
    if (!request.data) {
      return DescriptorV2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                                "invalid descriptor index request");
    }
    auto descriptor = DecodePayload(
        *state, static_cast<const uint8_t*>(request.data), request.size);
    const auto& key = descriptor->getDescriptorKey();
    auto header =
        DescriptorV2Header<open_lmm_descriptor_index_key_header_v2>();
    header.key_count = static_cast<uint64_t>(key.size());
    header.element_type = OPEN_LMM_ELEMENT_F64_V2;
    header.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
    std::vector<uint8_t> bytes(sizeof(header) + key.size() * sizeof(double));
    std::memcpy(bytes.data(), &header, sizeof(header));
    uint8_t* key_bytes = bytes.data() + sizeof(header);
    for (Eigen::Index i = 0; i < key.size(); ++i) {
      WriteLittle<double>(key_bytes + i * sizeof(double), key(i));
    }
    return Write(sink, bytes);
  }
};

}  // namespace open_lmm
