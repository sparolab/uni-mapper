#pragma once

#include <open_lmm/common/plugin_api_v2.h>
#include <open_lmm/utils/config.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <Eigen/Geometry>

#include <cstring>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace open_lmm::remover_v2_support {

template <typename T>
T Header() {
  T value{};
  value.struct_size = sizeof(T);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return value;
}

inline open_lmm_string_view_v2 String(const char* value) {
  auto view = Header<open_lmm_string_view_v2>();
  view.data = value;
  view.size = value ? std::strlen(value) : 0;
  return view;
}

inline open_lmm_status_v2 Status(open_lmm_status_code_v2 code,
                                 const char* message = "") {
  auto status = Header<open_lmm_status_v2>();
  status.code = code;
  status.message = String(message);
  return status;
}

inline bool Operation(const open_lmm_call_v2* call, const char* expected) {
  return call && call->struct_size >= OPEN_LMM_CALL_V2_MINOR_1_SIZE &&
         call->abi_major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR &&
         call->operation.data &&
         call->operation.size == std::strlen(expected) &&
         std::memcmp(call->operation.data, expected, call->operation.size) == 0;
}

inline bool ByteRangeFits(uint64_t count, uint64_t stride,
                          std::size_t record_size) {
  if (count == 0) return true;
  if (stride == 0 || stride > std::numeric_limits<std::size_t>::max() ||
      count - 1 >
          (std::numeric_limits<std::size_t>::max() - record_size) / stride) {
    return false;
  }
  return true;
}

template <typename Interface, typename Implementation, typename Params,
          bool Online>
struct Handle {
  open_lmm_host_api_v2 host{};
  std::string config_json;
  std::unique_ptr<Interface> model;
  std::unordered_map<uint64_t, Eigen::Isometry3d> poses;
  std::unordered_map<uint64_t, bool> pushed;
  bool active = false;
};

template <typename Handle>
bool Cancelled(const Handle& handle) {
  return handle.host.is_cancelled &&
         handle.host.is_cancelled(handle.host.host_context) != 0;
}

template <typename Interface, typename Implementation, typename Params,
          bool Online>
open_lmm_status_v2 Open(const open_lmm_host_api_v2* host,
                        const open_lmm_config_view_v2* config,
                        open_lmm_plugin_handle_v2** output) noexcept {
  using State = Handle<Interface, Implementation, Params, Online>;
  if (!output || !host || !config ||
      host->struct_size < sizeof(*host) ||
      host->abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      config->struct_size < sizeof(*config) ||
      config->abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      config->json.struct_size < sizeof(config->json) ||
      config->json.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      config->json.size > 16U * 1024U * 1024U ||
      (config->json.size && !config->json.data)) {
    return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid open input");
  }
  try {
    auto state = std::make_unique<State>();
    state->host = *host;
    state->config_json.assign(config->json.data ? config->json.data : "",
                              static_cast<std::size_t>(config->json.size));
    *output = reinterpret_cast<open_lmm_plugin_handle_v2*>(state.release());
    return Status(OPEN_LMM_STATUS_OK_V2);
  } catch (...) {
    return Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "remover open failed");
  }
}

template <typename Interface, typename Implementation, typename Params,
          bool Online>
open_lmm_status_v2 Call(open_lmm_plugin_handle_v2* opaque,
                        const open_lmm_call_v2* call,
                        open_lmm_result_sink_v2* sink) noexcept {
  using State = Handle<Interface, Implementation, Params, Online>;
  auto* state = reinterpret_cast<State*>(opaque);
  if (!state || !call) {
    return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "invalid remover call");
  }
  try {
    if (Operation(call, OPEN_LMM_REMOVER_ABORT_OPERATION_V2)) {
      state->model.reset();
      state->poses.clear();
      state->pushed.clear();
      state->active = false;
      return Status(OPEN_LMM_STATUS_OK_V2);
    }
    if (Cancelled(*state)) {
      return Status(OPEN_LMM_STATUS_CANCELLED_V2, "remover cancelled");
    }
    if (Operation(call, OPEN_LMM_REMOVER_BEGIN_OPERATION_V2)) {
      const auto& indexed = call->indexed_poses;
      if (state->active || indexed.struct_size < sizeof(indexed) ||
          indexed.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
          indexed.poses.struct_size < sizeof(indexed.poses) ||
          indexed.poses.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
          indexed.poses.memory_location != OPEN_LMM_MEMORY_HOST_V2 ||
          indexed.frame_count != indexed.poses.count ||
          (indexed.frame_count && !indexed.frame_ids) ||
          (indexed.frame_count && !indexed.poses.data) ||
          indexed.poses.element_type != OPEN_LMM_ELEMENT_F64_V2 ||
          indexed.poses.endian != OPEN_LMM_ENDIAN_LITTLE_V2 ||
          indexed.poses.stride_bytes < 16 * sizeof(double) ||
          !ByteRangeFits(indexed.frame_count, sizeof(uint64_t),
                         sizeof(uint64_t)) ||
          !ByteRangeFits(indexed.frame_count, indexed.poses.stride_bytes,
                         16 * sizeof(double))) {
        return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                      "invalid indexed poses");
      }
      Config config = Config::FromJson(state->config_json, "remover-v2");
      if (!config.is_valid()) {
        return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                      "invalid remover config");
      }
      state->model = std::make_unique<Implementation>(Params(config));
      const auto* pose_bytes =
          static_cast<const uint8_t*>(indexed.poses.data);
      for (uint64_t i = 0; i < indexed.frame_count; ++i) {
        double matrix_values[16];
        std::memcpy(matrix_values,
                    pose_bytes + i * indexed.poses.stride_bytes,
                    sizeof(matrix_values));
        Eigen::Matrix4d matrix;
        for (int row = 0; row < 4; ++row) {
          for (int col = 0; col < 4; ++col) {
            matrix(row, col) = matrix_values[row * 4 + col];
            if (!std::isfinite(matrix(row, col))) {
              throw std::invalid_argument("non-finite pose");
            }
          }
        }
        if (!matrix.bottomRows<1>().isApprox(
                Eigen::RowVector4d(0, 0, 0, 1), 1e-9) ||
            !matrix.topLeftCorner<3, 3>().transpose()
                 .isApprox(matrix.topLeftCorner<3, 3>().inverse(), 1e-6) ||
            matrix.topLeftCorner<3, 3>().determinant() <= 0) {
          throw std::invalid_argument("non-rigid pose");
        }
        if (!state->poses.emplace(indexed.frame_ids[i],
                                  Eigen::Isometry3d(matrix)).second) {
          state->model.reset();
          state->poses.clear();
          return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                        "duplicate pose frame");
        }
      }
      state->active = true;
      return Status(OPEN_LMM_STATUS_OK_V2);
    }
    if (Operation(call, OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2)) {
      const auto& frame = call->frame_points;
      const auto& points = frame.points;
      auto pose_it = state->poses.find(frame.frame_id);
      if (!state->active || !state->model || pose_it == state->poses.end() ||
          state->pushed.count(frame.frame_id) != 0 ||
          frame.struct_size < sizeof(frame) ||
          frame.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
          points.struct_size < sizeof(points) ||
          points.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
          points.memory_location != OPEN_LMM_MEMORY_HOST_V2 ||
          points.element_type != OPEN_LMM_ELEMENT_F32_V2 ||
          points.endian != OPEN_LMM_ENDIAN_LITTLE_V2 ||
          points.stride_bytes < 4 * sizeof(float) ||
          points.count > std::numeric_limits<std::size_t>::max() ||
          !ByteRangeFits(points.count, points.stride_bytes,
                         4 * sizeof(float)) ||
          (points.count && !points.data)) {
        return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                      "invalid scan frame");
      }
      auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
      cloud->reserve(static_cast<std::size_t>(points.count));
      const auto* bytes = static_cast<const uint8_t*>(points.data);
      for (uint64_t i = 0; i < points.count; ++i) {
        float values[4];
        std::memcpy(values, bytes + i * points.stride_bytes, sizeof(values));
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
            !std::isfinite(values[2]) || !std::isfinite(values[3])) {
          return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                        "non-finite scan point");
        }
        pcl::PointXYZI point;
        point.x = values[0]; point.y = values[1]; point.z = values[2];
        point.intensity = values[3];
        cloud->push_back(point);
      }
      state->pushed.emplace(frame.frame_id, true);
      Eigen::Isometry3d pose = pose_it->second;
      if constexpr (Online) {
        (void)state->model->run(cloud, pose);
      } else {
        state->model->run(cloud, pose);
      }
      return Status(OPEN_LMM_STATUS_OK_V2);
    }
    if (Operation(call, OPEN_LMM_REMOVER_FINISH_OPERATION_V2)) {
      if (!state->active || !state->model ||
          state->pushed.size() != state->poses.size() || !sink || !sink->write) {
        return Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                      "incomplete remover stream");
      }
      auto cloud = state->model->getStaticMap();
      if (!cloud) {
        return Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "null remover output");
      }
      auto header = Header<open_lmm_point_cloud_header_v2>();
      if (cloud->size() >
          (std::numeric_limits<std::size_t>::max() - sizeof(header)) /
              (4 * sizeof(float))) {
        return Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "remover output is too large");
      }
      header.count = cloud->size();
      header.stride_bytes = 4 * sizeof(float);
      header.component_count = 4;
      header.element_type = OPEN_LMM_ELEMENT_F32_V2;
      header.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
      std::vector<uint8_t> result(sizeof(header) +
                                  cloud->size() * header.stride_bytes);
      std::memcpy(result.data(), &header, sizeof(header));
      uint8_t* output = result.data() + sizeof(header);
      for (const auto& point : *cloud) {
        const float values[4] = {point.x, point.y, point.z, point.intensity};
        if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
            !std::isfinite(values[2]) || !std::isfinite(values[3])) {
          return Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                        "non-finite remover output");
        }
        std::memcpy(output, values, sizeof(values));
        output += sizeof(values);
      }
      auto chunk = Header<open_lmm_blob_view_v2>();
      chunk.data = result.data();
      chunk.size = result.size();
      chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
      if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2) {
        return Status(OPEN_LMM_STATUS_HOST_ERROR_V2, "sink rejected cloud");
      }
      state->model.reset();
      state->poses.clear();
      state->pushed.clear();
      state->active = false;
      return Status(OPEN_LMM_STATUS_OK_V2);
    }
    return Status(OPEN_LMM_STATUS_UNSUPPORTED_CAPABILITY_V2,
                  "unknown remover operation");
  } catch (...) {
    return Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2, "remover call failed");
  }
}

template <typename Interface, typename Implementation, typename Params,
          bool Online>
void Close(open_lmm_plugin_handle_v2* opaque) noexcept {
  using State = Handle<Interface, Implementation, Params, Online>;
  delete reinterpret_cast<State*>(opaque);
}

inline void FillDescriptor(open_lmm_plugin_descriptor_v2* descriptor,
                           const char* name, uint64_t mode) {
  static const open_lmm_operation_descriptor_v2 operations[] = {
      {sizeof(open_lmm_operation_descriptor_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       {sizeof(open_lmm_string_view_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
        OPEN_LMM_REMOVER_BEGIN_OPERATION_V2,
        sizeof(OPEN_LMM_REMOVER_BEGIN_OPERATION_V2) - 1},
       OPEN_LMM_CAPABILITY_POSE_VIEW_V2},
      {sizeof(open_lmm_operation_descriptor_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       {sizeof(open_lmm_string_view_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
        OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2,
        sizeof(OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2) - 1},
       OPEN_LMM_CAPABILITY_POINT_VIEW_V2},
      {sizeof(open_lmm_operation_descriptor_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       {sizeof(open_lmm_string_view_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
        OPEN_LMM_REMOVER_FINISH_OPERATION_V2,
        sizeof(OPEN_LMM_REMOVER_FINISH_OPERATION_V2) - 1}, 0},
      {sizeof(open_lmm_operation_descriptor_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
       {sizeof(open_lmm_string_view_v2), OPEN_LMM_PLUGIN_ABI_V2_MAJOR, OPEN_LMM_PLUGIN_ABI_V2_MINOR,
        OPEN_LMM_REMOVER_ABORT_OPERATION_V2,
        sizeof(OPEN_LMM_REMOVER_ABORT_OPERATION_V2) - 1}, 0}};
  *descriptor = Header<open_lmm_plugin_descriptor_v2>();
  descriptor->plugin_kind = String("dynamic_remover");
  descriptor->plugin_name = String(name);
  descriptor->capability = String("dynamic_remover:v2");
  descriptor->capability_bits = OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
                                OPEN_LMM_CAPABILITY_POSE_VIEW_V2 | mode;
  descriptor->minimum_host_minor = 1;
  descriptor->plugin_id = String(
      std::strcmp(name, "otd") == 0 ? "open_lmm.remover.otd"
                                     : "open_lmm.remover.free_dom");
  descriptor->plugin_version = String("1.0");
  descriptor->operations = operations;
  descriptor->operation_count = sizeof(operations) / sizeof(operations[0]);
  descriptor->schema_id = String("");
  descriptor->thread_safety = OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2;
  descriptor->cancellation = OPEN_LMM_CANCELLATION_COOPERATIVE_V2;
}

}  // namespace open_lmm::remover_v2_support
