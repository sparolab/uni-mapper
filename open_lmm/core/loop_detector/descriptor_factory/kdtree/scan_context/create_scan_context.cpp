#include "scan_context.h"
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/common/plugin_api_v2.h>
#include <open_lmm/utils/plugin_support.hpp>

#include <cstring>
#include <limits>
#include <memory>
#include <string>
#include <vector>

struct open_lmm_plugin_handle_v2 {
  open_lmm_host_api_v2 host{};
  std::unique_ptr<ScanContext> model;
};

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IDescriptorKdtree, ScanContext, ScanContextParams>(config,
                                                        "scan_context");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IDescriptorKdtree>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "descriptor", "scan_context",
    &Create, &Destroy, "descriptor:kdtree", 1, "open-lmm-1.0"};

template <typename T>
T V2Header() {
  T value{};
  value.struct_size = sizeof(T);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return value;
}

open_lmm_string_view_v2 V2Text(const char* value) {
  auto text = V2Header<open_lmm_string_view_v2>();
  text.data = value;
  text.size = value ? std::strlen(value) : 0;
  return text;
}

open_lmm_status_v2 V2Status(open_lmm_status_code_v2 code,
                            const char* message) {
  auto status = V2Header<open_lmm_status_v2>();
  status.code = code;
  status.message = V2Text(message);
  return status;
}

bool ValidV2Header(uint32_t size, uint16_t major,
                   std::size_t required) {
  return size >= required && major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
}

bool IsCancelled(const open_lmm_plugin_handle_v2& handle) {
  return handle.host.is_cancelled &&
         handle.host.is_cancelled(handle.host.host_context) != 0;
}

bool CopyPoint(const open_lmm_point_view_v2& view, uint64_t index,
               pcl::PointXYZI* point) {
  const auto* source = static_cast<const uint8_t*>(view.data) +
                       index * view.stride_bytes;
  if (view.element_type == OPEN_LMM_ELEMENT_F32_V2) {
    std::memcpy(&point->x, source, sizeof(float));
    std::memcpy(&point->y, source + sizeof(float), sizeof(float));
    std::memcpy(&point->z, source + 2 * sizeof(float), sizeof(float));
    return true;
  }
  if (view.element_type == OPEN_LMM_ELEMENT_F64_V2) {
    double xyz[3]{};
    std::memcpy(xyz, source, sizeof(xyz));
    point->x = static_cast<float>(xyz[0]);
    point->y = static_cast<float>(xyz[1]);
    point->z = static_cast<float>(xyz[2]);
    return true;
  }
  return false;
}
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_query_v2(open_lmm_plugin_descriptor_v2* descriptor) {
  if (!descriptor) {
    return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2, "descriptor is null");
  }
  *descriptor = V2Header<open_lmm_plugin_descriptor_v2>();
  descriptor->plugin_kind = V2Text("descriptor");
  descriptor->plugin_name = V2Text("scan_context");
  descriptor->capability = V2Text("descriptor.make:point-view");
  descriptor->capability_bits = OPEN_LMM_CAPABILITY_POINT_VIEW_V2;
  descriptor->minimum_host_minor = 0;
  return V2Status(OPEN_LMM_STATUS_OK_V2, "");
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_open_v2(const open_lmm_host_api_v2* host,
                        const open_lmm_config_view_v2* config,
                        open_lmm_plugin_handle_v2** out) {
  if (!host || !config || !out ||
      !ValidV2Header(host->struct_size, host->abi_major, sizeof(*host)) ||
      !ValidV2Header(config->struct_size, config->abi_major, sizeof(*config)) ||
      !ValidV2Header(config->json.struct_size, config->json.abi_major,
                     sizeof(config->json)) ||
      (config->json.size != 0 && !config->json.data) ||
      config->json.size > std::numeric_limits<std::size_t>::max()) {
    return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                    "invalid Scan Context open arguments");
  }
  *out = nullptr;
  try {
    const std::string json(config->json.data ? config->json.data : "",
                           static_cast<std::size_t>(config->json.size));
    auto parsed = open_lmm::Config::FromJson(json, "scan_context ABI-v2");
    if (!parsed.is_valid()) {
      return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                      "invalid Scan Context JSON");
    }
    auto handle = std::make_unique<open_lmm_plugin_handle_v2>();
    handle->host = *host;
    handle->model = std::make_unique<ScanContext>(ScanContextParams(parsed));
    *out = handle.release();
    return V2Status(OPEN_LMM_STATUS_OK_V2, "");
  } catch (...) {
    return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                    "failed to create Scan Context ABI-v2 plugin");
  }
}

extern "C" open_lmm_status_v2 OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_call_v2(open_lmm_plugin_handle_v2* handle,
                        const open_lmm_call_v2* call,
                        open_lmm_result_sink_v2* sink) {
  if (!handle || !handle->model || !call || !sink || !sink->write ||
      !ValidV2Header(call->struct_size, call->abi_major, sizeof(*call)) ||
      !ValidV2Header(sink->struct_size, sink->abi_major, sizeof(*sink)) ||
      call->operation.size !=
          sizeof(OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2) - 1 ||
      !call->operation.data ||
      std::memcmp(call->operation.data, OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2,
                  sizeof(OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2) - 1) != 0) {
    return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                    "unsupported Scan Context operation");
  }
  const auto& points = call->points;
  const uint64_t width = points.element_type == OPEN_LMM_ELEMENT_F32_V2
                             ? sizeof(float)
                         : points.element_type == OPEN_LMM_ELEMENT_F64_V2
                             ? sizeof(double)
                             : 0;
  if (!ValidV2Header(points.struct_size, points.abi_major, sizeof(points)) ||
      points.memory_location != OPEN_LMM_MEMORY_HOST_V2 ||
      points.endian != OPEN_LMM_ENDIAN_LITTLE_V2 || width == 0 ||
      points.count == 0 || !points.data || points.stride_bytes < 3 * width ||
      points.stride_bytes >
          std::numeric_limits<uint64_t>::max() / points.count ||
      points.count > std::numeric_limits<std::size_t>::max() ||
      points.count > std::numeric_limits<uint32_t>::max()) {
    return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                    "invalid Scan Context point view");
  }
  if (IsCancelled(*handle)) {
    return V2Status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
  }
  try {
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    cloud->points.resize(static_cast<std::size_t>(points.count));
    for (uint64_t index = 0; index < points.count; ++index) {
      if ((index & UINT64_C(4095)) == 0 && IsCancelled(*handle)) {
        return V2Status(OPEN_LMM_STATUS_CANCELLED_V2, "requested");
      }
      if (!CopyPoint(points, index, &cloud->points[index])) {
        return V2Status(OPEN_LMM_STATUS_INVALID_ARGUMENT_V2,
                        "unsupported Scan Context point type");
      }
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    auto generated = handle->model->makeDescriptor(cloud);
    const auto& descriptor = generated->getDescriptor();
    const auto& key = generated->getDescriptorKey();
    if (descriptor.rows() <= 0 || descriptor.cols() <= 0 || key.size() <= 0) {
      return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "Scan Context produced an empty descriptor");
    }
    const uint64_t rows = static_cast<uint64_t>(descriptor.rows());
    const uint64_t columns = static_cast<uint64_t>(descriptor.cols());
    const uint64_t key_count = static_cast<uint64_t>(key.size());
    if (rows > std::numeric_limits<uint64_t>::max() / columns) {
      return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "Scan Context descriptor size overflow");
    }
    const uint64_t descriptor_count =
        rows * columns;
    if (descriptor_count >
        std::numeric_limits<uint64_t>::max() - key_count) {
      return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "Scan Context result size overflow");
    }
    const uint64_t value_count = descriptor_count + key_count;
    if (value_count >
        (std::numeric_limits<std::size_t>::max() -
         sizeof(open_lmm_descriptor_result_v2)) / sizeof(double)) {
      return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                      "Scan Context result is too large");
    }
    auto header = V2Header<open_lmm_descriptor_result_v2>();
    header.rows = rows;
    header.columns = columns;
    header.key_count = key_count;
    header.element_type = OPEN_LMM_ELEMENT_F64_V2;
    header.endian = OPEN_LMM_ENDIAN_LITTLE_V2;
    std::vector<uint8_t> bytes(
        sizeof(header) + static_cast<std::size_t>(value_count) * sizeof(double));
    std::memcpy(bytes.data(), &header, sizeof(header));
    uint8_t* cursor = bytes.data() + sizeof(header);
    for (Eigen::Index row = 0; row < descriptor.rows(); ++row) {
      for (Eigen::Index column = 0; column < descriptor.cols(); ++column) {
        const double value = descriptor(row, column);
        std::memcpy(cursor, &value, sizeof(value));
        cursor += sizeof(value);
      }
    }
    for (Eigen::Index index = 0; index < key.size(); ++index) {
      const double value = key(index);
      std::memcpy(cursor, &value, sizeof(value));
      cursor += sizeof(value);
    }
    auto chunk = V2Header<open_lmm_blob_view_v2>();
    chunk.data = bytes.data();
    chunk.size = bytes.size();
    chunk.memory_location = OPEN_LMM_MEMORY_HOST_V2;
    if (sink->write(sink->host_context, &chunk) != OPEN_LMM_STATUS_OK_V2) {
      return V2Status(OPEN_LMM_STATUS_HOST_ERROR_V2,
                      "host rejected Scan Context result");
    }
    return V2Status(OPEN_LMM_STATUS_OK_V2, "");
  } catch (...) {
    return V2Status(OPEN_LMM_STATUS_PLUGIN_ERROR_V2,
                    "Scan Context descriptor generation failed");
  }
}

extern "C" void OPEN_LMM_PLUGIN_CALL_V2
open_lmm_plugin_close_v2(open_lmm_plugin_handle_v2* handle) {
  delete handle;
}
