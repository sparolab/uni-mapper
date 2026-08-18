#pragma once

#include <open_lmm/common/plugin_api_v2.h>
#include <open_lmm/common/result.hpp>

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm {

struct PluginV2Metadata {
  std::string kind;
  std::string name;
  std::string capability;
  uint64_t capability_bits = 0;
  uint16_t abi_minor = 0;
};

struct PluginPointView {
  const void* data = nullptr;
  uint64_t count = 0;
  uint64_t stride_bytes = 0;
  open_lmm_element_type_v2 element_type = OPEN_LMM_ELEMENT_F32_V2;
  open_lmm_endian_v2 endian = OPEN_LMM_ENDIAN_LITTLE_V2;
};

struct PluginPoseView {
  const void* data = nullptr;
  uint64_t count = 0;
  uint64_t stride_bytes = 0;
  open_lmm_element_type_v2 element_type = OPEN_LMM_ELEMENT_F64_V2;
  open_lmm_endian_v2 endian = OPEN_LMM_ENDIAN_LITTLE_V2;
};

struct PluginV2ResultLimits {
  uint64_t maximum_result_bytes = 64U * 1024U * 1024U;
  uint64_t maximum_chunk_bytes = 8U * 1024U * 1024U;
  uint64_t maximum_chunk_count = 1024;
};

struct PluginV2Call {
  std::string_view operation;
  const void* request_data = nullptr;
  uint64_t request_size = 0;
  std::optional<PluginPointView> points;
  std::optional<PluginPoseView> poses;
  PluginV2ResultLimits result_limits;
};

class PluginV2 {
 public:
  ~PluginV2();
  PluginV2(PluginV2&&) noexcept;
  PluginV2& operator=(PluginV2&&) noexcept;
  PluginV2(const PluginV2&) = delete;
  PluginV2& operator=(const PluginV2&) = delete;

  [[nodiscard]] const PluginV2Metadata& Metadata() const noexcept;
  Result<std::vector<uint8_t>> Call(const PluginV2Call& call);

 private:
  struct Impl;
  explicit PluginV2(std::unique_ptr<Impl> impl);
  std::unique_ptr<Impl> impl_;
  friend Result<PluginV2> LoadPluginV2(const std::string&, std::string_view,
                                       std::string_view, uint64_t, void*,
                                       open_lmm_is_cancelled_fn_v2);
};

Result<PluginV2> LoadPluginV2(
    const std::string& shared_library, std::string_view expected_kind,
    std::string_view config_json, uint64_t required_capability_bits = 0,
    void* host_context = nullptr,
    open_lmm_is_cancelled_fn_v2 is_cancelled = nullptr);

}  // namespace open_lmm
