#pragma once

#include <open_lmm/common/result.hpp>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace open_lmm {

class DescriptorOpaquePayload {
 public:
  virtual ~DescriptorOpaquePayload() = default;
};

class DescriptorArtifact {
 public:
  static Result<DescriptorArtifact> Create(
      std::string plugin_id, std::string format_id, uint32_t format_version,
      std::vector<double> index_key,
      std::shared_ptr<const DescriptorOpaquePayload> opaque_payload);

  [[nodiscard]] const std::string& plugin_id() const { return plugin_id_; }
  [[nodiscard]] const std::string& format_id() const { return format_id_; }
  [[nodiscard]] uint32_t format_version() const { return format_version_; }
  [[nodiscard]] const std::vector<double>& index_key() const {
    return index_key_;
  }

 private:
  DescriptorArtifact(std::string plugin_id, std::string format_id,
                     uint32_t format_version, std::vector<double> index_key,
                     std::shared_ptr<const DescriptorOpaquePayload> payload);

  std::string plugin_id_;
  std::string format_id_;
  uint32_t format_version_ = 0;
  std::vector<double> index_key_;
  std::shared_ptr<const DescriptorOpaquePayload> opaque_payload_;

  friend class BuiltInDescriptorEngine;
};

}  // namespace open_lmm
