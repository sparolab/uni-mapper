#pragma once

#include <open_lmm/core/descriptor/descriptor_engine.hpp>

#include <memory>
#include <string>

namespace open_lmm {

enum class DescriptorV2Availability { kAvailable, kUnavailable };

Result<DescriptorV2Availability> ProbeGenericDescriptorV2Plugin(
    const std::string& shared_library);

// Loads a model-independent descriptor engine backed entirely by the stable
// ABI-v2 make/compare operations. No descriptor mathematics run in the host.
Result<std::shared_ptr<DescriptorEngine>> LoadGenericDescriptorV2Adapter(
    const std::string& shared_library, const std::string& config_json);

Result<void> InspectGenericDescriptorV2Plugin(
    const std::string& shared_library, const std::string& config_json);

}  // namespace open_lmm
