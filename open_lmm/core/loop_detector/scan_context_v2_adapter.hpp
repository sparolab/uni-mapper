#pragma once

#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/common/result.hpp>

#include <memory>
#include <string>

namespace open_lmm {

Result<std::shared_ptr<IDescriptorKdtree>> LoadScanContextV2Adapter(
    const std::string& shared_library, const std::string& config_json);

Result<void> InspectScanContextV2Plugin(
    const std::string& shared_library, const std::string& config_json);

}  // namespace open_lmm
