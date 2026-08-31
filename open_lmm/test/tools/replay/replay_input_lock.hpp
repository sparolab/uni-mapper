#pragma once

#include <filesystem>

#include <nlohmann/json.hpp>

namespace open_lmm::test::replay {

// Verifies every config, pose, scan-index, and indexed scan file referenced by
// a validated replay case. Throws before runtime Open on missing, malformed, or
// content-mismatched inputs.
void VerifyReplayInputs(const nlohmann::json& manifest,
                        const std::filesystem::path& data_root,
                        const std::filesystem::path& config_root);

}  // namespace open_lmm::test::replay
