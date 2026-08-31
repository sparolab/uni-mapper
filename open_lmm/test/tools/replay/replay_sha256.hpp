#pragma once

#include <filesystem>
#include <string>
#include <string_view>

namespace open_lmm::test::replay {

[[nodiscard]] std::string Sha256(std::string_view bytes);
[[nodiscard]] std::string Sha256File(const std::filesystem::path& path);

}  // namespace open_lmm::test::replay
