#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json_fwd.hpp>

namespace open_lmm::test::benchmark {

struct ValidationIssue {
  std::string pointer;
  std::string message;
};

struct ValidationResult {
  std::vector<ValidationIssue> issues;
  [[nodiscard]] bool Ok() const noexcept { return issues.empty(); }
  [[nodiscard]] std::string Summary() const;
};

ValidationResult ValidatePerformanceReport(const nlohmann::json& report);
nlohmann::json MachineMetadata();
void WriteJsonExclusive(const std::filesystem::path& path,
                        const nlohmann::json& value);

}  // namespace open_lmm::test::benchmark
