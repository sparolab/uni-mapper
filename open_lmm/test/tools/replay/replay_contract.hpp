#pragma once

#include <filesystem>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace open_lmm::test::replay {

struct ValidationIssue {
  std::string pointer;
  std::string message;
};

struct ValidationResult {
  std::vector<ValidationIssue> issues;

  [[nodiscard]] bool Ok() const noexcept { return issues.empty(); }
  [[nodiscard]] std::string Summary() const;
};

struct ComparisonDifference {
  std::string pointer;
  std::string rule;
  nlohmann::json expected;
  nlohmann::json actual;
  nlohmann::json limit;
  std::string message;
};

struct ComparisonResult {
  std::vector<ComparisonDifference> differences;

  [[nodiscard]] bool Passed() const noexcept { return differences.empty(); }
  [[nodiscard]] nlohmann::json ToJson() const;
};

[[nodiscard]] nlohmann::json LoadJsonFile(const std::filesystem::path& path);
[[nodiscard]] std::string CanonicalJson(const nlohmann::json& value);

[[nodiscard]] ValidationResult ValidateCaseManifest(
    const nlohmann::json& document);
[[nodiscard]] ValidationResult ValidateReplayReport(
    const nlohmann::json& document);
[[nodiscard]] ValidationResult ValidateReplayBaseline(
    const nlohmann::json& document);

[[nodiscard]] ComparisonResult CompareReplayReport(
    const nlohmann::json& baseline, const nlohmann::json& report);

}  // namespace open_lmm::test::replay
