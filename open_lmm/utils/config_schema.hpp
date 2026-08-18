#pragma once

#include <open_lmm/common/result.hpp>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include <nlohmann/json.hpp>

namespace open_lmm {

enum class ConfigDocumentKind : uint8_t {
  kRoot,
  kMapServer,
  kDataLoader,
  kLoopDetector,
  kBackendOptimizer,
  kDynamicRemover,
};

enum class SchemaValueType : uint8_t {
  kBoolean,
  kSignedInteger,
  kUnsignedInteger,
  kNumber,
  kString,
  kPath,
  kObject,
  kArray,
};

enum class CrossFieldRuleKind : uint8_t {
  kLessThan,
  kLessThanOrEqual,
  kMutuallyExclusive,
  kRequiredWhenEquals,
  kEqualsConstant,
};

struct UiHint {
  std::string label;
  std::string help;
  std::string group;
  int order = 0;
  bool read_only = false;
  bool advanced = false;
};

struct DeprecationInfo {
  bool deprecated = false;
  std::string replacement_pointer;
  uint32_t since_version = 0;
};

struct ValueMigration {
  nlohmann::json deprecated_value;
  nlohmann::json replacement_value;
  uint32_t since_version = 0;
};

struct ConstraintSet {
  std::optional<double> minimum;
  std::optional<double> maximum;
  bool exclusive_minimum = false;
  bool exclusive_maximum = false;
  bool finite = true;

  std::optional<std::size_t> minimum_length;
  std::optional<std::size_t> maximum_length;
  std::string pattern;
  std::vector<nlohmann::json> allowed_values;

  std::optional<std::size_t> minimum_items;
  std::optional<std::size_t> maximum_items;
  std::optional<SchemaValueType> item_type;
  bool items_non_empty = false;
  bool unique_items = false;

  struct ArraySliceNorm {
    std::size_t offset = 0;
    std::size_t count = 0;
    double minimum = 0.0;
    bool exclusive_minimum = true;
  };
  std::optional<ArraySliceNorm> array_slice_norm;
};

struct FieldSpec {
  std::string pointer;
  SchemaValueType type = SchemaValueType::kString;
  bool required = false;
  std::optional<nlohmann::json> default_value;
  ConstraintSet constraints;
  UiHint ui;
  DeprecationInfo deprecation;
  std::vector<ValueMigration> value_migrations;
  bool secret = false;
  bool allow_unknown_children = false;
};

struct CrossFieldRule {
  CrossFieldRuleKind kind = CrossFieldRuleKind::kLessThan;
  std::vector<std::string> pointers;
  std::optional<nlohmann::json> expected_value;
  std::string message;
};

struct FragmentSelector {
  std::string pointer;
  nlohmann::json expected_value;
  std::string plugin_kind;
  std::string plugin_name;
};

struct SchemaFragment {
  std::string id;
  ConfigDocumentKind document_kind = ConfigDocumentKind::kRoot;
  uint32_t version = 1;
  std::optional<FragmentSelector> selector;
  std::vector<FieldSpec> fields;
  std::vector<CrossFieldRule> rules;
};

struct SchemaLimits {
  std::size_t maximum_document_bytes = 1024U * 1024U;
  std::size_t maximum_document_depth = 16;
  std::size_t maximum_fragment_fields = 128;
  std::size_t maximum_fragment_depth = 8;
  std::size_t maximum_fragment_bytes = 64U * 1024U;
  std::size_t maximum_registry_fields = 1024;
};

struct ValidationOptions {
  bool reject_unknown_keys = true;
  bool apply_defaults = true;
  bool migrate_deprecated_keys = true;
  SchemaLimits limits;
};

struct ValidationWarning {
  std::string pointer;
  std::string message;
};

class ValidatedConfigDocument {
 public:
  [[nodiscard]] ConfigDocumentKind Kind() const noexcept { return kind_; }
  [[nodiscard]] uint32_t Version() const noexcept { return version_; }
  [[nodiscard]] const nlohmann::json& Document() const noexcept {
    return document_;
  }
  [[nodiscard]] const std::vector<ValidationWarning>& Warnings() const noexcept {
    return warnings_;
  }
  [[nodiscard]] std::string CanonicalJson(int indent = -1) const;

 private:
  friend class SchemaRegistry;
  ValidatedConfigDocument(ConfigDocumentKind kind, uint32_t version,
                          nlohmann::json document,
                          std::vector<ValidationWarning> warnings)
      : kind_(kind),
        version_(version),
        document_(std::move(document)),
        warnings_(std::move(warnings)) {}

  ConfigDocumentKind kind_;
  uint32_t version_;
  nlohmann::json document_;
  std::vector<ValidationWarning> warnings_;
};

class SchemaRegistry {
 public:
  static Result<SchemaRegistry> Create(
      std::vector<SchemaFragment> fragments,
      SchemaLimits limits = {});

  [[nodiscard]] Result<ValidatedConfigDocument> ParseAndValidate(
      ConfigDocumentKind kind, std::string_view json_text,
      std::string source_name = "<memory>",
      const ValidationOptions& options = {}) const;

  [[nodiscard]] Result<ValidatedConfigDocument> Validate(
      ConfigDocumentKind kind, const nlohmann::json& document,
      std::string source_name = "<memory>",
      const ValidationOptions& options = {}) const;

  [[nodiscard]] std::vector<SchemaFragment> Fragments(
      ConfigDocumentKind kind) const;

 private:
  SchemaRegistry(std::vector<SchemaFragment> fragments, SchemaLimits limits)
      : fragments_(std::move(fragments)), limits_(limits) {}

  std::vector<SchemaFragment> fragments_;
  SchemaLimits limits_;
};

[[nodiscard]] const SchemaRegistry& BuiltinConfigSchemaRegistry();
[[nodiscard]] Result<void> ValidateRuntimeConfigDocuments(
    const ValidatedConfigDocument& root,
    const ValidatedConfigDocument& map_server);
[[nodiscard]] std::string_view ConfigDocumentKindName(
    ConfigDocumentKind kind) noexcept;
[[nodiscard]] std::string_view SchemaValueTypeName(
    SchemaValueType type) noexcept;

}  // namespace open_lmm
