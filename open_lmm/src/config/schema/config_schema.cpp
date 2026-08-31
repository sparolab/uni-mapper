#include <open_lmm/utils/config_schema.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <regex>
#include <set>
#include <sstream>
#include <utility>

namespace open_lmm {
namespace {

struct SelectedField {
  FieldSpec spec;
  std::string plugin;
};

std::string EscapePointerToken(std::string_view token) {
  std::string escaped;
  escaped.reserve(token.size());
  for (const char character : token) {
    if (character == '~') {
      escaped += "~0";
    } else if (character == '/') {
      escaped += "~1";
    } else {
      escaped += character;
    }
  }
  return escaped;
}

Result<std::vector<std::string>> ParsePointer(std::string_view pointer) {
  if (pointer.empty()) {
    return Result<std::vector<std::string>>::Ok({});
  }
  if (pointer.front() != '/') {
    return Result<std::vector<std::string>>::Failure(
        Error::InvalidArgument("schema pointer must start with '/': " +
                               std::string(pointer)));
  }
  std::vector<std::string> tokens;
  std::size_t begin = 1;
  while (begin <= pointer.size()) {
    const std::size_t end = pointer.find('/', begin);
    const std::string_view encoded = pointer.substr(
        begin, end == std::string_view::npos ? pointer.size() - begin
                                             : end - begin);
    std::string token;
    token.reserve(encoded.size());
    for (std::size_t index = 0; index < encoded.size(); ++index) {
      if (encoded[index] != '~') {
        token += encoded[index];
        continue;
      }
      if (index + 1 >= encoded.size() ||
          (encoded[index + 1] != '0' && encoded[index + 1] != '1')) {
        return Result<std::vector<std::string>>::Failure(
            Error::InvalidArgument("invalid JSON pointer escape: " +
                                   std::string(pointer)));
      }
      token += encoded[index + 1] == '0' ? '~' : '/';
      ++index;
    }
    if (token.empty()) {
      return Result<std::vector<std::string>>::Failure(
          Error::InvalidArgument("schema pointer contains an empty token: " +
                                 std::string(pointer)));
    }
    tokens.push_back(std::move(token));
    if (end == std::string_view::npos) break;
    begin = end + 1;
  }
  return Result<std::vector<std::string>>::Ok(std::move(tokens));
}

const nlohmann::json* FindValue(const nlohmann::json& document,
                                const std::vector<std::string>& tokens) {
  const nlohmann::json* current = &document;
  for (const auto& token : tokens) {
    if (!current->is_object()) return nullptr;
    const auto found = current->find(token);
    if (found == current->end()) return nullptr;
    current = &*found;
  }
  return current;
}

nlohmann::json* FindValue(nlohmann::json& document,
                          const std::vector<std::string>& tokens) {
  return const_cast<nlohmann::json*>(FindValue(
      static_cast<const nlohmann::json&>(document), tokens));
}

void SetValue(nlohmann::json& document, const std::vector<std::string>& tokens,
              const nlohmann::json& value) {
  nlohmann::json* current = &document;
  for (std::size_t index = 0; index < tokens.size(); ++index) {
    if (index + 1 == tokens.size()) {
      (*current)[tokens[index]] = value;
      return;
    }
    auto& child = (*current)[tokens[index]];
    if (!child.is_object()) child = nlohmann::json::object();
    current = &child;
  }
  document = value;
}

void EraseValue(nlohmann::json& document,
                const std::vector<std::string>& tokens) {
  if (tokens.empty()) {
    document = nlohmann::json::object();
    return;
  }
  nlohmann::json* parent = &document;
  for (std::size_t index = 0; index + 1 < tokens.size(); ++index) {
    if (!parent->is_object()) return;
    auto found = parent->find(tokens[index]);
    if (found == parent->end()) return;
    parent = &*found;
  }
  if (parent->is_object()) parent->erase(tokens.back());
}

std::optional<std::size_t> DepthLimitViolation(const nlohmann::json& value,
                                               std::size_t maximum_depth) {
  // Configuration is untrusted input.  Do not mirror its nesting on the C++
  // stack: a document may be constructed programmatically and validation must
  // remain bounded even when it is far deeper than the configured limit.
  struct Pending {
    const nlohmann::json* value;
    std::size_t depth;
  };
  std::vector<Pending> pending{{&value, 1}};
  while (!pending.empty()) {
    const Pending current = pending.back();
    pending.pop_back();
    if (current.depth > maximum_depth) return current.depth;
    if (!current.value->is_array() && !current.value->is_object()) continue;
    for (const auto& child : *current.value) {
      pending.push_back({&child, current.depth + 1});
    }
  }
  return std::nullopt;
}

std::string ActualTypeName(const nlohmann::json& value) {
  if (value.is_null()) return "null";
  if (value.is_boolean()) return "boolean";
  if (value.is_number_unsigned()) return "unsigned integer";
  if (value.is_number_integer()) return "signed integer";
  if (value.is_number_float()) return "number";
  if (value.is_string()) return "string";
  if (value.is_object()) return "object";
  if (value.is_array()) return "array";
  return "unknown";
}

bool MatchesType(const nlohmann::json& value, SchemaValueType type) {
  switch (type) {
    case SchemaValueType::kBoolean:
      return value.is_boolean();
    case SchemaValueType::kSignedInteger:
      return value.is_number_unsigned()
                 ? value.get<uint64_t>() <=
                       static_cast<uint64_t>(std::numeric_limits<int64_t>::max())
                 : value.is_number_integer();
    case SchemaValueType::kUnsignedInteger:
      return value.is_number_unsigned() ||
             (value.is_number_integer() && value.get<int64_t>() >= 0);
    case SchemaValueType::kNumber:
      return value.is_number();
    case SchemaValueType::kString:
    case SchemaValueType::kPath:
      return value.is_string();
    case SchemaValueType::kObject:
      return value.is_object();
    case SchemaValueType::kArray:
      return value.is_array();
  }
  return false;
}

std::string PluginContext(const SchemaFragment& fragment) {
  if (!fragment.selector) return {};
  if (fragment.selector->plugin_kind.empty()) {
    return fragment.selector->plugin_name;
  }
  return fragment.selector->plugin_kind + ":" +
         fragment.selector->plugin_name;
}

Error SchemaError(Error::Code code, std::string message,
                  std::string_view source, std::string pointer,
                  std::string expected, std::string actual, uint32_t version,
                  std::string plugin = {}) {
  Error error{code, std::move(message)};
  error.WithConfig(std::string(source))
      .WithValidation(std::move(pointer), std::move(expected),
                      std::move(actual), version);
  if (!plugin.empty()) error.WithPlugin(std::move(plugin));
  return error;
}

Result<void> ValidateConstraint(const SelectedField& field,
                                const nlohmann::json& value,
                                std::string_view source, uint32_t version) {
  const auto& constraints = field.spec.constraints;
  const auto fail = [&](std::string expected, std::string actual,
                        std::string detail) {
    if (field.spec.secret) actual = "[REDACTED]";
    return Result<void>::Failure(SchemaError(
        Error::Code::kInvalidArgument,
        "schema validation failed at " + field.spec.pointer + ": " + detail,
        source, field.spec.pointer, std::move(expected), std::move(actual),
        version, field.plugin));
  };

  if (!constraints.allowed_values.empty() &&
      std::find(constraints.allowed_values.begin(),
                constraints.allowed_values.end(), value) ==
          constraints.allowed_values.end()) {
    return fail("one of the declared enum values", value.dump(),
                "value is outside the enum");
  }

  if (value.is_number()) {
    const double number = value.get<double>();
    if (constraints.finite && !std::isfinite(number)) {
      return fail("finite number", value.dump(), "number is not finite");
    }
    if (constraints.minimum &&
        (constraints.exclusive_minimum
             ? number <= *constraints.minimum
             : number < *constraints.minimum)) {
      return fail((constraints.exclusive_minimum ? "> " : ">= ") +
                      std::to_string(*constraints.minimum),
                  value.dump(), "number is below the minimum");
    }
    if (constraints.maximum &&
        (constraints.exclusive_maximum
             ? number >= *constraints.maximum
             : number > *constraints.maximum)) {
      return fail((constraints.exclusive_maximum ? "< " : "<= ") +
                      std::to_string(*constraints.maximum),
                  value.dump(), "number is above the maximum");
    }
  }

  if (value.is_string()) {
    const auto& text = value.get_ref<const std::string&>();
    if (constraints.minimum_length &&
        text.size() < *constraints.minimum_length) {
      return fail("string length >= " +
                      std::to_string(*constraints.minimum_length),
                  std::to_string(text.size()), "string is too short");
    }
    if (constraints.maximum_length &&
        text.size() > *constraints.maximum_length) {
      return fail("string length <= " +
                      std::to_string(*constraints.maximum_length),
                  std::to_string(text.size()), "string is too long");
    }
    if (!constraints.pattern.empty()) {
      try {
        if (!std::regex_match(text, std::regex(constraints.pattern))) {
          return fail("string matching " + constraints.pattern, text,
                      "string does not match the pattern");
        }
      } catch (const std::regex_error&) {
        return fail("valid schema regex", constraints.pattern,
                    "schema contains an invalid regex");
      }
    }
  }

  if (value.is_array()) {
    if (constraints.minimum_items &&
        value.size() < *constraints.minimum_items) {
      return fail("array size >= " +
                      std::to_string(*constraints.minimum_items),
                  std::to_string(value.size()), "array is too short");
    }
    if (constraints.maximum_items &&
        value.size() > *constraints.maximum_items) {
      return fail("array size <= " +
                      std::to_string(*constraints.maximum_items),
                  std::to_string(value.size()), "array is too long");
    }
    std::set<std::string> unique_items;
    for (std::size_t index = 0; index < value.size(); ++index) {
      const auto& item = value[index];
      if (constraints.item_type &&
          !MatchesType(item, *constraints.item_type)) {
        return fail(std::string(SchemaValueTypeName(*constraints.item_type)) +
                        " array items",
                    ActualTypeName(item),
                    "array item " + std::to_string(index) +
                        " has the wrong type");
      }
      if (constraints.finite && constraints.item_type &&
          (*constraints.item_type == SchemaValueType::kNumber ||
           *constraints.item_type == SchemaValueType::kSignedInteger ||
           *constraints.item_type == SchemaValueType::kUnsignedInteger) &&
          item.is_number() && !std::isfinite(item.get<double>())) {
        return fail("finite numeric array items", item.dump(),
                    "array item " + std::to_string(index) +
                        " is not finite");
      }
      if (constraints.items_non_empty && item.is_string() &&
          item.get_ref<const std::string&>().empty()) {
        return fail("non-empty array items", "empty string",
                    "array item " + std::to_string(index) + " is empty");
      }
      if (constraints.unique_items &&
          !unique_items.insert(item.dump()).second) {
        return fail("unique array items", item.dump(),
                    "array contains a duplicate item");
      }
    }
    if (constraints.array_slice_norm) {
      const auto& slice = *constraints.array_slice_norm;
      if (slice.count == 0 || slice.offset > value.size() ||
          slice.count > value.size() - slice.offset) {
        return fail("valid array norm slice", value.dump(),
                    "array norm slice is out of range");
      }
      double norm = 0.0;
      for (std::size_t index = slice.offset;
           index < slice.offset + slice.count; ++index) {
        if (!value[index].is_number()) {
          return fail("numeric array norm slice", value[index].dump(),
                      "array norm item is non-numeric");
        }
        const double item = value[index].get<double>();
        if (!std::isfinite(item)) {
          return fail("finite array norm slice", value[index].dump(),
                      "array norm item is not finite");
        }
        norm = std::hypot(norm, item);
      }
      const bool valid = slice.exclusive_minimum
                             ? norm > slice.minimum
                             : norm >= slice.minimum;
      if (!valid) {
        return fail(slice.exclusive_minimum
                        ? "array slice norm > " +
                              std::to_string(slice.minimum)
                        : "array slice norm >= " +
                              std::to_string(slice.minimum),
                    std::to_string(norm), "array slice norm is too small");
      }
    }
  }
  return Result<void>::Ok();
}

bool IsNumeric(const nlohmann::json* value) {
  return value != nullptr && value->is_number();
}

Result<void> ValidateRule(const CrossFieldRule& rule,
                          const nlohmann::json& document,
                          std::string_view source, uint32_t version,
                          std::string plugin) {
  std::vector<std::vector<std::string>> parsed;
  parsed.reserve(rule.pointers.size());
  for (const auto& pointer : rule.pointers) {
    auto tokens = ParsePointer(pointer);
    if (!tokens) return Result<void>::Failure(tokens.GetError());
    parsed.push_back(std::move(tokens).Value());
  }
  const auto value = [&](std::size_t index) -> const nlohmann::json* {
    return index < parsed.size() ? FindValue(document, parsed[index]) : nullptr;
  };
  const auto fail = [&](std::string pointer, std::string expected,
                        std::string actual) {
    return Result<void>::Failure(SchemaError(
        Error::Code::kInvalidArgument,
        rule.message.empty() ? "cross-field schema rule failed"
                             : rule.message,
        source, std::move(pointer), std::move(expected), std::move(actual),
        version, plugin));
  };

  switch (rule.kind) {
    case CrossFieldRuleKind::kLessThan:
    case CrossFieldRuleKind::kLessThanOrEqual: {
      if (rule.pointers.size() != 2 || !IsNumeric(value(0)) ||
          !IsNumeric(value(1))) {
        return fail(rule.pointers.empty() ? "" : rule.pointers.front(),
                    "two numeric fields", "missing or non-numeric field");
      }
      const double left = value(0)->get<double>();
      const double right = value(1)->get<double>();
      const bool valid = rule.kind == CrossFieldRuleKind::kLessThan
                             ? left < right
                             : left <= right;
      if (!valid) {
        return fail(rule.pointers[0],
                    rule.kind == CrossFieldRuleKind::kLessThan
                        ? "value < " + rule.pointers[1]
                        : "value <= " + rule.pointers[1],
                    value(0)->dump() + " vs " + value(1)->dump());
      }
      break;
    }
    case CrossFieldRuleKind::kMutuallyExclusive: {
      std::size_t present = 0;
      for (std::size_t index = 0; index < parsed.size(); ++index) {
        if (FindValue(document, parsed[index])) ++present;
      }
      if (present > 1) {
        return fail(rule.pointers.empty() ? "" : rule.pointers.front(),
                    "at most one mutually exclusive field",
                    std::to_string(present) + " fields present");
      }
      break;
    }
    case CrossFieldRuleKind::kRequiredWhenEquals: {
      if (rule.pointers.size() != 2 || !rule.expected_value) {
        return fail(rule.pointers.empty() ? "" : rule.pointers.front(),
                    "valid conditional-required rule", "invalid schema rule");
      }
      const auto* condition = value(0);
      if (condition && *condition == *rule.expected_value && !value(1)) {
        return fail(rule.pointers[1], "required when " + rule.pointers[0] +
                                              " equals " +
                                              rule.expected_value->dump(),
                    "missing");
      }
      break;
    }
    case CrossFieldRuleKind::kEqualsConstant: {
      if (rule.pointers.size() != 1 || !rule.expected_value || !value(0) ||
          *value(0) != *rule.expected_value) {
        return fail(rule.pointers.empty() ? "" : rule.pointers.front(),
                    rule.expected_value ? rule.expected_value->dump()
                                        : "declared constant",
                    value(0) ? value(0)->dump() : "missing");
      }
      break;
    }
  }
  return Result<void>::Ok();
}

bool CouldCoexist(const SchemaFragment& left, const SchemaFragment& right) {
  if (left.document_kind != right.document_kind) return false;
  if (!left.selector || !right.selector) return true;
  if (left.selector->pointer != right.selector->pointer) return true;
  return left.selector->expected_value == right.selector->expected_value;
}

bool IsUnder(std::string_view pointer, std::string_view parent) {
  return pointer == parent ||
         (pointer.size() > parent.size() &&
          pointer.substr(0, parent.size()) == parent &&
          pointer[parent.size()] == '/');
}

Result<void> ValidateUnknownKeys(
    const nlohmann::json& value, const std::string& pointer,
    const std::set<std::string>& exact, const std::set<std::string>& prefixes,
    const std::vector<std::string>& open_subtrees, std::string_view source,
    uint32_t version, const std::string& plugin) {
  for (const auto& subtree : open_subtrees) {
    if (IsUnder(pointer, subtree)) return Result<void>::Ok();
  }
  if (value.is_object()) {
    for (auto iterator = value.begin(); iterator != value.end(); ++iterator) {
      const std::string child = pointer + "/" + EscapePointerToken(iterator.key());
      if (!exact.contains(child) && !prefixes.contains(child)) {
        return Result<void>::Failure(SchemaError(
            Error::Code::kInvalidArgument,
            "unknown config key at " + child, source, child,
            "declared schema field or explicit extensions object",
            "unknown key", version, plugin));
      }
      auto nested = ValidateUnknownKeys(iterator.value(), child, exact,
                                        prefixes, open_subtrees, source,
                                        version, plugin);
      if (!nested) return nested;
    }
  }
  return Result<void>::Ok();
}

}  // namespace

std::string ValidatedConfigDocument::CanonicalJson(int indent) const {
  return document_.dump(indent);
}

Result<SchemaRegistry> SchemaRegistry::Create(
    std::vector<SchemaFragment> fragments, SchemaLimits limits) {
  if (fragments.empty()) {
    return Result<SchemaRegistry>::Failure(
        Error::InvalidArgument("schema registry requires fragments"));
  }
  std::size_t total_fields = 0;
  std::set<std::string> fragment_ids;
  for (const auto& fragment : fragments) {
    if (fragment.id.empty() || fragment.version == 0) {
      return Result<SchemaRegistry>::Failure(
          Error::InvalidArgument("schema fragment id/version is invalid"));
    }
    if (!fragment_ids.insert(fragment.id).second) {
      return Result<SchemaRegistry>::Failure(
          Error::InvalidArgument("duplicate schema fragment id: " +
                                 fragment.id));
    }
    if (fragment.fields.size() > limits.maximum_fragment_fields) {
      return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
          "schema fragment field limit exceeded: " + fragment.id));
    }
    std::size_t fragment_bytes = fragment.id.size();
    total_fields += fragment.fields.size();
    std::set<std::string> local_fields;
    for (const auto& field : fragment.fields) {
      fragment_bytes += field.pointer.size() + field.ui.label.size() +
                        field.ui.help.size() + field.ui.group.size() +
                        field.deprecation.replacement_pointer.size();
      if (field.default_value) fragment_bytes += field.default_value->dump().size();
      std::set<std::string> migrated_values;
      for (const auto& migration : field.value_migrations) {
        fragment_bytes += migration.deprecated_value.dump().size() +
                          migration.replacement_value.dump().size();
        if (!MatchesType(migration.deprecated_value, field.type) ||
            !MatchesType(migration.replacement_value, field.type)) {
          return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
              "schema value migration type mismatch: " + fragment.id + ":" +
              field.pointer));
        }
        if (!migrated_values.insert(migration.deprecated_value.dump()).second) {
          return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
              "duplicate deprecated value migration: " + fragment.id + ":" +
              field.pointer));
        }
      }
      auto pointer = ParsePointer(field.pointer);
      if (!pointer) return Result<SchemaRegistry>::Failure(pointer.GetError());
      if (pointer.Value().size() > limits.maximum_fragment_depth) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema fragment depth limit exceeded: " + fragment.id + ":" +
            field.pointer));
      }
      if (!local_fields.insert(field.pointer).second) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "duplicate field in schema fragment: " + fragment.id + ":" +
            field.pointer));
      }
      if (field.deprecation.deprecated &&
          field.deprecation.replacement_pointer.empty()) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "deprecated field requires a replacement: " + field.pointer));
      }
      if (field.deprecation.deprecated) {
        auto replacement =
            ParsePointer(field.deprecation.replacement_pointer);
        if (!replacement)
          return Result<SchemaRegistry>::Failure(replacement.GetError());
      }
      if (field.constraints.minimum && field.constraints.maximum &&
          *field.constraints.minimum > *field.constraints.maximum) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema numeric range is inverted: " + fragment.id + ":" +
            field.pointer));
      }
      if (field.constraints.minimum_length &&
          field.constraints.maximum_length &&
          *field.constraints.minimum_length >
              *field.constraints.maximum_length) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema string length range is inverted: " + fragment.id + ":" +
            field.pointer));
      }
      if (field.constraints.minimum_items &&
          field.constraints.maximum_items &&
          *field.constraints.minimum_items >
              *field.constraints.maximum_items) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema array size range is inverted: " + fragment.id + ":" +
            field.pointer));
      }
      if (field.constraints.array_slice_norm &&
          (field.type != SchemaValueType::kArray ||
           field.constraints.array_slice_norm->count == 0 ||
           field.constraints.array_slice_norm->minimum < 0.0 ||
           !std::isfinite(field.constraints.array_slice_norm->minimum))) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema array norm constraint is invalid: " + fragment.id + ":" +
            field.pointer));
      }
      if (field.default_value) {
        if (!MatchesType(*field.default_value, field.type)) {
          return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
              "schema default has the wrong type: " + fragment.id + ":" +
              field.pointer));
        }
        auto valid_default = ValidateConstraint(
            {field, PluginContext(fragment)}, *field.default_value,
            fragment.id, fragment.version);
        if (!valid_default)
          return Result<SchemaRegistry>::Failure(valid_default.GetError());
      }
    }
    if (fragment.selector) {
      fragment_bytes += fragment.selector->pointer.size() +
                        fragment.selector->expected_value.dump().size() +
                        fragment.selector->plugin_kind.size() +
                        fragment.selector->plugin_name.size();
      auto selector_pointer = ParsePointer(fragment.selector->pointer);
      if (!selector_pointer)
        return Result<SchemaRegistry>::Failure(selector_pointer.GetError());
      if (selector_pointer.Value().size() > limits.maximum_fragment_depth) {
        return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
            "schema selector depth limit exceeded: " + fragment.id));
      }
    }
    for (const auto& rule : fragment.rules) {
      fragment_bytes += rule.message.size();
      for (const auto& pointer : rule.pointers) {
        fragment_bytes += pointer.size();
        auto parsed = ParsePointer(pointer);
        if (!parsed) return Result<SchemaRegistry>::Failure(parsed.GetError());
        if (parsed.Value().size() > limits.maximum_fragment_depth) {
          return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
              "schema rule depth limit exceeded: " + fragment.id));
        }
      }
      if (rule.expected_value) fragment_bytes += rule.expected_value->dump().size();
    }
    if (fragment_bytes > limits.maximum_fragment_bytes) {
      return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
          "schema fragment size limit exceeded: " + fragment.id));
    }
  }
  if (total_fields > limits.maximum_registry_fields) {
    return Result<SchemaRegistry>::Failure(
        Error::InvalidArgument("schema registry field limit exceeded"));
  }

  for (std::size_t left = 0; left < fragments.size(); ++left) {
    for (std::size_t right = left + 1; right < fragments.size(); ++right) {
      if (!CouldCoexist(fragments[left], fragments[right])) continue;
      std::set<std::string> left_fields;
      for (const auto& field : fragments[left].fields)
        left_fields.insert(field.pointer);
      for (const auto& field : fragments[right].fields) {
        if (left_fields.contains(field.pointer)) {
          return Result<SchemaRegistry>::Failure(Error::InvalidArgument(
              "schema field collision between " + fragments[left].id +
              " and " + fragments[right].id + ": " + field.pointer));
        }
      }
    }
  }
  return Result<SchemaRegistry>::Ok(
      SchemaRegistry(std::move(fragments), limits));
}

Result<ValidatedConfigDocument> SchemaRegistry::ParseAndValidate(
    ConfigDocumentKind kind, std::string_view json_text,
    std::string source_name, const ValidationOptions& options) const {
  const std::size_t maximum_bytes = std::min(
      limits_.maximum_document_bytes, options.limits.maximum_document_bytes);
  if (json_text.size() > maximum_bytes) {
    return Result<ValidatedConfigDocument>::Failure(SchemaError(
        Error::Code::kInvalidArgument, "config document size limit exceeded",
        source_name, "", "document bytes <= " +
                             std::to_string(maximum_bytes),
        std::to_string(json_text.size()), 1));
  }
  try {
    auto document = nlohmann::json::parse(json_text.begin(), json_text.end(),
                                          nullptr, true, true);
    return Validate(kind, document, std::move(source_name), options);
  } catch (const nlohmann::json::exception& error) {
    return Result<ValidatedConfigDocument>::Failure(
        SchemaError(Error::Code::kParseError,
                    "failed to parse config document: " +
                        std::string(error.what()),
                    source_name, "", "valid JSON", "parse error", 1));
  }
}

Result<ValidatedConfigDocument> SchemaRegistry::Validate(
    ConfigDocumentKind kind, const nlohmann::json& source_document,
    std::string source_name, const ValidationOptions& options) const {
  if (!source_document.is_object()) {
    return Result<ValidatedConfigDocument>::Failure(SchemaError(
        Error::Code::kInvalidArgument, "config document must be an object",
        source_name, "", "object", ActualTypeName(source_document), 1));
  }
  const std::size_t maximum_depth = std::min(
      limits_.maximum_document_depth, options.limits.maximum_document_depth);
  const auto depth_violation =
      DepthLimitViolation(source_document, maximum_depth);
  if (depth_violation) {
    return Result<ValidatedConfigDocument>::Failure(SchemaError(
        Error::Code::kInvalidArgument, "config document depth limit exceeded",
        source_name, "", "depth <= " + std::to_string(maximum_depth),
        std::to_string(*depth_violation), 1));
  }
  if (source_document.dump().size() >
      std::min(limits_.maximum_document_bytes,
               options.limits.maximum_document_bytes)) {
    return Result<ValidatedConfigDocument>::Failure(SchemaError(
        Error::Code::kInvalidArgument, "config document size limit exceeded",
        source_name, "", "bounded canonical document", "too large", 1));
  }

  std::vector<const SchemaFragment*> selected;
  for (const auto& fragment : fragments_) {
    if (fragment.document_kind != kind) continue;
    if (!fragment.selector) {
      selected.push_back(&fragment);
      continue;
    }
    auto selector_tokens = ParsePointer(fragment.selector->pointer);
    if (!selector_tokens) {
      return Result<ValidatedConfigDocument>::Failure(
          selector_tokens.GetError());
    }
    const auto* selected_value = FindValue(source_document,
                                           selector_tokens.Value());
    if (selected_value &&
        *selected_value == fragment.selector->expected_value) {
      selected.push_back(&fragment);
    }
  }
  if (selected.empty() ||
      std::none_of(selected.begin(), selected.end(),
                   [](const SchemaFragment* fragment) {
                     return !fragment->selector.has_value();
                   })) {
    return Result<ValidatedConfigDocument>::Failure(SchemaError(
        Error::Code::kInvalidArgument,
        "no base schema registered for config document",
        source_name, "", "base schema for " +
                             std::string(ConfigDocumentKindName(kind)),
        "missing", 1));
  }

  uint32_t version = selected.front()->version;
  std::vector<SelectedField> fields;
  std::vector<std::pair<CrossFieldRule, std::string>> rules;
  std::set<std::string> merged_pointers;
  std::set<std::string> selected_plugins;
  for (const auto* fragment : selected) {
    if (fragment->version != version) {
      return Result<ValidatedConfigDocument>::Failure(SchemaError(
          Error::Code::kInvalidArgument,
          "selected schema fragments have incompatible versions",
          source_name, "", "schema version " + std::to_string(version),
          std::to_string(fragment->version), version,
          PluginContext(*fragment)));
    }
    if (!PluginContext(*fragment).empty())
      selected_plugins.insert(PluginContext(*fragment));
    for (const auto& field : fragment->fields) {
      if (!merged_pointers.insert(field.pointer).second) {
        return Result<ValidatedConfigDocument>::Failure(SchemaError(
            Error::Code::kInvalidArgument,
            "selected schema fragments collide at " + field.pointer,
            source_name, field.pointer, "one authoritative field",
            "duplicate field", version, PluginContext(*fragment)));
      }
      fields.push_back({field, PluginContext(*fragment)});
    }
    for (const auto& rule : fragment->rules)
      rules.emplace_back(rule, PluginContext(*fragment));
  }

  nlohmann::json document = source_document;
  std::vector<ValidationWarning> warnings;
  for (const auto& field : fields) {
    if (!field.spec.deprecation.deprecated) continue;
    auto old_tokens = ParsePointer(field.spec.pointer);
    auto replacement_tokens =
        ParsePointer(field.spec.deprecation.replacement_pointer);
    if (!old_tokens || !replacement_tokens) {
      return Result<ValidatedConfigDocument>::Failure(
          !old_tokens ? old_tokens.GetError() : replacement_tokens.GetError());
    }
    const auto* old_value = FindValue(document, old_tokens.Value());
    if (!old_value) continue;
    if (!options.migrate_deprecated_keys) {
      return Result<ValidatedConfigDocument>::Failure(SchemaError(
          Error::Code::kInvalidArgument,
          "deprecated config key requires migration: " + field.spec.pointer,
          source_name, field.spec.pointer,
          "replacement " + field.spec.deprecation.replacement_pointer,
          "deprecated key", version, field.plugin));
    }
    if (FindValue(document, replacement_tokens.Value())) {
      return Result<ValidatedConfigDocument>::Failure(SchemaError(
          Error::Code::kInvalidArgument,
          "deprecated and replacement config keys are both present",
          source_name, field.spec.pointer,
          "only " + field.spec.deprecation.replacement_pointer,
          "both keys present", version, field.plugin));
    }
    const nlohmann::json migrated = *old_value;
    SetValue(document, replacement_tokens.Value(), migrated);
    EraseValue(document, old_tokens.Value());
    warnings.push_back({field.spec.pointer,
                        "migrated to " +
                            field.spec.deprecation.replacement_pointer});
  }

  if (options.apply_defaults) {
    for (const auto& field : fields) {
      if (!field.spec.default_value || field.spec.deprecation.deprecated)
        continue;
      auto tokens = ParsePointer(field.spec.pointer);
      if (!tokens)
        return Result<ValidatedConfigDocument>::Failure(tokens.GetError());
      if (!FindValue(document, tokens.Value()))
        SetValue(document, tokens.Value(), *field.spec.default_value);
    }
  }

  for (const auto& field : fields) {
    if (field.spec.value_migrations.empty()) continue;
    auto tokens = ParsePointer(field.spec.pointer);
    if (!tokens)
      return Result<ValidatedConfigDocument>::Failure(tokens.GetError());
    auto* value = FindValue(document, tokens.Value());
    if (!value) continue;
    for (const auto& migration : field.spec.value_migrations) {
      if (*value != migration.deprecated_value) continue;
      if (!options.migrate_deprecated_keys) {
        return Result<ValidatedConfigDocument>::Failure(SchemaError(
            Error::Code::kInvalidArgument,
            "deprecated config value requires migration: " +
                field.spec.pointer,
            source_name, field.spec.pointer,
            migration.replacement_value.dump(), value->dump(), version,
            field.plugin));
      }
      const std::string old_value = value->dump();
      *value = migration.replacement_value;
      warnings.push_back(
          {field.spec.pointer,
           "migrated deprecated value " + old_value + " to " + value->dump()});
      break;
    }
  }

  std::set<std::string> exact;
  std::set<std::string> prefixes;
  std::vector<std::string> open_subtrees;
  for (const auto& field : fields) {
    if (field.spec.deprecation.deprecated) continue;
    exact.insert(field.spec.pointer);
    auto tokens = ParsePointer(field.spec.pointer);
    if (!tokens)
      return Result<ValidatedConfigDocument>::Failure(tokens.GetError());
    std::string prefix;
    for (const auto& token : tokens.Value()) {
      prefix += "/" + EscapePointerToken(token);
      prefixes.insert(prefix);
    }
    if (field.spec.allow_unknown_children)
      open_subtrees.push_back(field.spec.pointer);

    const auto* value = FindValue(document, tokens.Value());
    if (!value) {
      if (field.spec.required) {
        return Result<ValidatedConfigDocument>::Failure(SchemaError(
            Error::Code::kInvalidArgument,
            "required config field is missing: " + field.spec.pointer,
            source_name, field.spec.pointer,
            std::string(SchemaValueTypeName(field.spec.type)), "missing",
            version, field.plugin));
      }
      continue;
    }
    if (!MatchesType(*value, field.spec.type)) {
      return Result<ValidatedConfigDocument>::Failure(SchemaError(
          Error::Code::kInvalidArgument,
          "config field has the wrong type: " + field.spec.pointer,
          source_name, field.spec.pointer,
          std::string(SchemaValueTypeName(field.spec.type)),
          field.spec.secret ? "[REDACTED]" : ActualTypeName(*value), version,
          field.plugin));
    }
    auto constraints = ValidateConstraint(field, *value, source_name, version);
    if (!constraints) return Result<ValidatedConfigDocument>::Failure(
        constraints.GetError());
  }

  if (options.reject_unknown_keys) {
    auto unknown = ValidateUnknownKeys(document, "", exact, prefixes,
                                       open_subtrees, source_name, version,
                                       selected_plugins.size() == 1
                                           ? *selected_plugins.begin()
                                           : std::string());
    if (!unknown) return Result<ValidatedConfigDocument>::Failure(
        unknown.GetError());
  }

  for (const auto& [rule, plugin] : rules) {
    auto valid = ValidateRule(rule, document, source_name, version, plugin);
    if (!valid)
      return Result<ValidatedConfigDocument>::Failure(valid.GetError());
  }
  return Result<ValidatedConfigDocument>::Ok(ValidatedConfigDocument(
      kind, version, std::move(document), std::move(warnings)));
}

std::vector<SchemaFragment> SchemaRegistry::Fragments(
    ConfigDocumentKind kind) const {
  std::vector<SchemaFragment> result;
  for (const auto& fragment : fragments_) {
    if (fragment.document_kind == kind) result.push_back(fragment);
  }
  return result;
}

std::string_view ConfigDocumentKindName(ConfigDocumentKind kind) noexcept {
  switch (kind) {
    case ConfigDocumentKind::kRoot:
      return "root";
    case ConfigDocumentKind::kMapServer:
      return "map_server";
    case ConfigDocumentKind::kDataLoader:
      return "data_loader";
    case ConfigDocumentKind::kLoopDetector:
      return "loop_detector";
    case ConfigDocumentKind::kBackendOptimizer:
      return "backend_optimizer";
    case ConfigDocumentKind::kDynamicRemover:
      return "dynamic_remover";
  }
  return "unknown";
}

std::string_view SchemaValueTypeName(SchemaValueType type) noexcept {
  switch (type) {
    case SchemaValueType::kBoolean:
      return "boolean";
    case SchemaValueType::kSignedInteger:
      return "signed integer";
    case SchemaValueType::kUnsignedInteger:
      return "unsigned integer";
    case SchemaValueType::kNumber:
      return "number";
    case SchemaValueType::kString:
      return "string";
    case SchemaValueType::kPath:
      return "path string";
    case SchemaValueType::kObject:
      return "object";
    case SchemaValueType::kArray:
      return "array";
  }
  return "unknown";
}

}  // namespace open_lmm
