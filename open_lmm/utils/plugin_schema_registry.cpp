#include "plugin_schema_registry.hpp"

#include <algorithm>
#include <array>
#include <limits>
#include <string>

namespace open_lmm {
namespace {

Result<ConfigDocumentKind> DocumentKind(std::string_view value) {
  if (value == "root") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kRoot);
  if (value == "map_server") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kMapServer);
  if (value == "data_loader") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kDataLoader);
  if (value == "loop_detector") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kLoopDetector);
  if (value == "backend_optimizer") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kBackendOptimizer);
  if (value == "dynamic_remover") return Result<ConfigDocumentKind>::Ok(ConfigDocumentKind::kDynamicRemover);
  return Result<ConfigDocumentKind>::Failure(
      Error::InvalidArgument("unknown plugin schema document_kind"));
}

Result<SchemaValueType> ValueType(std::string_view value) {
  if (value == "boolean") return Result<SchemaValueType>::Ok(SchemaValueType::kBoolean);
  if (value == "integer") return Result<SchemaValueType>::Ok(SchemaValueType::kSignedInteger);
  if (value == "unsigned") return Result<SchemaValueType>::Ok(SchemaValueType::kUnsignedInteger);
  if (value == "number") return Result<SchemaValueType>::Ok(SchemaValueType::kNumber);
  if (value == "string") return Result<SchemaValueType>::Ok(SchemaValueType::kString);
  if (value == "path") return Result<SchemaValueType>::Ok(SchemaValueType::kPath);
  if (value == "object") return Result<SchemaValueType>::Ok(SchemaValueType::kObject);
  if (value == "array") return Result<SchemaValueType>::Ok(SchemaValueType::kArray);
  return Result<SchemaValueType>::Failure(
      Error::InvalidArgument("unknown plugin schema field type"));
}

bool ExceedsJsonDepth(const nlohmann::json& value, std::size_t maximum) {
  std::vector<std::pair<const nlohmann::json*, std::size_t>> pending;
  pending.push_back({&value, 0});
  while (!pending.empty()) {
    const auto [current, depth] = pending.back();
    pending.pop_back();
    if (depth > maximum) return true;
    if (!current->is_array() && !current->is_object()) continue;
    for (const auto& child : *current) pending.push_back({&child, depth + 1});
  }
  return false;
}

bool HasOnly(const nlohmann::json& object,
             std::initializer_list<std::string_view> allowed) {
  if (!object.is_object()) return false;
  for (auto iterator = object.begin(); iterator != object.end(); ++iterator) {
    if (std::none_of(allowed.begin(), allowed.end(), [&](auto key) {
          return iterator.key() == key;
        })) return false;
  }
  return true;
}

template <typename T>
void OptionalNumber(const nlohmann::json& source, const char* key,
                    std::optional<T>* destination) {
  if (source.contains(key)) *destination = source.at(key).get<T>();
}

Result<SchemaFragment> Decode(const PluginV2Metadata& metadata,
                              SchemaLimits limits) {
  if ((metadata.capability_bits & OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) == 0)
    return Result<SchemaFragment>::Failure(
        Error::InvalidArgument("plugin does not advertise schema fragment"));
  if (metadata.schema_fragment_json.empty() ||
      metadata.schema_fragment_json.size() > limits.maximum_fragment_bytes)
    return Result<SchemaFragment>::Failure(
        Error::InvalidArgument("plugin schema fragment byte limit exceeded"));
  try {
    bool depth_exceeded = false;
    const auto maximum_parse_depth = limits.maximum_fragment_depth + 2;
    const auto callback = [&](int depth, nlohmann::json::parse_event_t event,
                              nlohmann::json&) {
      if ((event == nlohmann::json::parse_event_t::object_start ||
           event == nlohmann::json::parse_event_t::array_start) &&
          static_cast<std::size_t>(depth) > maximum_parse_depth) {
        depth_exceeded = true;
        return false;
      }
      return !depth_exceeded;
    };
    const auto json = nlohmann::json::parse(
        metadata.schema_fragment_json, callback, true, false);
    if (depth_exceeded) {
      return Result<SchemaFragment>::Failure(Error::InvalidArgument(
          "plugin schema fragment depth limit exceeded during parse"));
    }
    if (!json.is_object() ||
        ExceedsJsonDepth(json, limits.maximum_fragment_depth + 2) ||
        !HasOnly(json, {"id", "version", "document_kind", "selector",
                        "fields"}) || !json.contains("fields"))
      return Result<SchemaFragment>::Failure(
          Error::InvalidArgument("plugin schema fragment shape/depth is invalid"));
    SchemaFragment fragment;
    fragment.id = json.at("id").get<std::string>();
    fragment.version = json.at("version").get<uint32_t>();
    auto kind = DocumentKind(json.at("document_kind").get<std::string>());
    if (!kind) return Result<SchemaFragment>::Failure(kind.GetError());
    fragment.document_kind = kind.Value();
    if (fragment.id != metadata.schema_id ||
        fragment.version != metadata.schema_version)
      return Result<SchemaFragment>::Failure(Error::InvalidArgument(
          "plugin schema identity does not match query metadata"));
    ConfigDocumentKind owned_document;
    std::string_view owned_selector;
    std::string_view owned_root;
    if (metadata.kind == "descriptor") {
      owned_document = ConfigDocumentKind::kLoopDetector;
      owned_selector = "/loop_detector/model";
      owned_root = "/loop_detector/";
    } else if (metadata.kind == "dynamic_remover") {
      owned_document = ConfigDocumentKind::kDynamicRemover;
      owned_selector = "/dynamic_remover/model";
      owned_root = "/dynamic_remover/";
    } else {
      return Result<SchemaFragment>::Failure(
          Error::InvalidArgument("plugin schema kind has no config domain"));
    }
    if (fragment.document_kind != owned_document ||
        metadata.selected_model.empty() || !json.contains("selector")) {
      return Result<SchemaFragment>::Failure(Error::InvalidArgument(
          "plugin schema is not bound to its selected config domain"));
    }
    const auto& source = json.at("selector");
    if (!HasOnly(source, {"pointer", "equals"}) ||
        source.at("pointer").get<std::string>() != owned_selector ||
        !source.at("equals").is_string() ||
        source.at("equals").get<std::string>() != metadata.selected_model) {
      return Result<SchemaFragment>::Failure(Error::InvalidArgument(
          "plugin schema selector does not match the selected model"));
    }
    fragment.selector = FragmentSelector{
        std::string(owned_selector), metadata.selected_model,
        metadata.kind, metadata.plugin_id};
    const auto& fields = json.at("fields");
    if (!fields.is_array() || fields.size() > limits.maximum_fragment_fields)
      return Result<SchemaFragment>::Failure(
          Error::InvalidArgument("plugin schema field limit exceeded"));
    for (const auto& source : fields) {
      if (!HasOnly(source, {"pointer", "type", "required", "default",
                            "allow_unknown_children", "secret", "constraints",
                            "ui"})) {
        return Result<SchemaFragment>::Failure(
            Error::InvalidArgument("unknown plugin schema field member"));
      }
      FieldSpec field;
      field.pointer = source.at("pointer").get<std::string>();
      if (!field.pointer.starts_with(owned_root)) {
        return Result<SchemaFragment>::Failure(Error::InvalidArgument(
            "plugin schema field escapes its selected config domain"));
      }
      auto type = ValueType(source.at("type").get<std::string>());
      if (!type) return Result<SchemaFragment>::Failure(type.GetError());
      field.type = type.Value();
      field.required = source.value("required", false);
      if (source.contains("default")) field.default_value = source.at("default");
      field.allow_unknown_children = source.value("allow_unknown_children", false);
      field.secret = source.value("secret", false);
      if (source.contains("constraints")) {
        const auto& constraints = source.at("constraints");
        if (!HasOnly(constraints,
                     {"minimum", "maximum", "minimum_length",
                      "maximum_length", "minimum_items", "maximum_items",
                      "exclusive_minimum", "exclusive_maximum", "finite",
                      "pattern", "unique_items", "items_non_empty",
                      "allowed_values", "item_type"})) {
          return Result<SchemaFragment>::Failure(Error::InvalidArgument(
              "unknown plugin schema constraint member"));
        }
        OptionalNumber(constraints, "minimum", &field.constraints.minimum);
        OptionalNumber(constraints, "maximum", &field.constraints.maximum);
        OptionalNumber(constraints, "minimum_length", &field.constraints.minimum_length);
        OptionalNumber(constraints, "maximum_length", &field.constraints.maximum_length);
        OptionalNumber(constraints, "minimum_items", &field.constraints.minimum_items);
        OptionalNumber(constraints, "maximum_items", &field.constraints.maximum_items);
        field.constraints.exclusive_minimum = constraints.value("exclusive_minimum", false);
        field.constraints.exclusive_maximum = constraints.value("exclusive_maximum", false);
        field.constraints.finite = constraints.value("finite", true);
        field.constraints.pattern = constraints.value("pattern", std::string{});
        field.constraints.unique_items = constraints.value("unique_items", false);
        field.constraints.items_non_empty = constraints.value("items_non_empty", false);
        if (constraints.contains("allowed_values"))
          field.constraints.allowed_values =
              constraints.at("allowed_values").get<std::vector<nlohmann::json>>();
        if (constraints.contains("item_type")) {
          auto item = ValueType(constraints.at("item_type").get<std::string>());
          if (!item) return Result<SchemaFragment>::Failure(item.GetError());
          field.constraints.item_type = item.Value();
        }
      }
      if (source.contains("ui")) {
        const auto& ui = source.at("ui");
        if (!HasOnly(ui, {"label", "help", "group", "order", "read_only",
                          "advanced"})) {
          return Result<SchemaFragment>::Failure(
              Error::InvalidArgument("unknown plugin schema UI member"));
        }
        field.ui.label = ui.value("label", std::string{});
        field.ui.help = ui.value("help", std::string{});
        field.ui.group = ui.value("group", std::string{});
        field.ui.order = ui.value("order", 0);
        field.ui.read_only = ui.value("read_only", false);
        field.ui.advanced = ui.value("advanced", false);
      }
      fragment.fields.push_back(std::move(field));
    }
    // SchemaRegistry::Create performs pointer, collision, type/default and
    // aggregate bound validation; decoding alone never makes a fragment valid.
    auto validated = SchemaRegistry::Create({fragment}, limits);
    if (!validated)
      return Result<SchemaFragment>::Failure(validated.GetError());
    return Result<SchemaFragment>::Ok(std::move(fragment));
  } catch (const std::exception& error) {
    return Result<SchemaFragment>::Failure(Error::InvalidArgument(
        std::string("invalid plugin schema fragment: ") + error.what()));
  }
}

}  // namespace

Result<SchemaFragment> DecodePluginSchemaFragment(
    const PluginV2Metadata& metadata, SchemaLimits limits) {
  return Decode(metadata, limits);
}

Result<SchemaRegistry> BuildSessionSchemaRegistry(
    std::span<const PluginV2Metadata> plugins, SchemaLimits limits) {
  std::vector<SchemaFragment> fragments;
  constexpr std::array kinds{
      ConfigDocumentKind::kRoot, ConfigDocumentKind::kMapServer,
      ConfigDocumentKind::kDataLoader, ConfigDocumentKind::kLoopDetector,
      ConfigDocumentKind::kBackendOptimizer,
      ConfigDocumentKind::kDynamicRemover};
  for (const auto kind : kinds) {
    auto builtin = BuiltinConfigSchemaRegistry().Fragments(kind);
    fragments.insert(fragments.end(),
                     std::make_move_iterator(builtin.begin()),
                     std::make_move_iterator(builtin.end()));
  }
  for (const auto& plugin : plugins) {
    if ((plugin.capability_bits & OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) == 0)
      continue;
    auto decoded = Decode(plugin, limits);
    if (!decoded) return Result<SchemaRegistry>::Failure(decoded.GetError());
    fragments.push_back(std::move(decoded).Value());
  }
  return SchemaRegistry::Create(std::move(fragments), limits);
}

}  // namespace open_lmm
