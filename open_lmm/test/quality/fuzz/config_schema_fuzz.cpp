#include <open_lmm/utils/config_schema.hpp>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string>
#include <string_view>

namespace {

[[noreturn]] void InvariantFailure() { std::abort(); }

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  if (size < 2 || size > 65536) return 0;

  constexpr std::size_t kind_count = 6;
  const auto kind = static_cast<open_lmm::ConfigDocumentKind>(
      static_cast<std::size_t>(data[0]) % kind_count);
  open_lmm::ValidationOptions options;
  options.reject_unknown_keys = (data[1] & 0x1U) != 0;
  options.apply_defaults = (data[1] & 0x2U) != 0;
  options.migrate_deprecated_keys = (data[1] & 0x4U) != 0;
  const std::string_view input(
      reinterpret_cast<const char*>(data + 2), size - 2);

  const auto& registry = open_lmm::BuiltinConfigSchemaRegistry();
  const auto first = registry.ParseAndValidate(kind, input, "<fuzz>", options);
  if (!first) return 0;

  const std::string canonical = first.Value().CanonicalJson();
  if (canonical.size() > options.limits.maximum_document_bytes) {
    InvariantFailure();
  }
  const auto second = registry.ParseAndValidate(
      kind, canonical, "<fuzz-canonical>", options);
  if (!second || second.Value().Kind() != first.Value().Kind() ||
      second.Value().Version() != first.Value().Version() ||
      second.Value().CanonicalJson() != canonical) {
    InvariantFailure();
  }
  return 0;
}
