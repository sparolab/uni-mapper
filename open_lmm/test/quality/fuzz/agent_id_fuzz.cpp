#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/result.hpp>

#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <string_view>
#include <vector>

namespace {

bool IsAsciiAlphanumeric(unsigned char value) {
  return (value >= 'a' && value <= 'z') ||
         (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9');
}

[[noreturn]] void InvariantFailure() { std::abort(); }

}  // namespace

extern "C" int LLVMFuzzerTestOneInput(const uint8_t* data, std::size_t size) {
  if (size > 128) return 0;
  const std::string_view input(reinterpret_cast<const char*>(data), size);
  const auto parsed = open_lmm::AgentId::Parse(input);
  if (!parsed) return 0;

  const auto& id = parsed.Value();
  const auto& value = id.Value();
  if (value.empty() || value.size() > 64 ||
      !IsAsciiAlphanumeric(static_cast<unsigned char>(value.front()))) {
    InvariantFailure();
  }
  for (const unsigned char character : value) {
    if (!IsAsciiAlphanumeric(character) && character != '_' &&
        character != '-' && character != '.') {
      InvariantFailure();
    }
  }

  const auto reparsed = open_lmm::AgentId::Parse(value);
  if (!reparsed || reparsed.Value() != id) InvariantFailure();
  const auto catalog = open_lmm::AgentSymbolCatalog::Build({id});
  if (!catalog) InvariantFailure();
  const auto symbol = catalog.Value().SymbolFor(id);
  if (!symbol) InvariantFailure();
  const auto round_trip = catalog.Value().AgentFor(symbol.Value());
  if (!round_trip || round_trip.Value() != id) InvariantFailure();
  return 0;
}
