#include <open_lmm/common/agent_id.hpp>

#include <open_lmm/common/result.hpp>

#include <algorithm>
#include <set>

namespace open_lmm {
namespace {

std::string AsciiLower(std::string_view value) {
  std::string result;
  result.reserve(value.size());
  for (const unsigned char character : value) {
    result.push_back(static_cast<char>(
        character >= 'A' && character <= 'Z' ? character - 'A' + 'a'
                                              : character));
  }
  return result;
}

bool IsAsciiAlphanumeric(unsigned char character) {
  return (character >= 'a' && character <= 'z') ||
         (character >= 'A' && character <= 'Z') ||
         (character >= '0' && character <= '9');
}

}  // namespace

Result<AgentId> AgentId::Parse(std::string_view value) {
  if (value.empty() || value.size() > 64) {
    return Result<AgentId>::Failure(
        Error::InvalidArgument("AgentId length must be between 1 and 64"));
  }
  if (value == "." || value == ".." ||
      !IsAsciiAlphanumeric(static_cast<unsigned char>(value.front()))) {
    return Result<AgentId>::Failure(Error::InvalidArgument(
        "AgentId must start with an ASCII alphanumeric character"));
  }
  for (const unsigned char character : value) {
    if (!IsAsciiAlphanumeric(character) && character != '_' &&
        character != '-' && character != '.') {
      return Result<AgentId>::Failure(Error::InvalidArgument(
          "AgentId may contain only ASCII alphanumeric, '_', '-', and '.'"));
    }
  }
  return Result<AgentId>::Ok(AgentId(std::string(value)));
}

std::ostream& operator<<(std::ostream& stream, const AgentId& id) {
  return stream << id.Value();
}

Result<AgentSymbol> AgentSymbol::FromByte(uint16_t value) {
  if (value == 0 || value > 255) {
    return Result<AgentSymbol>::Failure(
        Error::InvalidArgument("agent symbol byte must be in range 1..255"));
  }
  return Result<AgentSymbol>::Ok(
      AgentSymbol(static_cast<unsigned char>(value)));
}

Result<AgentSymbolCatalog> AgentSymbolCatalog::Build(
    std::vector<AgentId> configured_order) {
  if (configured_order.empty() || configured_order.size() > kMaximumAgents) {
    return Result<AgentSymbolCatalog>::Failure(Error::InvalidArgument(
        "agent catalog size must be in range 1..255"));
  }
  std::map<AgentId, AgentSymbol> by_id;
  std::set<std::string> case_folded;
  for (std::size_t index = 0; index < configured_order.size(); ++index) {
    const auto& id = configured_order[index];
    if (!id.IsValid()) {
      return Result<AgentSymbolCatalog>::Failure(
          Error::InvalidArgument("agent catalog contains an invalid AgentId"));
    }
    if (!case_folded.insert(AsciiLower(id.Value())).second) {
      return Result<AgentSymbolCatalog>::Failure(Error::InvalidArgument(
          "agent catalog contains a duplicate or case-fold duplicate: " +
          id.Value()));
    }
    auto symbol = AgentSymbol::FromByte(index + 1);
    if (!symbol)
      return Result<AgentSymbolCatalog>::Failure(symbol.GetError());
    by_id.emplace(id, symbol.Value());
  }
  return Result<AgentSymbolCatalog>::Ok(
      AgentSymbolCatalog(std::move(configured_order), std::move(by_id)));
}

Result<AgentSymbol> AgentSymbolCatalog::SymbolFor(const AgentId& id) const {
  const auto found = by_id_.find(id);
  if (found == by_id_.end()) {
    return Result<AgentSymbol>::Failure(
        Error::InvalidArgument("unknown AgentId: " + id.Value()));
  }
  return Result<AgentSymbol>::Ok(found->second);
}

Result<AgentId> AgentSymbolCatalog::AgentFor(AgentSymbol symbol) const {
  if (!symbol.IsValid() || symbol.Byte() > ordered_ids_.size()) {
    return Result<AgentId>::Failure(Error::InvalidArgument(
        "unknown agent symbol byte: " + std::to_string(symbol.Byte())));
  }
  return Result<AgentId>::Ok(ordered_ids_[symbol.Byte() - 1]);
}

std::string AgentSymbolCatalog::Format(AgentSymbol symbol,
                                       uint64_t frame_index,
                                       uint64_t anchor_index) const {
  auto id = AgentFor(symbol);
  if (!id) return "unknown-agent/" + std::to_string(frame_index);
  return id.Value().Value() + "/" +
         (frame_index == anchor_index ? "anchor"
                                      : std::to_string(frame_index));
}

}  // namespace open_lmm
