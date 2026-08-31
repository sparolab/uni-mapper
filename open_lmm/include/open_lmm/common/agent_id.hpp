#pragma once

#include <compare>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <ostream>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace open_lmm {

template <typename T>
class Result;

class AgentId {
 public:
  AgentId() = default;

  [[nodiscard]] static Result<AgentId> Parse(std::string_view value);
  [[nodiscard]] const std::string& Value() const noexcept { return value_; }
  [[nodiscard]] bool IsValid() const noexcept { return !value_.empty(); }

  friend bool operator==(const AgentId&, const AgentId&) = default;
  friend auto operator<=>(const AgentId&, const AgentId&) = default;

 private:
  explicit AgentId(std::string value) : value_(std::move(value)) {}
  std::string value_;
};

std::ostream& operator<<(std::ostream& stream, const AgentId& id);

class AgentSymbol {
 public:
  AgentSymbol() = default;

  [[nodiscard]] static Result<AgentSymbol> FromByte(uint16_t value);
  [[nodiscard]] unsigned char Byte() const noexcept { return byte_; }
  [[nodiscard]] bool IsValid() const noexcept { return byte_ != 0; }

  friend bool operator==(const AgentSymbol&, const AgentSymbol&) = default;
  friend auto operator<=>(const AgentSymbol&, const AgentSymbol&) = default;

 private:
  explicit AgentSymbol(unsigned char byte) : byte_(byte) {}
  unsigned char byte_ = 0;
};

class AgentSymbolCatalog {
 public:
  static constexpr uint32_t kVersion = 1;
  static constexpr std::size_t kMaximumAgents = 255;

  [[nodiscard]] static Result<AgentSymbolCatalog> Build(
      std::vector<AgentId> configured_order);
  [[nodiscard]] Result<AgentSymbol> SymbolFor(const AgentId& id) const;
  [[nodiscard]] Result<AgentId> AgentFor(AgentSymbol symbol) const;
  [[nodiscard]] const std::vector<AgentId>& OrderedIds() const noexcept {
    return ordered_ids_;
  }
  [[nodiscard]] std::string Format(AgentSymbol symbol,
                                   uint64_t frame_index,
                                   uint64_t anchor_index) const;

 private:
  explicit AgentSymbolCatalog(std::vector<AgentId> ordered_ids,
                              std::map<AgentId, AgentSymbol> by_id)
      : ordered_ids_(std::move(ordered_ids)), by_id_(std::move(by_id)) {}

  std::vector<AgentId> ordered_ids_;
  std::map<AgentId, AgentSymbol> by_id_;
};

using AgentSymbolCatalogHandle = std::shared_ptr<const AgentSymbolCatalog>;

}  // namespace open_lmm
