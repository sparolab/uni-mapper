#include "runtime_contracts.hpp"

namespace open_lmm {

Result<SessionId> SessionId::Parse(std::string_view value) {
  if (value.size() != 36) {
    return Result<SessionId>::Failure(
        Error::InvalidArgument("SessionId must be a canonical 128-bit UUID"));
  }
  for (std::size_t index = 0; index < value.size(); ++index) {
    const bool separator = index == 8 || index == 13 || index == 18 || index == 23;
    const char character = value[index];
    if ((separator && character != '-') ||
        (!separator && !((character >= '0' && character <= '9') ||
                         (character >= 'a' && character <= 'f')))) {
      return Result<SessionId>::Failure(Error::InvalidArgument(
          "SessionId must be a canonical lowercase UUID"));
    }
  }
  return Result<SessionId>::Ok(SessionId(std::string(value)));
}

}  // namespace open_lmm
