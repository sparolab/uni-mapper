#include "registration_log.hpp"

#include <open_lmm/utils/logging.hpp>
#include <sstream>

namespace open_lmm {

void LogRegistrationDidNotConverge() {
  LogDebug("registration rejected: solver did not converge");
}

void LogRegistrationFitnessRejected(double fitness_score) {
  std::ostringstream message;
  message << "registration rejected: fitness score " << fitness_score
          << " exceeds 0.5";
  LogDebug(message.str());
}

}  // namespace open_lmm
