#include "registration_log.hpp"

#include <spdlog/spdlog.h>

namespace open_lmm {

void LogRegistrationDidNotConverge() {
  spdlog::debug("registration rejected: solver did not converge");
}

void LogRegistrationFitnessRejected(double fitness_score) {
  spdlog::debug("registration rejected: fitness score {} exceeds 0.5",
                fitness_score);
}

}  // namespace open_lmm
