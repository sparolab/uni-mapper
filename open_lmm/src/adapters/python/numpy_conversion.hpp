#pragma once

#include <open_lmm/common/visualization_snapshot.hpp>

#include <pybind11/pybind11.h>

namespace open_lmm::python {

pybind11::dict VisualizationPayload(VisualizationSnapshot snapshot);

}  // namespace open_lmm::python
