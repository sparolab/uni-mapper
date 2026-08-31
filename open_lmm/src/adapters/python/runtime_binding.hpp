#pragma once

#include <pybind11/pybind11.h>

namespace open_lmm::python {

void BindRuntime(pybind11::module_& module);

}  // namespace open_lmm::python
