#include "runtime_binding.hpp"

#include <pybind11/pybind11.h>

#ifndef OPEN_LMM_PYTHON_VERSION
#error "OPEN_LMM_PYTHON_VERSION must be supplied by the OpenLMM build"
#endif

PYBIND11_MODULE(_native, module) {
  module.doc() = "Private native bridge for the OpenLMM Python API v1";
  module.attr("API_VERSION") = 1;
  module.attr("__version__") = OPEN_LMM_PYTHON_VERSION;
  open_lmm::python::BindRuntime(module);
}
