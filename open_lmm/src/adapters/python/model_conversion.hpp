#pragma once

#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/common/runtime_api.hpp>

#include <pybind11/pybind11.h>

#include <exception>

namespace open_lmm::python {

namespace py = pybind11;

py::dict ErrorPayload(const Error& error);
py::dict ExecutionEventPayload(const ExecutionEvent& event);
py::dict RuntimeSnapshotPayload(const RuntimeSnapshot& snapshot);
py::dict ConfigApplyReceiptPayload(const ConfigApplyReceipt& receipt);

[[noreturn]] void RaiseError(const Error& error);
[[noreturn]] void RaiseInternal(std::exception_ptr exception);

}  // namespace open_lmm::python
