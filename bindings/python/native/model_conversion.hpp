#pragma once

#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/runtime_api.hpp>

#include <pybind11/pybind11.h>

#include <exception>
#include <optional>
#include <vector>

namespace open_lmm::python {

namespace py = pybind11;

py::dict ErrorPayload(const Error& error);
py::dict ExecutionEventPayload(const ExecutionEvent& event);
py::dict RuntimeSnapshotPayload(const RuntimeSnapshot& snapshot);
py::dict ConfigApplyReceiptPayload(const ConfigApplyReceipt& receipt);
py::dict RuntimeReplaceReceiptPayload(const RuntimeReplaceReceipt& receipt);
py::dict ConfigDocumentsPayload(const CommittedConfigDocuments& documents);
py::dict ConfigCandidatesPayload(const ConfigCandidateCatalog& catalog);
py::list NodeDescriptorsPayload(const std::vector<NodeDescriptor>& descriptors);
py::object AlignmentFeedbackPayload(
    std::optional<AlignmentFeedbackSnapshot> snapshot);

[[noreturn]] void RaiseError(const Error& error);
[[noreturn]] void RaiseInternal(std::exception_ptr exception);

}  // namespace open_lmm::python
