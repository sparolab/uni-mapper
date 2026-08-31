#include "runtime_binding.hpp"

#include "model_conversion.hpp"
#include "numpy_conversion.hpp"

#include <open_lmm/server/runtime_client.hpp>

#include <pybind11/stl.h>

#include <atomic>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>
#include <utility>

namespace open_lmm::python {
namespace py = pybind11;
namespace fs = std::filesystem;
namespace {

template <typename Function>
auto CallNative(Function&& function) -> std::invoke_result_t<Function> {
  using Return = std::invoke_result_t<Function>;
  std::optional<Return> result;
  std::exception_ptr exception;
  {
    py::gil_scoped_release release;
    try {
      result.emplace(std::invoke(std::forward<Function>(function)));
    } catch (...) {
      exception = std::current_exception();
    }
  }
  if (exception) RaiseInternal(exception);
  return std::move(*result);
}

template <typename T>
T Unwrap(Result<T> result) {
  if (!result) RaiseError(result.GetError());
  return std::move(result).Value();
}

void Unwrap(Result<void> result) {
  if (!result) RaiseError(result.GetError());
}

AgentId ParseAgent(const std::string& value) {
  auto parsed = AgentId::Parse(value);
  if (!parsed) RaiseError(parsed.GetError());
  return std::move(parsed).Value();
}

class RuntimeHolder {
 public:
  explicit RuntimeHolder(std::size_t max_agent_tasks)
      : client_(std::make_unique<RuntimeClient>(max_agent_tasks)) {}

  ~RuntimeHolder() {
    auto* retiring = client_.release();
    if (!retiring) return;
    if (!retiring->IsOpen() || !Py_IsInitialized() || !PyGILState_Check()) {
      delete retiring;
      return;
    }
    // RuntimeClient destruction can wait for a Python event callback. Never
    // hold the GIL while that drain occurs. Explicit close/atexit remains the
    // deterministic path; this is the GC safety fallback.
    try {
      std::thread([retiring] { delete retiring; }).detach();
    } catch (...) {
      // A bounded process-exit leak is safer than a GIL/callback deadlock.
    }
  }

  RuntimeClient& Client() { return *client_; }
  const RuntimeClient& Client() const { return *client_; }

 private:
  std::unique_ptr<RuntimeClient> client_;
};

class NativeJob {
 public:
  NativeJob(std::weak_ptr<RuntimeHolder> owner, JobHandle handle)
      : owner_(std::move(owner)), handle_(handle) {}

  uint64_t Id() const { return handle_.value; }

  void Wait() {
    auto owner = RequireOwner();
    Unwrap(CallNative([owner, handle = handle_] {
      return owner->Client().Wait(handle);
    }));
  }

  void Cancel() {
    auto owner = RequireOwner();
    Unwrap(CallNative([owner, handle = handle_] {
      return owner->Client().Cancel(handle);
    }));
  }

  bool BelongsTo(const std::shared_ptr<RuntimeHolder>& owner) const {
    return owner_.lock() == owner;
  }

  JobHandle Handle() const { return handle_; }

 private:
  std::shared_ptr<RuntimeHolder> RequireOwner() const {
    auto owner = owner_.lock();
    if (!owner) {
      RaiseError(Error::InvalidArgument(
          "job owner no longer exists; keep Runtime alive until job use ends"));
    }
    return owner;
  }

  std::weak_ptr<RuntimeHolder> owner_;
  JobHandle handle_;
};

class PythonCallbackState {
 public:
  explicit PythonCallbackState(const py::object& callback)
      : callback_(callback.ptr()) {
    Py_INCREF(callback_.load());
  }

  ~PythonCallbackState() { ClearSafely(); }

  void Deactivate() noexcept { active_.store(false, std::memory_order_release); }

  void Invoke(const ExecutionEvent& event) {
    if (!active_.load(std::memory_order_acquire)) return;
    py::gil_scoped_acquire acquire;
    if (!active_.load(std::memory_order_acquire)) return;
    PyObject* raw = callback_.load(std::memory_order_acquire);
    if (!raw) return;
    py::object callback = py::reinterpret_borrow<py::object>(raw);
    try {
      callback(ExecutionEventPayload(event));
    } catch (py::error_already_set& error) {
      error.discard_as_unraisable(callback);
    }
  }

  void ClearWithGilHeld() noexcept {
    PyObject* raw = callback_.exchange(nullptr, std::memory_order_acq_rel);
    Py_XDECREF(raw);
  }

 private:
  void ClearSafely() noexcept {
    PyObject* raw = callback_.exchange(nullptr, std::memory_order_acq_rel);
    if (!raw || !Py_IsInitialized()) return;
    try {
      if (PyGILState_Check()) {
        Py_DECREF(raw);
      } else {
        py::gil_scoped_acquire acquire;
        Py_DECREF(raw);
      }
    } catch (...) {
      // Interpreter finalization may make reacquisition unsafe. Leaking this
      // last reference is preferable to touching a finalized interpreter.
    }
  }

  std::atomic<bool> active_{true};
  std::atomic<PyObject*> callback_{nullptr};
};

class NativeSubscription {
 public:
  NativeSubscription(ExecutionEventSubscription subscription,
                     std::shared_ptr<PythonCallbackState> callback)
      : subscription_(std::move(subscription)), callback_(std::move(callback)) {}

  ~NativeSubscription() { CloseNoexcept(); }

  void Close() {
    if (closed_.exchange(true, std::memory_order_acq_rel)) return;
    callback_->Deactivate();
    {
      py::gil_scoped_release release;
      subscription_.Reset();
    }
    callback_->ClearWithGilHeld();
  }

 private:
  void CloseNoexcept() noexcept {
    if (closed_.exchange(true, std::memory_order_acq_rel)) return;
    callback_->Deactivate();
    try {
      if (Py_IsInitialized() && PyGILState_Check()) {
        {
          py::gil_scoped_release release;
          subscription_.Reset();
        }
        callback_->ClearWithGilHeld();
      } else {
        subscription_.Reset();
      }
    } catch (...) {
    }
  }

  std::atomic<bool> closed_{false};
  ExecutionEventSubscription subscription_;
  std::shared_ptr<PythonCallbackState> callback_;
};

class NativeRuntime {
 public:
  explicit NativeRuntime(std::size_t max_agent_tasks)
      : holder_(std::make_shared<RuntimeHolder>(max_agent_tasks)) {}

  void Open(const std::string& config_directory, const std::string& label,
            const std::optional<std::string>& output_root) {
    BootstrapRequest request;
    request.config_directory = fs::path(config_directory);
    request.label = label;
    if (output_root) request.output_root = fs::path(*output_root);
    auto holder = holder_;
    Unwrap(CallNative([holder, request = std::move(request)] {
      return holder->Client().Open(request);
    }));
  }

  std::shared_ptr<NativeJob> RunAll() {
    auto holder = holder_;
    auto handle = Unwrap(CallNative(
        [holder] { return holder->Client().SubmitRunAll(); }));
    return std::make_shared<NativeJob>(holder, handle);
  }

  std::shared_ptr<NativeJob> RunStage(
      int stage, const std::optional<std::string>& agent) {
    if (stage < static_cast<int>(StageId::kDataLoad) ||
        stage > static_cast<int>(StageId::kSave)) {
      RaiseError(Error::InvalidArgument("invalid Python stage value"));
    }
    ExecutionRequest request;
    request.kind = ExecutionRequestKind::kStage;
    request.stage = static_cast<StageId>(stage);
    if (agent) request.agent = ParseAgent(*agent);
    auto holder = holder_;
    auto handle = Unwrap(CallNative([holder, request = std::move(request)] {
      return holder->Client().Submit(request);
    }));
    return std::make_shared<NativeJob>(holder, handle);
  }

  py::dict Snapshot() const {
    auto holder = holder_;
    auto snapshot = Unwrap(CallNative(
        [holder] { return holder->Client().Snapshot(); }));
    return RuntimeSnapshotPayload(snapshot);
  }

  py::dict Visualization(const std::string& agent, bool include_points,
                         float preview_voxel_size_m) const {
    VisualizationQuery query;
    query.agent = ParseAgent(agent);
    query.include_points = include_points;
    query.preview_voxel_size_m = preview_voxel_size_m;
    auto holder = holder_;
    auto snapshot = Unwrap(CallNative([holder, query] {
      return holder->Client().Visualization(query);
    }));
    return VisualizationPayload(std::move(snapshot));
  }

  py::dict ApplyConfig(int domain, const std::string& document_json,
                       uint64_t expected_runtime_revision,
                       uint64_t expected_config_revision,
                       const std::optional<std::string>& selected_document) {
    if (domain < static_cast<int>(ConfigDomain::kGlobal) ||
        domain > static_cast<int>(ConfigDomain::kMapSave)) {
      RaiseError(Error::InvalidArgument("invalid Python config domain value"));
    }
    ConfigCandidate candidate;
    candidate.domain = static_cast<ConfigDomain>(domain);
    candidate.document_json = document_json;
    if (selected_document) candidate.selected_document = fs::path(*selected_document);
    const ExpectedRevision expected{expected_runtime_revision,
                                    expected_config_revision};
    auto holder = holder_;
    auto receipt = Unwrap(CallNative(
        [holder, candidate = std::move(candidate), expected] {
          return holder->Client().ApplyConfig(candidate, expected);
        }));
    return ConfigApplyReceiptPayload(receipt);
  }

  std::shared_ptr<NativeSubscription> SubscribeEvents(
      const py::object& callback) {
    if (callback.is_none() || !PyCallable_Check(callback.ptr())) {
      RaiseError(Error::InvalidArgument("event callback must be callable"));
    }
    auto callback_state = std::make_shared<PythonCallbackState>(callback);
    auto holder = holder_;
    auto result = CallNative([holder, callback_state] {
      return holder->Client().SubscribeEvents(
          [callback_state](const ExecutionEvent& event) {
            callback_state->Invoke(event);
          });
    });
    auto subscription = Unwrap(std::move(result));
    return std::make_shared<NativeSubscription>(std::move(subscription),
                                                std::move(callback_state));
  }

  void Cancel(const std::shared_ptr<NativeJob>& job) {
    if (!job || !job->BelongsTo(holder_)) {
      RaiseError(Error::InvalidArgument(
          "job belongs to a different OpenLMM Runtime"));
    }
    auto holder = holder_;
    const auto handle = job->Handle();
    Unwrap(CallNative(
        [holder, handle] { return holder->Client().Cancel(handle); }));
  }

  void Close(bool cancel_running) {
    auto holder = holder_;
    const auto mode = cancel_running ? CloseMode::kCancelAndWait
                                     : CloseMode::kRejectIfRunning;
    Unwrap(CallNative(
        [holder, mode] { return holder->Client().Close(mode); }));
  }

  bool IsOpen() const {
    auto holder = holder_;
    return CallNative([holder] { return holder->Client().IsOpen(); });
  }

 private:
  std::shared_ptr<RuntimeHolder> holder_;
};

}  // namespace

void BindRuntime(py::module_& module) {
  py::class_<NativeJob, std::shared_ptr<NativeJob>>(module, "_NativeJob")
      .def_property_readonly("id", &NativeJob::Id)
      .def("wait", &NativeJob::Wait)
      .def("cancel", &NativeJob::Cancel);

  py::class_<NativeSubscription, std::shared_ptr<NativeSubscription>>(
      module, "_NativeSubscription")
      .def("close", &NativeSubscription::Close);

  py::class_<NativeRuntime>(module, "NativeRuntime")
      .def(py::init([](py::ssize_t max_agent_tasks) {
        if (max_agent_tasks <= 0) {
          RaiseError(Error::InvalidArgument(
              "max_agent_tasks must be greater than zero"));
        }
        return std::make_unique<NativeRuntime>(
            static_cast<std::size_t>(max_agent_tasks));
      }), py::arg("max_agent_tasks") = 4)
      .def("open", &NativeRuntime::Open, py::arg("config_directory"),
           py::arg("label"), py::arg("output_root"))
      .def("run_all", &NativeRuntime::RunAll)
      .def("run_stage", &NativeRuntime::RunStage, py::arg("stage"),
           py::arg("agent"))
      .def("snapshot", &NativeRuntime::Snapshot)
      .def("visualization", &NativeRuntime::Visualization,
           py::arg("agent"), py::arg("include_points"),
           py::arg("preview_voxel_size_m"))
      .def("apply_config", &NativeRuntime::ApplyConfig, py::arg("domain"),
           py::arg("document_json"), py::arg("expected_runtime_revision"),
           py::arg("expected_config_revision"),
           py::arg("selected_document"))
      .def("subscribe_events", &NativeRuntime::SubscribeEvents)
      .def("cancel", &NativeRuntime::Cancel)
      .def("close", &NativeRuntime::Close)
      .def("is_open", &NativeRuntime::IsOpen);
}

}  // namespace open_lmm::python
