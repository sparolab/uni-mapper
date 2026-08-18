#include "plugin_host_v2.hpp"

#include <dlfcn.h>

#include <cstring>
#include <limits>
#include <mutex>
#include <utility>

namespace open_lmm {
namespace {

template <typename T>
T Header() {
  T value{};
  value.struct_size = sizeof(T);
  value.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  value.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return value;
}

bool ValidHeader(uint32_t size, uint16_t major, uint16_t minor,
                 std::size_t required) {
  (void)minor;
  return size >= required && major == OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
}

Result<std::string> CopyString(const open_lmm_string_view_v2& value,
                               std::string_view field) {
  if (!ValidHeader(value.struct_size, value.abi_major, value.abi_minor,
                   sizeof(value)) ||
      (value.size != 0 && value.data == nullptr) ||
      value.size > std::numeric_limits<std::size_t>::max()) {
    return Result<std::string>::Failure(Error::PluginLoadFailed(
        "malformed ABI-v2 string view: " + std::string(field)));
  }
  try {
    return Result<std::string>::Ok(std::string(
        value.data ? value.data : "", static_cast<std::size_t>(value.size)));
  } catch (...) {
    return Result<std::string>::Failure(Error::PluginLoadFailed(
        "could not copy ABI-v2 string view: " + std::string(field)));
  }
}

Result<void> ValidateStatus(const open_lmm_status_v2& status,
                            std::string* message) {
  if (!ValidHeader(status.struct_size, status.abi_major, status.abi_minor,
                   sizeof(status))) {
    return Result<void>::Failure(
        Error::PluginLoadFailed("plugin returned malformed ABI-v2 status"));
  }
  auto copied = CopyString(status.message, "status.message");
  if (!copied) return Result<void>::Failure(copied.GetError());
  if (status.code > OPEN_LMM_STATUS_HOST_ERROR_V2) {
    return Result<void>::Failure(
        Error::PluginLoadFailed("plugin returned unknown ABI-v2 status code"));
  }
  *message = std::move(copied).Value();
  return Result<void>::Ok();
}

uint64_t ElementWidth(open_lmm_element_type_v2 type) {
  if (type == OPEN_LMM_ELEMENT_F32_V2) return sizeof(float);
  if (type == OPEN_LMM_ELEMENT_F64_V2) return sizeof(double);
  return 0;
}

template <typename View>
Result<void> ValidateNumericView(const View& view, uint64_t elements,
                                 std::string_view name) {
  const uint64_t width = ElementWidth(view.element_type);
  if (width == 0 ||
      (view.endian != OPEN_LMM_ENDIAN_LITTLE_V2 &&
       view.endian != OPEN_LMM_ENDIAN_BIG_V2) ||
      (view.count != 0 && view.data == nullptr) ||
      (view.count != 0 && view.stride_bytes < elements * width) ||
      (view.count != 0 &&
       view.stride_bytes > std::numeric_limits<uint64_t>::max() / view.count)) {
    return Result<void>::Failure(Error::InvalidArgument(
        "invalid ABI-v2 " + std::string(name) + " view"));
  }
  return Result<void>::Ok();
}

template <typename Function>
Result<Function> Resolve(void* library, const char* symbol,
                         const std::string& path) {
  dlerror();
  void* raw = dlsym(library, symbol);
  const char* error = dlerror();
  if (error || raw == nullptr) {
    return Result<Function>::Failure(Error::PluginLoadFailed(
        "missing '" + std::string(symbol) + "' in " + path + ": " +
        (error ? error : "null symbol")));
  }
  return Result<Function>::Ok(reinterpret_cast<Function>(raw));
}

struct SinkContext {
  std::vector<uint8_t> bytes;
  bool malformed = false;
};

open_lmm_status_code_v2 OPEN_LMM_PLUGIN_CALL_V2 WriteResult(
    void* context, const open_lmm_blob_view_v2* chunk) {
  auto* sink = static_cast<SinkContext*>(context);
  if (!sink || !chunk ||
      !ValidHeader(chunk->struct_size, chunk->abi_major, chunk->abi_minor,
                   sizeof(*chunk)) ||
      chunk->memory_location != OPEN_LMM_MEMORY_HOST_V2 ||
      (chunk->size != 0 && chunk->data == nullptr) ||
      chunk->size > static_cast<uint64_t>(
                        std::numeric_limits<std::ptrdiff_t>::max()) ||
      chunk->size > std::numeric_limits<std::size_t>::max() - sink->bytes.size()) {
    if (sink) sink->malformed = true;
    return OPEN_LMM_STATUS_HOST_ERROR_V2;
  }
  const auto* first = static_cast<const uint8_t*>(chunk->data);
  if (chunk->size != 0) {
    try {
      sink->bytes.insert(sink->bytes.end(), first, first + chunk->size);
    } catch (...) {
      sink->malformed = true;
      return OPEN_LMM_STATUS_HOST_ERROR_V2;
    }
  }
  return OPEN_LMM_STATUS_OK_V2;
}

}  // namespace

struct PluginV2::Impl {
  ~Impl() {
    if (handle && close) {
      try { close(handle); } catch (...) {}
      handle = nullptr;
    }
    if (library) dlclose(library);
  }
  void* library = nullptr;
  open_lmm_plugin_handle_v2* handle = nullptr;
  open_lmm_plugin_call_fn_v2 call = nullptr;
  open_lmm_plugin_close_fn_v2 close = nullptr;
  PluginV2Metadata metadata;
  open_lmm_host_api_v2 host{};
  std::mutex call_mutex;
};

PluginV2::PluginV2(std::unique_ptr<Impl> impl) : impl_(std::move(impl)) {}
PluginV2::~PluginV2() = default;
PluginV2::PluginV2(PluginV2&&) noexcept = default;
PluginV2& PluginV2::operator=(PluginV2&&) noexcept = default;

const PluginV2Metadata& PluginV2::Metadata() const noexcept {
  return impl_->metadata;
}

Result<std::vector<uint8_t>> PluginV2::Call(const PluginV2Call& request) {
  if (!impl_ || !impl_->handle || !impl_->call || request.operation.empty() ||
      (request.request_size != 0 && request.request_data == nullptr)) {
    return Result<std::vector<uint8_t>>::Failure(
        Error::InvalidArgument("invalid ABI-v2 plugin call"));
  }
  if (request.points) {
    if ((impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_POINT_VIEW_V2) == 0) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 plugin does not advertise point-view support"));
    }
    auto valid = ValidateNumericView(*request.points, 3, "point");
    if (!valid)
      return Result<std::vector<uint8_t>>::Failure(valid.GetError());
  }
  if (request.poses) {
    if ((impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_POSE_VIEW_V2) == 0) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 plugin does not advertise pose-view support"));
    }
    auto valid = ValidateNumericView(*request.poses, 16, "pose");
    if (!valid)
      return Result<std::vector<uint8_t>>::Failure(valid.GetError());
  }
  std::unique_lock<std::mutex> serialized(impl_->call_mutex, std::defer_lock);
  if ((impl_->metadata.capability_bits &
       OPEN_LMM_CAPABILITY_CONCURRENT_CALLS_V2) == 0) {
    serialized.lock();
  }
  auto call = Header<open_lmm_call_v2>();
  call.operation = Header<open_lmm_string_view_v2>();
  call.operation.data = request.operation.data();
  call.operation.size = request.operation.size();
  call.request = Header<open_lmm_blob_view_v2>();
  call.request.data = request.request_data;
  call.request.size = request.request_size;
  call.request.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  call.points = Header<open_lmm_point_view_v2>();
  call.points.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (request.points) {
    call.points.data = request.points->data;
    call.points.count = request.points->count;
    call.points.stride_bytes = request.points->stride_bytes;
    call.points.element_type = request.points->element_type;
    call.points.endian = request.points->endian;
  }
  call.poses = Header<open_lmm_pose_view_v2>();
  call.poses.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (request.poses) {
    call.poses.data = request.poses->data;
    call.poses.count = request.poses->count;
    call.poses.stride_bytes = request.poses->stride_bytes;
    call.poses.element_type = request.poses->element_type;
    call.poses.endian = request.poses->endian;
  }
  SinkContext context;
  auto sink = Header<open_lmm_result_sink_v2>();
  sink.host_context = &context;
  sink.write = &WriteResult;
  open_lmm_status_v2 status{};
  try {
    status = impl_->call(impl_->handle, &call, &sink);
  } catch (...) {
    return Result<std::vector<uint8_t>>::Failure(Error::PluginLoadFailed(
        "ABI-v2 plugin call threw an exception").WithPlugin(impl_->metadata.name));
  }
  std::string message;
  auto valid = ValidateStatus(status, &message);
  if (!valid) return Result<std::vector<uint8_t>>::Failure(valid.GetError());
  if (context.malformed) {
    return Result<std::vector<uint8_t>>::Failure(Error::PluginLoadFailed(
        "ABI-v2 plugin wrote a malformed result chunk")
                                                     .WithPlugin(impl_->metadata.name));
  }
  if (status.code != OPEN_LMM_STATUS_OK_V2) {
    if (status.code == OPEN_LMM_STATUS_CANCELLED_V2) {
      return Result<std::vector<uint8_t>>::Failure(Error::Cancelled(message));
    }
    return Result<std::vector<uint8_t>>::Failure(
        Error::PluginLoadFailed(message.empty() ? "ABI-v2 plugin call failed" : message)
            .WithPlugin(impl_->metadata.name));
  }
  return Result<std::vector<uint8_t>>::Ok(std::move(context.bytes));
}

Result<PluginV2> LoadPluginV2(const std::string& path,
                              std::string_view expected_kind,
                              std::string_view config_json,
                              uint64_t required_capability_bits,
                              void* host_context,
                              open_lmm_is_cancelled_fn_v2 is_cancelled) {
  void* library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library) {
    const char* error = dlerror();
    return Result<PluginV2>::Failure(Error::PluginLoadFailed(
        "dlopen failed for " + path + ": " + (error ? error : "unknown error")));
  }
  auto impl = std::make_unique<PluginV2::Impl>();
  impl->library = library;
  auto query = Resolve<open_lmm_plugin_query_fn_v2>(library,
      OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2, path);
  auto open = Resolve<open_lmm_plugin_open_fn_v2>(library,
      OPEN_LMM_PLUGIN_OPEN_SYMBOL_V2, path);
  auto call = Resolve<open_lmm_plugin_call_fn_v2>(library,
      OPEN_LMM_PLUGIN_CALL_SYMBOL_V2, path);
  auto close = Resolve<open_lmm_plugin_close_fn_v2>(library,
      OPEN_LMM_PLUGIN_CLOSE_SYMBOL_V2, path);
  if (!query) return Result<PluginV2>::Failure(query.GetError());
  if (!open) return Result<PluginV2>::Failure(open.GetError());
  if (!call) return Result<PluginV2>::Failure(call.GetError());
  if (!close) return Result<PluginV2>::Failure(close.GetError());
  impl->call = call.Value();
  impl->close = close.Value();

  auto descriptor = Header<open_lmm_plugin_descriptor_v2>();
  open_lmm_status_v2 query_status{};
  try { query_status = query.Value()(&descriptor); }
  catch (...) {
    return Result<PluginV2>::Failure(
        Error::PluginLoadFailed("ABI-v2 plugin query threw an exception"));
  }
  std::string message;
  auto valid_status = ValidateStatus(query_status, &message);
  if (!valid_status) return Result<PluginV2>::Failure(valid_status.GetError());
  if (query_status.code != OPEN_LMM_STATUS_OK_V2 ||
      !ValidHeader(descriptor.struct_size, descriptor.abi_major,
                   descriptor.abi_minor, sizeof(descriptor)) ||
      descriptor.minimum_host_minor > OPEN_LMM_PLUGIN_ABI_V2_MINOR) {
    return Result<PluginV2>::Failure(Error::PluginLoadFailed(
        message.empty() ? "incompatible ABI-v2 plugin descriptor" : message));
  }
  auto kind = CopyString(descriptor.plugin_kind, "plugin_kind");
  auto name = CopyString(descriptor.plugin_name, "plugin_name");
  auto capability = CopyString(descriptor.capability, "capability");
  if (!kind) return Result<PluginV2>::Failure(kind.GetError());
  if (!name) return Result<PluginV2>::Failure(name.GetError());
  if (!capability) return Result<PluginV2>::Failure(capability.GetError());
  if (kind.Value() != expected_kind || name.Value().empty()) {
    return Result<PluginV2>::Failure(
        Error::PluginLoadFailed("ABI-v2 plugin kind/name mismatch"));
  }
  if ((descriptor.capability_bits & required_capability_bits) !=
      required_capability_bits) {
    return Result<PluginV2>::Failure(
        Error::PluginLoadFailed("ABI-v2 plugin lacks required capabilities"));
  }
  impl->metadata = {kind.Value(), name.Value(), capability.Value(),
                    descriptor.capability_bits, descriptor.abi_minor};

  impl->host = Header<open_lmm_host_api_v2>();
  impl->host.host_context = host_context;
  impl->host.is_cancelled = is_cancelled;
  auto config = Header<open_lmm_config_view_v2>();
  config.json = Header<open_lmm_string_view_v2>();
  config.json.data = config_json.data();
  config.json.size = config_json.size();
  open_lmm_plugin_handle_v2* partial = nullptr;
  open_lmm_status_v2 open_status{};
  try { open_status = open.Value()(&impl->host, &config, &partial); }
  catch (...) {
    if (partial) { try { impl->close(partial); } catch (...) {} }
    return Result<PluginV2>::Failure(
        Error::PluginLoadFailed("ABI-v2 plugin open threw an exception"));
  }
  valid_status = ValidateStatus(open_status, &message);
  if (!valid_status || open_status.code != OPEN_LMM_STATUS_OK_V2 || !partial) {
    if (partial) { try { impl->close(partial); } catch (...) {} }
    if (!valid_status) return Result<PluginV2>::Failure(valid_status.GetError());
    return Result<PluginV2>::Failure(Error::PluginLoadFailed(
        message.empty() ? "ABI-v2 plugin open failed" : message));
  }
  impl->handle = partial;
  return Result<PluginV2>::Ok(PluginV2(std::move(impl)));
}

}  // namespace open_lmm
