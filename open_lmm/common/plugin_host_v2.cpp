#include "plugin_host_v2.hpp"

#include <dlfcn.h>

#include <atomic>
#include <cstring>
#include <algorithm>
#include <limits>
#include <mutex>
#include <shared_mutex>
#include <unordered_set>
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
      value.size > std::numeric_limits<std::size_t>::max() ||
      value.size > UINT64_C(1024) * 1024) {
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

uint32_t DescriptorMinimumPrefix(uint16_t minor) {
  return minor == 0 ? OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_0_SIZE
                    : OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_1_SIZE;
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
  bool limit_exceeded = false;
  uint64_t chunk_count = 0;
  PluginV2ResultLimits limits;
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
  ++sink->chunk_count;
  if (chunk->size > sink->limits.maximum_chunk_bytes ||
      sink->chunk_count > sink->limits.maximum_chunk_count ||
      chunk->size > sink->limits.maximum_result_bytes - sink->bytes.size()) {
    sink->limit_exceeded = true;
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
  static int32_t OPEN_LMM_PLUGIN_CALL_V2 IsCancelled(void* context) {
    auto* self = static_cast<Impl*>(context);
    const auto active = self->active_is_cancelled.load(
        std::memory_order_acquire);
    const auto callback = active ? active : self->default_is_cancelled;
    void* callback_context = active
        ? self->active_cancellation_context.load(std::memory_order_acquire)
        : self->default_cancellation_context;
    return callback ? callback(callback_context) : 0;
  }

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
  std::shared_mutex cancellation_mutex;
  void* default_cancellation_context = nullptr;
  open_lmm_is_cancelled_fn_v2 default_is_cancelled = nullptr;
  std::atomic<void*> active_cancellation_context{nullptr};
  std::atomic<open_lmm_is_cancelled_fn_v2> active_is_cancelled{nullptr};
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
  if (request.result_limits.maximum_result_bytes == 0 ||
      request.result_limits.maximum_chunk_bytes == 0 ||
      request.result_limits.maximum_chunk_count == 0 ||
      request.result_limits.maximum_chunk_bytes >
          request.result_limits.maximum_result_bytes) {
    return Result<std::vector<uint8_t>>::Failure(
        Error::InvalidArgument("invalid ABI-v2 plugin result limits"));
  }
  if (impl_->metadata.abi_minor >= 1) {
    const auto declared = std::find_if(
        impl_->metadata.operations.begin(), impl_->metadata.operations.end(),
        [&](const auto& operation) {
          return operation.operation == request.operation;
        });
    if (declared == impl_->metadata.operations.end()) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 operation is not declared by plugin metadata"));
    }
    if ((impl_->metadata.capability_bits &
         declared->required_capability_bits) !=
        declared->required_capability_bits) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 operation capability requirement is not satisfied"));
    }
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
  if (request.indexed_poses) {
    if ((impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2) == 0 &&
        (impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2) == 0) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 plugin does not advertise indexed-pose support"));
    }
    const auto& indexed = *request.indexed_poses;
    auto valid = ValidateNumericView(indexed.poses, 16, "indexed pose");
    if (!valid) return Result<std::vector<uint8_t>>::Failure(valid.GetError());
    if (indexed.frame_count != indexed.poses.count ||
        (indexed.frame_count != 0 && indexed.frame_ids == nullptr)) {
      return Result<std::vector<uint8_t>>::Failure(
          Error::InvalidArgument("invalid ABI-v2 indexed-pose frame IDs"));
    }
    std::unordered_set<uint64_t> unique;
    unique.reserve(static_cast<std::size_t>(indexed.frame_count));
    for (uint64_t i = 0; i < indexed.frame_count; ++i) {
      if (!unique.insert(indexed.frame_ids[i]).second) {
        return Result<std::vector<uint8_t>>::Failure(
            Error::InvalidArgument("duplicate ABI-v2 indexed-pose frame ID"));
      }
    }
  }
  if (request.frame_points) {
    if ((impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2) == 0 &&
        (impl_->metadata.capability_bits &
         OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2) == 0) {
      return Result<std::vector<uint8_t>>::Failure(Error::InvalidArgument(
          "ABI-v2 plugin does not advertise a remover execution mode"));
    }
    auto valid = ValidateNumericView(request.frame_points->points, 3,
                                     "frame point");
    if (!valid) return Result<std::vector<uint8_t>>::Failure(valid.GetError());
  }
  std::unique_lock<std::mutex> serialized(impl_->call_mutex, std::defer_lock);
  if (impl_->metadata.thread_safety !=
      OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2) {
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
  call.indexed_poses = Header<open_lmm_indexed_pose_view_v2>();
  call.indexed_poses.poses = Header<open_lmm_pose_view_v2>();
  call.indexed_poses.poses.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (request.indexed_poses) {
    const auto& source = *request.indexed_poses;
    call.indexed_poses.poses.data = source.poses.data;
    call.indexed_poses.poses.count = source.poses.count;
    call.indexed_poses.poses.stride_bytes = source.poses.stride_bytes;
    call.indexed_poses.poses.element_type = source.poses.element_type;
    call.indexed_poses.poses.endian = source.poses.endian;
    call.indexed_poses.frame_ids = source.frame_ids;
    call.indexed_poses.frame_count = source.frame_count;
  }
  call.frame_points = Header<open_lmm_frame_point_view_v2>();
  call.frame_points.points = Header<open_lmm_point_view_v2>();
  call.frame_points.points.memory_location = OPEN_LMM_MEMORY_HOST_V2;
  if (request.frame_points) {
    const auto& source = *request.frame_points;
    call.frame_points.frame_id = source.frame_id;
    call.frame_points.points.data = source.points.data;
    call.frame_points.points.count = source.points.count;
    call.frame_points.points.stride_bytes = source.points.stride_bytes;
    call.frame_points.points.element_type = source.points.element_type;
    call.frame_points.points.endian = source.points.endian;
  }
  SinkContext context;
  context.limits = request.result_limits;
  auto sink = Header<open_lmm_result_sink_v2>();
  sink.host_context = &context;
  sink.write = &WriteResult;
  open_lmm_status_v2 status{};
  try {
    if (request.is_cancelled) {
      std::unique_lock cancellation_lock(impl_->cancellation_mutex);
      impl_->active_cancellation_context.store(request.cancellation_context,
                                               std::memory_order_release);
      impl_->active_is_cancelled.store(request.is_cancelled,
                                       std::memory_order_release);
      struct ResetCancellation {
        Impl* impl;
        ~ResetCancellation() {
          impl->active_is_cancelled.store(nullptr, std::memory_order_release);
          impl->active_cancellation_context.store(nullptr,
                                                  std::memory_order_release);
        }
      } reset{impl_.get()};
      status = impl_->call(impl_->handle, &call, &sink);
    } else {
      std::shared_lock cancellation_lock(impl_->cancellation_mutex);
      status = impl_->call(impl_->handle, &call, &sink);
    }
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
  if (context.limit_exceeded) {
    return Result<std::vector<uint8_t>>::Failure(
        Error::PluginLoadFailed("ABI-v2 plugin result exceeded host limits")
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
                   descriptor.abi_minor,
                   DescriptorMinimumPrefix(descriptor.abi_minor)) ||
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
  impl->metadata.kind = std::move(kind).Value();
  impl->metadata.name = std::move(name).Value();
  impl->metadata.capability = std::move(capability).Value();
  impl->metadata.capability_bits = descriptor.capability_bits;
  impl->metadata.abi_minor = descriptor.abi_minor;
  if (descriptor.abi_minor >= 1) {
    auto plugin_id = CopyString(descriptor.plugin_id, "plugin_id");
    auto plugin_version = CopyString(descriptor.plugin_version,
                                     "plugin_version");
    auto schema_id = CopyString(descriptor.schema_id, "schema_id");
    if (!plugin_id) return Result<PluginV2>::Failure(plugin_id.GetError());
    if (!plugin_version)
      return Result<PluginV2>::Failure(plugin_version.GetError());
    if (!schema_id) return Result<PluginV2>::Failure(schema_id.GetError());
    if (plugin_id.Value().empty() || plugin_version.Value().empty() ||
        descriptor.operation_count > 128 ||
        (descriptor.operation_count != 0 && !descriptor.operations) ||
        descriptor.artifact_format_count > 64 ||
        (descriptor.artifact_format_count != 0 &&
         !descriptor.artifact_formats) ||
        descriptor.thread_safety >
            OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2 ||
        descriptor.cancellation > OPEN_LMM_CANCELLATION_COOPERATIVE_V2) {
      return Result<PluginV2>::Failure(Error::PluginLoadFailed(
          "malformed ABI-v2 minor-1 plugin metadata"));
    }
    impl->metadata.plugin_id = std::move(plugin_id).Value();
    impl->metadata.plugin_version = std::move(plugin_version).Value();
    impl->metadata.schema_id = std::move(schema_id).Value();
    impl->metadata.schema_version = descriptor.schema_version;
    impl->metadata.thread_safety = descriptor.thread_safety;
    impl->metadata.cancellation = descriptor.cancellation;
    const bool concurrent_metadata = descriptor.thread_safety ==
        OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2;
    const bool concurrent_capability =
        (descriptor.capability_bits &
         OPEN_LMM_CAPABILITY_CONCURRENT_CALLS_V2) != 0;
    if (concurrent_metadata != concurrent_capability) {
      return Result<PluginV2>::Failure(Error::PluginLoadFailed(
          "plugin thread-safety metadata contradicts capabilities"));
    }
    for (uint64_t index = 0; index < descriptor.operation_count; ++index) {
      const auto& operation = descriptor.operations[index];
      if (!ValidHeader(operation.struct_size, operation.abi_major,
                       operation.abi_minor, sizeof(operation)) ||
          (operation.required_capability_bits & descriptor.capability_bits) !=
              operation.required_capability_bits) {
        return Result<PluginV2>::Failure(Error::PluginLoadFailed(
            "malformed ABI-v2 operation metadata"));
      }
      auto copied = CopyString(operation.operation, "operation");
      if (!copied || copied.Value().empty()) {
        return Result<PluginV2>::Failure(
            copied ? Error::PluginLoadFailed("empty ABI-v2 operation")
                   : copied.GetError());
      }
      const bool duplicate = std::any_of(
          impl->metadata.operations.begin(), impl->metadata.operations.end(),
          [&](const auto& existing) {
            return existing.operation == copied.Value();
          });
      if (duplicate) {
        return Result<PluginV2>::Failure(
            Error::PluginLoadFailed("duplicate ABI-v2 operation metadata"));
      }
      impl->metadata.operations.push_back(
          {std::move(copied).Value(), operation.required_capability_bits});
    }
    for (uint64_t index = 0; index < descriptor.artifact_format_count;
         ++index) {
      const auto& format = descriptor.artifact_formats[index];
      if (!ValidHeader(format.struct_size, format.abi_major, format.abi_minor,
                       sizeof(format)) ||
          format.format_version == 0 || format.index_dimension == 0) {
        return Result<PluginV2>::Failure(
            Error::PluginLoadFailed("malformed ABI-v2 artifact format"));
      }
      auto format_id = CopyString(format.format_id, "artifact.format_id");
      if (!format_id || format_id.Value().empty()) {
        return Result<PluginV2>::Failure(
            format_id ? Error::PluginLoadFailed("empty artifact format ID")
                      : format_id.GetError());
      }
      impl->metadata.artifact_formats.push_back(
          {std::move(format_id).Value(), format.format_version,
           format.index_dimension});
    }
    if ((descriptor.capability_bits &
         OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0 &&
        (impl->metadata.schema_id.empty() || descriptor.schema_version == 0)) {
      return Result<PluginV2>::Failure(Error::PluginLoadFailed(
          "schema capability requires schema ID and version"));
    }
    if ((descriptor.capability_bits &
         OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0 &&
        std::none_of(impl->metadata.operations.begin(),
                     impl->metadata.operations.end(), [](const auto& op) {
          return op.operation == OPEN_LMM_PLUGIN_SCHEMA_OPERATION_V2 &&
                 (op.required_capability_bits &
                  OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0;
        })) {
      return Result<PluginV2>::Failure(Error::PluginLoadFailed(
          "schema capability requires plugin.schema operation metadata"));
    }
  } else {
    impl->metadata.plugin_id = impl->metadata.name;
    impl->metadata.plugin_version = "abi2.0-compat";
  }

  impl->default_cancellation_context = host_context;
  impl->default_is_cancelled = is_cancelled;
  impl->host = Header<open_lmm_host_api_v2>();
  impl->host.host_context = impl.get();
  impl->host.is_cancelled = &PluginV2::Impl::IsCancelled;
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
  PluginV2 loaded(std::move(impl));
  if ((loaded.Metadata().capability_bits &
       OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2) != 0) {
    PluginV2Call schema_call;
    schema_call.operation = OPEN_LMM_PLUGIN_SCHEMA_OPERATION_V2;
    schema_call.result_limits = {UINT64_C(64) * 1024,
                                 UINT64_C(64) * 1024, 16};
    auto schema = loaded.Call(schema_call);
    if (!schema) return Result<PluginV2>::Failure(schema.GetError());
    loaded.impl_->metadata.schema_fragment_json.assign(
        schema.Value().begin(), schema.Value().end());
    if (loaded.impl_->metadata.schema_fragment_json.empty()) {
      return Result<PluginV2>::Failure(
          Error::PluginLoadFailed("plugin schema fragment is empty"));
    }
  }
  return Result<PluginV2>::Ok(std::move(loaded));
}

}  // namespace open_lmm
