#ifndef OPEN_LMM_COMMON_PLUGIN_API_V2_H_
#define OPEN_LMM_COMMON_PLUGIN_API_V2_H_

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#define OPEN_LMM_PLUGIN_CALL_V2 __cdecl
#else
#define OPEN_LMM_PLUGIN_CALL_V2
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define OPEN_LMM_PLUGIN_ABI_V2_MAJOR 2u
#define OPEN_LMM_PLUGIN_ABI_V2_MINOR 0u
#define OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2 "open_lmm_plugin_query_v2"
#define OPEN_LMM_PLUGIN_OPEN_SYMBOL_V2 "open_lmm_plugin_open_v2"
#define OPEN_LMM_PLUGIN_CALL_SYMBOL_V2 "open_lmm_plugin_call_v2"
#define OPEN_LMM_PLUGIN_CLOSE_SYMBOL_V2 "open_lmm_plugin_close_v2"

#define OPEN_LMM_V2_HEADER_FIELDS \
  uint32_t struct_size;          \
  uint16_t abi_major;            \
  uint16_t abi_minor

typedef uint32_t open_lmm_status_code_v2;
#define OPEN_LMM_STATUS_OK_V2 UINT32_C(0)
#define OPEN_LMM_STATUS_INVALID_ARGUMENT_V2 UINT32_C(1)
#define OPEN_LMM_STATUS_INCOMPATIBLE_ABI_V2 UINT32_C(2)
#define OPEN_LMM_STATUS_UNSUPPORTED_CAPABILITY_V2 UINT32_C(3)
#define OPEN_LMM_STATUS_CANCELLED_V2 UINT32_C(4)
#define OPEN_LMM_STATUS_PLUGIN_ERROR_V2 UINT32_C(5)
#define OPEN_LMM_STATUS_HOST_ERROR_V2 UINT32_C(6)

typedef uint32_t open_lmm_element_type_v2;
#define OPEN_LMM_ELEMENT_F32_V2 UINT32_C(1)
#define OPEN_LMM_ELEMENT_F64_V2 UINT32_C(2)

typedef uint32_t open_lmm_endian_v2;
#define OPEN_LMM_ENDIAN_LITTLE_V2 UINT32_C(1)
#define OPEN_LMM_ENDIAN_BIG_V2 UINT32_C(2)

typedef uint32_t open_lmm_memory_location_v2;
#define OPEN_LMM_MEMORY_HOST_V2 UINT32_C(1)

#define OPEN_LMM_CAPABILITY_POINT_VIEW_V2 (UINT64_C(1) << 0)
#define OPEN_LMM_CAPABILITY_POSE_VIEW_V2 (UINT64_C(1) << 1)
#define OPEN_LMM_CAPABILITY_CONCURRENT_CALLS_V2 (UINT64_C(1) << 2)

#define OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2 "descriptor.make"

typedef struct open_lmm_string_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  const char* data;
  uint64_t size;
} open_lmm_string_view_v2;

typedef struct open_lmm_status_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_status_code_v2 code;
  open_lmm_string_view_v2 message;
} open_lmm_status_v2;

typedef struct open_lmm_blob_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  const void* data;
  uint64_t size;
  open_lmm_memory_location_v2 memory_location;
} open_lmm_blob_view_v2;

/* Point records are read using byte stride; x/y/z begin at offsets 0/width/2*width. */
typedef struct open_lmm_point_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  const void* data;
  uint64_t count;
  uint64_t stride_bytes;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
  open_lmm_memory_location_v2 memory_location;
} open_lmm_point_view_v2;

/* Row-major 4x4 transforms. stride_bytes separates consecutive transforms. */
typedef struct open_lmm_pose_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  const void* data;
  uint64_t count;
  uint64_t stride_bytes;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
  open_lmm_memory_location_v2 memory_location;
} open_lmm_pose_view_v2;

/* Binary payload header returned by descriptor.make. The header is followed
 * by rows*columns descriptor elements in row-major order and key_count key
 * elements, all using element_type/endian. */
typedef struct open_lmm_descriptor_result_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t rows;
  uint64_t columns;
  uint64_t key_count;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
} open_lmm_descriptor_result_v2;

typedef struct open_lmm_config_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 json;
} open_lmm_config_view_v2;

typedef int32_t(OPEN_LMM_PLUGIN_CALL_V2* open_lmm_is_cancelled_fn_v2)(
    void* host_context);

typedef struct open_lmm_host_api_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  void* host_context;
  open_lmm_is_cancelled_fn_v2 is_cancelled;
} open_lmm_host_api_v2;

typedef struct open_lmm_plugin_descriptor_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 plugin_kind;
  open_lmm_string_view_v2 plugin_name;
  open_lmm_string_view_v2 capability;
  uint64_t capability_bits;
  uint16_t minimum_host_minor;
  uint16_t reserved;
} open_lmm_plugin_descriptor_v2;

typedef struct open_lmm_call_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 operation;
  open_lmm_blob_view_v2 request;
  open_lmm_point_view_v2 points;
  open_lmm_pose_view_v2 poses;
} open_lmm_call_v2;

typedef open_lmm_status_code_v2(OPEN_LMM_PLUGIN_CALL_V2*
                                    open_lmm_result_write_fn_v2)(
    void* host_context, const open_lmm_blob_view_v2* chunk);

typedef struct open_lmm_result_sink_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  void* host_context;
  open_lmm_result_write_fn_v2 write;
} open_lmm_result_sink_v2;

typedef struct open_lmm_plugin_handle_v2 open_lmm_plugin_handle_v2;

typedef open_lmm_status_v2(OPEN_LMM_PLUGIN_CALL_V2*
                               open_lmm_plugin_query_fn_v2)(
    open_lmm_plugin_descriptor_v2* descriptor);
typedef open_lmm_status_v2(OPEN_LMM_PLUGIN_CALL_V2* open_lmm_plugin_open_fn_v2)(
    const open_lmm_host_api_v2* host, const open_lmm_config_view_v2* config,
    open_lmm_plugin_handle_v2** out);
typedef open_lmm_status_v2(OPEN_LMM_PLUGIN_CALL_V2* open_lmm_plugin_call_fn_v2)(
    open_lmm_plugin_handle_v2* handle, const open_lmm_call_v2* call,
    open_lmm_result_sink_v2* result_sink);
typedef void(OPEN_LMM_PLUGIN_CALL_V2* open_lmm_plugin_close_fn_v2)(
    open_lmm_plugin_handle_v2* handle);

/* host and its callbacks remain valid until close. Input views are borrowed
 * only for the duration of open/call. Sink writes copy into
 * host storage before returning. A non-NULL handle returned on open failure is
 * partially initialized and must be accepted by close. close is noexcept; the
 * host calls it at most once for each non-NULL handle. */

#ifdef __cplusplus
}
#endif

#endif
