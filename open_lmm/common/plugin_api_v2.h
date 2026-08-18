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
#define OPEN_LMM_PLUGIN_ABI_V2_MINOR 1u
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
#define OPEN_LMM_CAPABILITY_DESCRIPTOR_MAKE_V2 (UINT64_C(1) << 3)
#define OPEN_LMM_CAPABILITY_DESCRIPTOR_COMPARE_V2 (UINT64_C(1) << 4)
#define OPEN_LMM_CAPABILITY_DESCRIPTOR_INDEX_KEY_V2 (UINT64_C(1) << 5)
#define OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2 (UINT64_C(1) << 6)
#define OPEN_LMM_CAPABILITY_REMOVER_STREAMING_V2 (UINT64_C(1) << 7)
#define OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2 (UINT64_C(1) << 8)

#define OPEN_LMM_DESCRIPTOR_MAKE_OPERATION_V2 "descriptor.make"
#define OPEN_LMM_DESCRIPTOR_COMPARE_OPERATION_V2 "descriptor.compare"
#define OPEN_LMM_DESCRIPTOR_INDEX_KEY_OPERATION_V2 "descriptor.index_key"
#define OPEN_LMM_PLUGIN_SCHEMA_OPERATION_V2 "plugin.schema"
#define OPEN_LMM_REMOVER_BEGIN_OPERATION_V2 "remover.begin"
#define OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2 "remover.push_scan"
#define OPEN_LMM_REMOVER_FINISH_OPERATION_V2 "remover.finish"
#define OPEN_LMM_REMOVER_ABORT_OPERATION_V2 "remover.abort"

typedef uint32_t open_lmm_thread_safety_v2;
#define OPEN_LMM_THREAD_SAFETY_HANDLE_SERIALIZED_V2 UINT32_C(0)
#define OPEN_LMM_THREAD_SAFETY_CONCURRENT_CALLS_V2 UINT32_C(1)

typedef uint32_t open_lmm_cancellation_mode_v2;
#define OPEN_LMM_CANCELLATION_NONE_V2 UINT32_C(0)
#define OPEN_LMM_CANCELLATION_COOPERATIVE_V2 UINT32_C(1)

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

/* frame_ids and poses are borrowed for the duration of call. frame_count must
 * equal poses.count and frame IDs must be unique. */
typedef struct open_lmm_indexed_pose_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_pose_view_v2 poses;
  const uint64_t* frame_ids;
  uint64_t frame_count;
} open_lmm_indexed_pose_view_v2;

typedef struct open_lmm_frame_point_view_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t frame_id;
  open_lmm_point_view_v2 points;
} open_lmm_frame_point_view_v2;

/* Pointer-free sink header followed by count records of stride_bytes. Remover
 * static-cloud results use four scalar components in XYZI order. */
typedef struct open_lmm_point_cloud_header_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t count;
  uint64_t stride_bytes;
  uint32_t component_count;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
  uint32_t reserved;
} open_lmm_point_cloud_header_v2;

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

/* Pointer-free descriptor wire records. Variable bytes immediately follow
 * each header in the order documented by the fields. */
typedef struct open_lmm_descriptor_artifact_header_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t format_id_size;
  uint32_t format_version;
  uint32_t reserved;
  uint64_t key_count;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
  uint64_t payload_size;
} open_lmm_descriptor_artifact_header_v2;

typedef struct open_lmm_descriptor_compare_request_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t format_id_size;
  uint32_t format_version;
  uint32_t reserved;
  uint64_t lhs_payload_size;
  uint64_t rhs_payload_size;
} open_lmm_descriptor_compare_request_v2;

typedef struct open_lmm_descriptor_compare_result_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  double score;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
  double relative_pose_row_major[16];
} open_lmm_descriptor_compare_result_v2;

typedef struct open_lmm_descriptor_index_key_header_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  uint64_t key_count;
  open_lmm_element_type_v2 element_type;
  open_lmm_endian_v2 endian;
} open_lmm_descriptor_index_key_header_v2;

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
  /* ABI minor 1 suffix. Every view is borrowed until query returns. */
  open_lmm_string_view_v2 plugin_id;
  open_lmm_string_view_v2 plugin_version;
  const struct open_lmm_operation_descriptor_v2* operations;
  uint64_t operation_count;
  open_lmm_string_view_v2 schema_id;
  uint32_t schema_version;
  open_lmm_thread_safety_v2 thread_safety;
  open_lmm_cancellation_mode_v2 cancellation;
  uint32_t reserved_v1;
  const struct open_lmm_artifact_format_v2* artifact_formats;
  uint64_t artifact_format_count;
} open_lmm_plugin_descriptor_v2;

typedef struct open_lmm_operation_descriptor_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 operation;
  uint64_t required_capability_bits;
} open_lmm_operation_descriptor_v2;

typedef struct open_lmm_artifact_format_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 format_id;
  uint32_t format_version;
  uint32_t reserved;
  uint64_t index_dimension;
} open_lmm_artifact_format_v2;

#define OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_0_SIZE \
  ((uint32_t)offsetof(open_lmm_plugin_descriptor_v2, plugin_id))
#define OPEN_LMM_PLUGIN_DESCRIPTOR_V2_MINOR_1_SIZE \
  ((uint32_t)sizeof(open_lmm_plugin_descriptor_v2))

typedef struct open_lmm_call_v2 {
  OPEN_LMM_V2_HEADER_FIELDS;
  open_lmm_string_view_v2 operation;
  open_lmm_blob_view_v2 request;
  open_lmm_point_view_v2 points;
  open_lmm_pose_view_v2 poses;
  /* ABI minor 1 suffix. */
  open_lmm_indexed_pose_view_v2 indexed_poses;
  open_lmm_frame_point_view_v2 frame_points;
} open_lmm_call_v2;

#define OPEN_LMM_CALL_V2_MINOR_0_SIZE \
  ((uint32_t)offsetof(open_lmm_call_v2, indexed_poses))
#define OPEN_LMM_CALL_V2_MINOR_1_SIZE ((uint32_t)sizeof(open_lmm_call_v2))

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
