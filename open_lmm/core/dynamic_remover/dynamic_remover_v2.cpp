#include "dynamic_remover_v2.hpp"

#include <dlfcn.h>

#include <open_lmm/common/validation.hpp>
#include <open_lmm/core/algorithm_invariants.hpp>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <unordered_map>

namespace open_lmm {
namespace {

class AbortGuard {
 public:
  explicit AbortGuard(std::shared_ptr<PluginV2> plugin)
      : plugin_(std::move(plugin)) {}
  ~AbortGuard() {
    if (!active_) return;
    try {
      PluginV2Call abort{OPEN_LMM_REMOVER_ABORT_OPERATION_V2};
      (void)plugin_->Call(abort);
    } catch (...) {
    }
  }
  void Arm() noexcept { active_ = true; }
  void Commit() noexcept { active_ = false; }

 private:
  std::shared_ptr<PluginV2> plugin_;
  bool active_ = false;
};

open_lmm_endian_v2 NativeEndian() {
  const uint16_t value = 1;
  return *reinterpret_cast<const uint8_t*>(&value) == 1
             ? OPEN_LMM_ENDIAN_LITTLE_V2
             : OPEN_LMM_ENDIAN_BIG_V2;
}

bool HasV2Symbols(const std::string& path) {
  void* library = dlopen(path.c_str(), RTLD_NOW | RTLD_LOCAL);
  if (!library) return false;
  const bool complete =
      dlsym(library, OPEN_LMM_PLUGIN_QUERY_SYMBOL_V2) &&
      dlsym(library, OPEN_LMM_PLUGIN_OPEN_SYMBOL_V2) &&
      dlsym(library, OPEN_LMM_PLUGIN_CALL_SYMBOL_V2) &&
      dlsym(library, OPEN_LMM_PLUGIN_CLOSE_SYMBOL_V2);
  dlclose(library);
  return complete;
}

bool HasCanonicalRemoverOperations(const PluginV2Metadata& metadata) {
  const auto matches = [&](std::string_view name, uint64_t required_bits) {
    return std::count_if(metadata.operations.begin(), metadata.operations.end(),
                         [&](const auto& operation) {
                           return operation.operation == name &&
                                  operation.required_capability_bits ==
                                      required_bits;
                         }) == 1;
  };
  return matches(OPEN_LMM_REMOVER_BEGIN_OPERATION_V2,
                 OPEN_LMM_CAPABILITY_POSE_VIEW_V2) &&
         matches(OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2,
                 OPEN_LMM_CAPABILITY_POINT_VIEW_V2) &&
         matches(OPEN_LMM_REMOVER_FINISH_OPERATION_V2, 0) &&
         matches(OPEN_LMM_REMOVER_ABORT_OPERATION_V2, 0);
}

Result<DynamicRemoverV2::PointCloud::Ptr> DecodeStaticCloud(
    const std::vector<uint8_t>& bytes) {
  if (bytes.size() < sizeof(open_lmm_point_cloud_header_v2)) {
    return Result<DynamicRemoverV2::PointCloud::Ptr>::Failure(
        Error::PluginLoadFailed("remover.finish returned a truncated cloud"));
  }
  open_lmm_point_cloud_header_v2 header{};
  std::memcpy(&header, bytes.data(), sizeof(header));
  if (header.struct_size < sizeof(header) ||
      header.abi_major != OPEN_LMM_PLUGIN_ABI_V2_MAJOR ||
      header.component_count != 4 ||
      header.element_type != OPEN_LMM_ELEMENT_F32_V2 ||
      header.endian != OPEN_LMM_ENDIAN_LITTLE_V2 ||
      header.stride_bytes != 4 * sizeof(float) ||
      header.count > (std::numeric_limits<std::size_t>::max() - sizeof(header)) /
                         header.stride_bytes ||
      sizeof(header) + header.count * header.stride_bytes != bytes.size()) {
    return Result<DynamicRemoverV2::PointCloud::Ptr>::Failure(
        Error::PluginLoadFailed("remover.finish returned a malformed cloud"));
  }
  auto cloud = std::make_shared<DynamicRemoverV2::PointCloud>();
  cloud->reserve(static_cast<std::size_t>(header.count));
  const uint8_t* record = bytes.data() + sizeof(header);
  for (uint64_t i = 0; i < header.count; ++i, record += header.stride_bytes) {
    float values[4];
    std::memcpy(values, record, sizeof(values));
    if (!std::isfinite(values[0]) || !std::isfinite(values[1]) ||
        !std::isfinite(values[2]) || !std::isfinite(values[3])) {
      return Result<DynamicRemoverV2::PointCloud::Ptr>::Failure(
          Error::PluginLoadFailed("remover.finish returned a non-finite point"));
    }
    pcl::PointXYZI point;
    point.x = values[0];
    point.y = values[1];
    point.z = values[2];
    point.intensity = values[3];
    cloud->push_back(point);
  }
  return Result<DynamicRemoverV2::PointCloud::Ptr>::Ok(std::move(cloud));
}

}  // namespace

DynamicRemoverV2::DynamicRemoverV2(std::shared_ptr<PluginV2> plugin,
                                   uint64_t required_mode_capability)
    : plugin_(std::move(plugin)),
      required_mode_capability_(required_mode_capability) {}

Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>
DynamicRemoverV2::TryLoad(const std::string& shared_library,
                          std::string_view config_json,
                          uint64_t required_mode_capability) {
  if (!HasV2Symbols(shared_library)) {
    return Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>::Ok(
        std::nullopt);
  }
  auto loaded = LoadPluginV2(shared_library, "dynamic_remover", config_json,
                             OPEN_LMM_CAPABILITY_POINT_VIEW_V2 |
                                 OPEN_LMM_CAPABILITY_POSE_VIEW_V2);
  if (!loaded) {
    return Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>::Failure(
        loaded.GetError());
  }
  auto plugin = std::make_shared<PluginV2>(std::move(loaded).Value());
  if (!HasCanonicalRemoverOperations(plugin->Metadata())) {
    return Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>::Failure(
        Error::PluginLoadFailed(
            "dynamic remover ABI-v2 operation contract is incomplete"));
  }
  if ((plugin->Metadata().capability_bits & required_mode_capability) == 0) {
    return Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>::Ok(
        std::nullopt);
  }
  return Result<std::optional<std::shared_ptr<DynamicRemoverV2>>>::Ok(
      std::make_shared<DynamicRemoverV2>(std::move(plugin),
                                         required_mode_capability));
}

Result<DynamicRemoverBase::PointCloud::Ptr> DynamicRemoverV2::Process(
    const AlgorithmExecutionContext& context, DynamicRemoverInput input) {
  std::vector<IndexedScan> scans;
  scans.reserve(input.scans.size());
  for (std::size_t i = 0; i < input.scans.size(); ++i) {
    scans.push_back({static_cast<uint64_t>(i), std::move(input.scans[i])});
  }
  return Execute(context, std::move(scans), input.optimized_poses);
}

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverV2::ProcessStreaming(
    const AlgorithmExecutionContext& context,
    const DynamicRemoverStreamingInput& input) {
  std::vector<IndexedScan> scans;
  std::set<uint64_t> seen;
  const auto visitor = [&](std::size_t frame_id,
                           const PointCloud::Ptr& cloud) {
    auto active = CheckAlgorithmCancellation(
        context, "while buffering remover ABI-v2 input");
    if (!active) return Result<void>::Failure(active.GetError());
    if (!seen.insert(frame_id).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "dynamic remover source returned duplicate frame " +
          std::to_string(frame_id)));
    }
    auto valid = ValidatePointCloud(
        cloud, "dynamic remover source frame " + std::to_string(frame_id));
    if (!valid) return valid;
    scans.push_back({static_cast<uint64_t>(frame_id), cloud});
    return Result<void>::Ok();
  };
  Result<std::size_t> loaded = Result<std::size_t>::Failure(
      Error::IoError("dynamic remover source did not run"));
  try {
    loaded = input.source(visitor);
  } catch (const std::exception& exception) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::IoError(std::string("dynamic remover source threw: ") +
                       exception.what()),
        context));
  } catch (...) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::IoError("dynamic remover source threw an unknown exception"),
        context));
  }
  if (!loaded) return Result<PointCloud::Ptr>::Failure(loaded.GetError());
  if (loaded.Value() != scans.size()) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "dynamic remover source count differs from visited scan count"));
  }
  std::shared_ptr<void> heavy_phase;
  if (required_mode_capability_ == OPEN_LMM_CAPABILITY_REMOVER_BATCH_V2 &&
      input.heavy_phase_admission) {
    auto admitted = input.heavy_phase_admission();
    if (!admitted) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(admitted.GetError(), context));
    }
    heavy_phase = std::move(admitted).Value();
  }
  return Execute(context, std::move(scans), input.optimized_poses);
}

Result<DynamicRemoverBase::PointCloud::Ptr> DynamicRemoverV2::Execute(
    const AlgorithmExecutionContext& context, std::vector<IndexedScan> scans,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses) {
  AlgorithmExecutionTimer timer(context);
  if (!plugin_ ||
      (plugin_->Metadata().capability_bits & required_mode_capability_) == 0) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("dynamic remover ABI-v2 capability mismatch"),
        context));
  }

  std::unordered_map<uint64_t, Eigen::Isometry3d> poses;
  for (const auto& [signed_id, pose] : optimized_poses) {
    if (signed_id < 0 ||
        !poses.emplace(static_cast<uint64_t>(signed_id), pose).second ||
        !IsFiniteRigidTransform(pose)) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("invalid or duplicate remover pose frame"),
          context));
    }
  }
  std::sort(scans.begin(), scans.end(), [](const auto& a, const auto& b) {
    return a.frame_id < b.frame_id;
  });
  if (scans.size() != poses.size()) {
    return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
        Error::InvalidArgument("remover scan/pose frame count mismatch"),
        context));
  }
  std::vector<uint64_t> frame_ids;
  std::vector<double> matrices;
  frame_ids.reserve(scans.size());
  matrices.reserve(scans.size() * 16);
  for (std::size_t i = 0; i < scans.size(); ++i) {
    if ((i != 0 && scans[i - 1].frame_id == scans[i].frame_id) ||
        !scans[i].cloud || poses.find(scans[i].frame_id) == poses.end()) {
      return Result<PointCloud::Ptr>::Failure(WithAlgorithmContext(
          Error::InvalidArgument("missing, duplicate, or extra remover frame"),
          context));
    }
    auto valid = ValidatePointCloud(scans[i].cloud, "remover ABI-v2 scan");
    if (!valid) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(valid.GetError(), context));
    }
    frame_ids.push_back(scans[i].frame_id);
    const Eigen::Matrix4d matrix = poses.at(scans[i].frame_id).matrix();
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) matrices.push_back(matrix(row, col));
    }
  }

  auto active = CheckAlgorithmCancellation(context, "before remover.begin");
  if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
  AbortGuard abort(plugin_);
  // Both capability modes use the frame-aware begin/push/finish protocol.
  // BATCH means the host has validated and buffered the complete frame set
  // before begin; STREAMING permits incremental plugin processing per push.
  PluginV2Call begin{OPEN_LMM_REMOVER_BEGIN_OPERATION_V2};
  begin.indexed_poses = PluginIndexedPoseView{
      PluginPoseView{matrices.data(), static_cast<uint64_t>(frame_ids.size()),
                     16 * sizeof(double), OPEN_LMM_ELEMENT_F64_V2,
                     NativeEndian()},
      frame_ids.data(), static_cast<uint64_t>(frame_ids.size())};
  abort.Arm();
  auto begun = plugin_->Call(begin);
  if (!begun) {
    return Result<PointCloud::Ptr>::Failure(
        WithAlgorithmContext(begun.GetError(), context));
  }
  for (const auto& scan : scans) {
    active = CheckAlgorithmCancellation(context, "before remover.push_scan");
    if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
    PluginV2Call push{OPEN_LMM_REMOVER_PUSH_SCAN_OPERATION_V2};
    std::vector<float> packed_points;
    packed_points.reserve(scan.cloud->size() * 4);
    for (const auto& point : *scan.cloud) {
      packed_points.insert(packed_points.end(),
                           {point.x, point.y, point.z, point.intensity});
    }
    push.frame_points = PluginFramePointView{
        scan.frame_id,
        PluginPointView{packed_points.data(),
                        static_cast<uint64_t>(scan.cloud->size()),
                        4 * sizeof(float), OPEN_LMM_ELEMENT_F32_V2,
                        NativeEndian()}};
    auto pushed = plugin_->Call(push);
    if (!pushed) {
      return Result<PointCloud::Ptr>::Failure(
          WithAlgorithmContext(pushed.GetError(), context));
    }
  }
  active = CheckAlgorithmCancellation(context, "before remover.finish");
  if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
  PluginV2Call finish{OPEN_LMM_REMOVER_FINISH_OPERATION_V2};
  auto finished = plugin_->Call(finish);
  if (!finished) {
    return Result<PointCloud::Ptr>::Failure(
        WithAlgorithmContext(finished.GetError(), context));
  }
  auto decoded = DecodeStaticCloud(finished.Value());
  if (!decoded) {
    return Result<PointCloud::Ptr>::Failure(
        WithAlgorithmContext(decoded.GetError(), context));
  }
  active = CheckAlgorithmCancellation(context, "after remover.finish");
  if (!active) return Result<PointCloud::Ptr>::Failure(active.GetError());
  abort.Commit();
  return decoded;
}

}  // namespace open_lmm
